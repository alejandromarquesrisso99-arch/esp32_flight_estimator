#include "gnc_task.hpp"
#include "telemetry_task.hpp"
#include "mpu6050_driver.hpp"
#include "drdy_sync.hpp"
#include "timer_watchdog.hpp"
#include "fdir_manager.hpp"
#include "ekf.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_cpu.h"
#include <cmath>

static const char* TAG = "GNC_CORE1";

namespace flight {

// FreeRTOS Static Allocation Buffers (Zero-Heap Policy)
static constexpr size_t GNC_STACK_SZ = 8192;
static StackType_t   s_gnc_stack[GNC_STACK_SZ];
static StaticTask_t  s_gnc_tcb;
static TaskHandle_t  s_gnc_task_handle = nullptr;

// 7-State Extended Kalman Filter (Static instance)
static ExtendedKalmanFilter s_ekf;

// Metrics & Diagnostics
static uint32_t s_max_wcet_cycles = 0;
static float    s_loop_freq_hz    = 0.0f;
static uint64_t s_last_loop_us    = 0;

TaskHandle_t gnc_task_get_handle() {
    return s_gnc_task_handle;
}

void gnc_task_init() {
    // 1. Create static FreeRTOS task pinned to Core 1 (Priority 24 - Hard Real-Time)
    s_gnc_task_handle = xTaskCreateStaticPinnedToCore(
        gnc_task_run,
        "GncTask",
        GNC_STACK_SZ,
        nullptr,
        24,                   // Priority 24 (Maximum Hard Real-Time Domain)
        s_gnc_stack,
        &s_gnc_tcb,
        1                     // Strictly pinned to Core 1
    );
    assert(s_gnc_task_handle != nullptr);

    ESP_LOGI(TAG, "GNC Task initialized on Core 1 (Priority 24, Zero-Heap Static Allocation)");
}

void gnc_task_run(void* pvParameters) {
    (void)pvParameters;
    FlightProfileId current_profile = FlightProfileId::UNKNOWN;

    ESP_LOGI(TAG, "GNC Core 1 loop running, awaiting system start...");

    while (true) {
        // Wait for synchronization notification from DRDY ISR or GPTimer Watchdog Alarm (20ms timeout fail-safe)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        // Only process EKF when the vehicle is in RUNNING_ESTIMATOR state
        if (g_system_state != SystemState::RUNNING_ESTIMATOR) {
            continue;
        }

        // Initialize EKF profile and FDIR when transition to RUNNING_ESTIMATOR first occurs
        if (current_profile != g_active_profile) {
            current_profile = g_active_profile;
            s_ekf.setProfile(current_profile);
            FDIRManager::init(current_profile);
            s_last_loop_us = esp_timer_get_time();
            s_max_wcet_cycles = 0;

            const auto* p_cfg = get_profile_config(current_profile);
            if (p_cfg != nullptr) {
                s_loop_freq_hz = static_cast<float>(p_cfg->rate_hz);
                // Initialize DRDY and GPTimer watchdog
                drivers::drdy_sync_init(drivers::DEFAULT_DRDY_GPIO, s_gnc_task_handle);
                timer_watchdog_init(current_profile, s_gnc_task_handle);
                timer_watchdog_start();
            }
            ESP_LOGI(TAG, "EKF 7D & FDIR initialized for profile: %u (%s)", 
                     static_cast<unsigned>(current_profile),
                     get_profile_config(current_profile)->name);
        }

        // 1. Read burst data from MPU6050 (14 bytes atomically over I2C)
        drivers::InertialRawData raw{};
        drivers::InertialScaledData scaled{};
        esp_err_t err = drivers::MPU6050Driver::read_burst_raw(raw);
        if (err != ESP_OK) {
            if (FDIRManager::register_i2c_error(g_health_flags)) {
                g_system_state = SystemState::HARD_FAULT_LOCK;
            }
            continue;
        }

        FDIRManager::register_i2c_success();
        g_health_flags |= HEALTH_FLAG_IMU_OK;

        // Scale data and subtract calibrated gyro biases
        drivers::MPU6050Driver::scale_data(raw, scaled);

        // 2. Feed hardware GPTimer watchdog
        timer_watchdog_feed();

        // 3. High-resolution delta time computation
        uint64_t now_us = esp_timer_get_time();
        float dt = static_cast<float>(now_us - s_last_loop_us) * 1e-6f;
        s_last_loop_us = now_us;
        const auto* p_cfg = get_profile_config(current_profile);
        float nominal_dt = (p_cfg != nullptr && p_cfg->rate_hz > 0) ? (1.0f / p_cfg->rate_hz) : 0.002f;
        if (dt <= 0.0f || dt > nominal_dt * 3.0f) {
            dt = nominal_dt;
        }

        // 4. FDIR Health & Dynamic bounds checking
        FDIRManager::process_sample(scaled, dt, g_health_flags);

        // 5. Benchmark WCET with hardware CPU cycle counter
        uint32_t cycle_start = esp_cpu_get_cycle_count();

        // 6. EKF Predict Step: propagate quaternion kinematics and covariance P
        Vector3f gyro_vec{scaled.gyro_rads[0], scaled.gyro_rads[1], scaled.gyro_rads[2]};
        s_ekf.predict(gyro_vec, dt);

        // 7. EKF Update Step: gated accelerometer gravity correction
        if (FDIRManager::should_fuse_accelerometer(scaled, g_health_flags)) {
            Vector3f accel_vec{scaled.accel_mss[0], scaled.accel_mss[1], scaled.accel_mss[2]};
            s_ekf.update(accel_vec);
        }

        uint32_t cycle_end = esp_cpu_get_cycle_count();
        uint32_t elapsed_cycles = (cycle_end >= cycle_start) ? (cycle_end - cycle_start) : 0;
        if (elapsed_cycles > s_max_wcet_cycles) {
            s_max_wcet_cycles = elapsed_cycles;
        }

        // 7. Calculate Euler angles from normalized quaternion
        float q0 = s_ekf.x(0), q1 = s_ekf.x(1), q2 = s_ekf.x(2), q3 = s_ekf.x(3);
        float sin_p = 2.0f * (q0 * q2 - q3 * q1);
        if (sin_p > 1.0f)  sin_p = 1.0f;
        if (sin_p < -1.0f) sin_p = -1.0f;
        float pitch_deg = std::asin(sin_p) * PhysicsConstants::RAD_TO_DEG;

        float roll_deg = 0.0f;
        if (std::abs(sin_p) < 0.999f) {
            roll_deg = std::atan2(2.0f * (q0 * q1 + q2 * q3), 
                                  1.0f - 2.0f * (q1 * q1 + q2 * q2)) * PhysicsConstants::RAD_TO_DEG;
        }

        float yaw_deg = std::atan2(2.0f * (q0 * q3 + q1 * q2), 
                                   1.0f - 2.0f * (q2 * q2 + q3 * q3)) * PhysicsConstants::RAD_TO_DEG;

        // 8. Track loop frequency
        float instant_freq = 1.0f / dt;
        s_loop_freq_hz = (s_loop_freq_hz * 0.95f) + (instant_freq * 0.05f);

        // 9. Prepare and emit telemetry packet to Core 0 (Lock-Free Queue Overwrite)
        if (g_telemetry_queue != nullptr) {
            protocol::PayloadTelemetry payload{};
            payload.timestamp_us      = static_cast<uint32_t>(now_us);
            payload.q[0]              = q0;
            payload.q[1]              = q1;
            payload.q[2]              = q2;
            payload.q[3]              = q3;
            payload.euler_deg[0]      = roll_deg;
            payload.euler_deg[1]      = pitch_deg;
            payload.euler_deg[2]      = yaw_deg;
            payload.gyro_dps[0]       = scaled.gyro_dps[0];
            payload.gyro_dps[1]       = scaled.gyro_dps[1];
            payload.gyro_dps[2]       = scaled.gyro_dps[2];
            payload.accel_g[0]        = scaled.accel_g[0];
            payload.accel_g[1]        = scaled.accel_g[1];
            payload.accel_g[2]        = scaled.accel_g[2];
            payload.wcet_cycles       = s_max_wcet_cycles;
            payload.wcet_us           = static_cast<float>(s_max_wcet_cycles) / 240.0f;
            payload.loop_freq_hz      = s_loop_freq_hz;
            payload.health_flags      = g_health_flags;
            payload.system_state      = static_cast<uint8_t>(g_system_state);
            payload.active_profile_id = static_cast<uint8_t>(g_active_profile);

            xQueueOverwrite(g_telemetry_queue, &payload);
        }
    }
}

} // namespace flight
