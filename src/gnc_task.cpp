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

// Búferes de asignación estática de FreeRTOS (Política Zero-Heap)
static constexpr size_t GNC_STACK_SZ = 8192;
static StackType_t   s_gnc_stack[GNC_STACK_SZ];
static StaticTask_t  s_gnc_tcb;
static TaskHandle_t  s_gnc_task_handle = nullptr;

// Instancia estática del Filtro de Kalman Extendido (EKF 7D)
static ExtendedKalmanFilter s_ekf;

// Métricas de ejecución y diagnóstico
static uint32_t s_max_wcet_cycles = 0;
static float    s_loop_freq_hz    = 0.0f;
static uint64_t s_last_loop_us    = 0;

TaskHandle_t gnc_task_get_handle() {
    return s_gnc_task_handle;
}

void gnc_task_init() {
    // 1. Crear tarea estática de FreeRTOS anclada al Núcleo 1 (Prioridad 24 - Tiempo Real Duro)
    s_gnc_task_handle = xTaskCreateStaticPinnedToCore(
        gnc_task_run,
        "GncTask",
        GNC_STACK_SZ,
        nullptr,
        24,                   // Prioridad 24 (Máximo Dominio de Tiempo Real)
        s_gnc_stack,
        &s_gnc_tcb,
        1                     // Estrictamente anclada al Núcleo 1
    );
    assert(s_gnc_task_handle != nullptr);

    ESP_LOGI(TAG, "Tarea GNC inicializada en Nucleo 1 (Prioridad 24, Asignacion Estatica Zero-Heap)");
}

void gnc_task_run(void* pvParameters) {
    (void)pvParameters;
    FlightProfileId current_profile = FlightProfileId::UNKNOWN;

    ESP_LOGI(TAG, "Bucle GNC en Nucleo 1 en ejecucion, esperando inicio del sistema...");

    while (true) {
        // Esperar notificación de sincronización de la ISR DRDY o de la alarma del Watchdog GPTimer (timeout seguro de 20ms)
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));

        // Procesar EKF exclusivamente cuando el vehículo esté en estado RUNNING_ESTIMATOR
        if (g_system_state != SystemState::RUNNING_ESTIMATOR) {
            continue;
        }

        // Inicializar perfil de EKF y FDIR cuando ocurra la primera transición a RUNNING_ESTIMATOR
        if (current_profile != g_active_profile) {
            current_profile = g_active_profile;
            s_ekf.setProfile(current_profile);
            FDIRManager::init(current_profile);
            s_last_loop_us = esp_timer_get_time();
            s_max_wcet_cycles = 0;

            const auto* p_cfg = get_profile_config(current_profile);
            if (p_cfg != nullptr) {
                s_loop_freq_hz = static_cast<float>(p_cfg->rate_hz);
                // Inicializar sincronización DRDY y watchdog GPTimer
                drivers::drdy_sync_init(drivers::DEFAULT_DRDY_GPIO, s_gnc_task_handle);
                timer_watchdog_init(current_profile, s_gnc_task_handle);
                timer_watchdog_start();
            }
            ESP_LOGI(TAG, "EKF 7D y FDIR inicializados para perfil: %u (%s)", 
                     static_cast<unsigned>(current_profile),
                     get_profile_config(current_profile)->name);
        }

        // 1. Lectura en ráfaga del MPU6050 (14 bytes atómicos por I2C)
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
        g_health_flags.fetch_or(HEALTH_FLAG_IMU_OK, std::memory_order_relaxed);

        // Escalar datos y sustraer sesgos calibrados
        drivers::MPU6050Driver::scale_data(raw, scaled);

        // 2. Alimentar watchdog hardware GPTimer
        timer_watchdog_feed();

        // 3. Cálculo de delta de tiempo de alta resolución
        uint64_t now_us = esp_timer_get_time();
        float dt = static_cast<float>(now_us - s_last_loop_us) * 1e-6f;
        s_last_loop_us = now_us;
        const auto* p_cfg = get_profile_config(current_profile);
        float nominal_dt = (p_cfg != nullptr && p_cfg->rate_hz > 0) ? (1.0f / p_cfg->rate_hz) : 0.002f;
        if (dt <= 0.0f || dt > nominal_dt * 3.0f) {
            dt = nominal_dt;
        }

        // 4. Verificación de salud y límites dinámicos FDIR
        FDIRManager::process_sample(scaled, dt, g_health_flags);

        // 5. Medición de WCET con contador de ciclos de CPU hardware
        uint32_t cycle_start = esp_cpu_get_cycle_count();

        // 6. Paso de Predicción del EKF: propagación cinemática del cuaternión y covarianza P
        Vector3f gyro_vec{scaled.gyro_rads[0], scaled.gyro_rads[1], scaled.gyro_rads[2]};
        s_ekf.predict(gyro_vec, dt);

        // 7. Paso de Actualización del EKF: corrección por vector de gravedad con G-Gating
        if (FDIRManager::should_fuse_accelerometer(scaled, g_health_flags)) {
            Vector3f accel_vec{scaled.accel_mss[0], scaled.accel_mss[1], scaled.accel_mss[2]};
            s_ekf.update(accel_vec);
        }

        uint32_t cycle_end = esp_cpu_get_cycle_count();
        uint32_t elapsed_cycles = cycle_end - cycle_start;
        if (elapsed_cycles > s_max_wcet_cycles) {
            s_max_wcet_cycles = elapsed_cycles;
        }

        // 8. Cálculo de ángulos de Euler a partir del cuaternión normalizado
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

        // 9. Seguimiento de frecuencia de ejecución del lazo
        float instant_freq = 1.0f / dt;
        s_loop_freq_hz = (s_loop_freq_hz * 0.95f) + (instant_freq * 0.05f);

        // 10. Preparar y emitir paquete de telemetría al Núcleo 0 (Sobrescritura sin bloqueo)
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
            payload.health_flags      = g_health_flags.load(std::memory_order_relaxed);
            payload.system_state      = static_cast<uint8_t>(g_system_state);
            payload.active_profile_id = static_cast<uint8_t>(g_active_profile);

            xQueueOverwrite(g_telemetry_queue, &payload);
        }
    }
}

} // namespace flight
