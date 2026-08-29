#include "fdir_manager.hpp"
#include <cmath>

#if __has_include("esp_log.h")
#include "esp_log.h"
static const char* TAG = "FDIR";
#else
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#endif

namespace flight {

FlightProfileId FDIRManager::s_active_profile          = FlightProfileId::DRONE_HOVER;
uint32_t        FDIRManager::s_consecutive_i2c_errors  = 0;
uint32_t        FDIRManager::s_high_g_event_count      = 0;
uint32_t        FDIRManager::s_anomaly_count           = 0;
uint32_t        FDIRManager::s_jitter_warning_count    = 0;

void FDIRManager::init(FlightProfileId profile) {
    s_active_profile = profile;
    reset_diagnostics();
    ESP_LOGI(TAG, "FDIR Subsystem initialized for profile %u (%s)",
             static_cast<unsigned>(profile),
             get_profile_config(profile)->name);
}

bool FDIRManager::process_sample(const drivers::InertialScaledData& scaled, float dt, volatile uint32_t& health_flags) {
    const auto* p_cfg = get_profile_config(s_active_profile);
    if (p_cfg == nullptr) {
        return false;
    }

    bool sample_valid = true;

    // 1. Gyro dynamic range and saturation check
    float gyro_mag_dps = std::sqrt(scaled.gyro_dps[0] * scaled.gyro_dps[0] +
                                   scaled.gyro_dps[1] * scaled.gyro_dps[1] +
                                   scaled.gyro_dps[2] * scaled.gyro_dps[2]);

    float max_allowed_dps = static_cast<float>(p_cfg->gyro_fs_dps) * 1.05f;
    if (gyro_mag_dps > max_allowed_dps || std::isnan(gyro_mag_dps) || std::isinf(gyro_mag_dps)) {
        s_anomaly_count = s_anomaly_count + 1;
        health_flags |= HEALTH_FLAG_ANOMALY_DETECTED;
        sample_valid = false;
    }

    // 2. Sampling interval / Jitter supervisor
    float nominal_dt = (p_cfg->rate_hz > 0) ? (1.0f / p_cfg->rate_hz) : 0.005f;
    if (dt > 2.0f * nominal_dt || dt < 0.2f * nominal_dt) {
        s_jitter_warning_count = s_jitter_warning_count + 1;
        health_flags |= HEALTH_FLAG_ANOMALY_DETECTED;
    }

    return sample_valid;
}

bool FDIRManager::should_fuse_accelerometer(const drivers::InertialScaledData& scaled, volatile uint32_t& health_flags) {
    const auto* p_cfg = get_profile_config(s_active_profile);
    if (p_cfg == nullptr) {
        return false;
    }

    float accel_mag_g = std::sqrt(scaled.accel_g[0] * scaled.accel_g[0] +
                                  scaled.accel_g[1] * scaled.accel_g[1] +
                                  scaled.accel_g[2] * scaled.accel_g[2]);

    // Check for NaN or Inf
    if (std::isnan(accel_mag_g) || std::isinf(accel_mag_g) || accel_mag_g < 0.1f) {
        health_flags |= HEALTH_FLAG_HIGH_G_REJECTION;
        return false;
    }

    // G-Gating: evaluate absolute deviation from 1.0g reference
    float g_error = std::abs(accel_mag_g - 1.0f);
    if (g_error >= p_cfg->high_g_threshold_g) {
        s_high_g_event_count = s_high_g_event_count + 1;
        health_flags |= HEALTH_FLAG_HIGH_G_REJECTION;
        return false; // Decouple accelerometer from EKF
    }

    // Measurement is within safe gravitational envelope
    health_flags &= ~HEALTH_FLAG_HIGH_G_REJECTION;
    return true;
}

bool FDIRManager::register_i2c_error(volatile uint32_t& health_flags) {
    s_consecutive_i2c_errors = s_consecutive_i2c_errors + 1;
    health_flags &= ~HEALTH_FLAG_IMU_OK;

    if (s_consecutive_i2c_errors >= MAX_CONSECUTIVE_I2C_ERRORS) {
        ESP_LOGE(TAG, "CRITICAL: %u consecutive I2C bus read failures! Declaring HARD_FAULT",
                 static_cast<unsigned>(s_consecutive_i2c_errors));
        health_flags |= HEALTH_FLAG_HARD_FAULT;
        return true; // Require state transition to HARD_FAULT_LOCK
    }

    return false;
}

void FDIRManager::register_i2c_success() {
    s_consecutive_i2c_errors = 0;
}

uint32_t FDIRManager::get_consecutive_i2c_errors() {
    return s_consecutive_i2c_errors;
}

uint32_t FDIRManager::get_high_g_event_count() {
    return s_high_g_event_count;
}

uint32_t FDIRManager::get_anomaly_count() {
    return s_anomaly_count;
}

uint32_t FDIRManager::get_jitter_warning_count() {
    return s_jitter_warning_count;
}

void FDIRManager::reset_diagnostics() {
    s_consecutive_i2c_errors = 0;
    s_high_g_event_count     = 0;
    s_anomaly_count          = 0;
    s_jitter_warning_count   = 0;
}

} // namespace flight
