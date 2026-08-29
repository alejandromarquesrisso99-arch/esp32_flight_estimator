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
uint32_t        FDIRManager::s_stuck_samples_count     = 0;
uint32_t        FDIRManager::s_nominal_samples_count   = 0;
uint32_t        FDIRManager::s_stuck_event_count       = 0;
bool            FDIRManager::s_sensor_stuck            = false;
float           FDIRManager::s_last_gyro_dps[3]        = {-9999.0f, -9999.0f, -9999.0f};
float           FDIRManager::s_last_accel_g[3]         = {-9999.0f, -9999.0f, -9999.0f};

void FDIRManager::init(FlightProfileId profile) {
    s_active_profile = profile;
    reset_diagnostics();
    ESP_LOGI(TAG, "Subsistema FDIR inicializado para perfil %u (%s)",
             static_cast<unsigned>(profile),
             get_profile_config(profile)->name);
}

bool FDIRManager::process_sample(const drivers::InertialScaledData& scaled, float dt, std::atomic<uint32_t>& health_flags) {
    const auto* p_cfg = get_profile_config(s_active_profile);
    if (p_cfg == nullptr) {
        return false;
    }

    bool sample_valid = true;
    bool anomaly_in_this_sample = false;

    // 1. Verificacion de rango dinamico y saturacion del giroscopo
    float gyro_mag_dps = std::sqrt(scaled.gyro_dps[0] * scaled.gyro_dps[0] +
                                   scaled.gyro_dps[1] * scaled.gyro_dps[1] +
                                   scaled.gyro_dps[2] * scaled.gyro_dps[2]);

    float max_allowed_dps = static_cast<float>(p_cfg->gyro_fs_dps) * 1.05f;
    if (gyro_mag_dps > max_allowed_dps || std::isnan(gyro_mag_dps) || std::isinf(gyro_mag_dps)) {
        s_anomaly_count = s_anomaly_count + 1;
        health_flags.fetch_or(HEALTH_FLAG_ANOMALY_DETECTED, std::memory_order_relaxed);
        anomaly_in_this_sample = true;
        sample_valid = false;
    }

    // 2. Supervisor de jitter en el periodo de muestreo
    float nominal_dt = (p_cfg->rate_hz > 0) ? (1.0f / p_cfg->rate_hz) : 0.005f;
    if (dt > 2.0f * nominal_dt || dt < 0.2f * nominal_dt) {
        s_jitter_warning_count = s_jitter_warning_count + 1;
        health_flags.fetch_or(HEALTH_FLAG_ANOMALY_DETECTED, std::memory_order_relaxed);
        anomaly_in_this_sample = true;
    }

    // 3. Supervisor de sensor congelado / datos estancados (Stuck-Data Watchdog)
    bool is_identical = (scaled.gyro_dps[0] == s_last_gyro_dps[0] &&
                         scaled.gyro_dps[1] == s_last_gyro_dps[1] &&
                         scaled.gyro_dps[2] == s_last_gyro_dps[2] &&
                         scaled.accel_g[0]  == s_last_accel_g[0] &&
                         scaled.accel_g[1]  == s_last_accel_g[1] &&
                         scaled.accel_g[2]  == s_last_accel_g[2]);

    s_last_gyro_dps[0] = scaled.gyro_dps[0];
    s_last_gyro_dps[1] = scaled.gyro_dps[1];
    s_last_gyro_dps[2] = scaled.gyro_dps[2];
    s_last_accel_g[0]  = scaled.accel_g[0];
    s_last_accel_g[1]  = scaled.accel_g[1];
    s_last_accel_g[2]  = scaled.accel_g[2];

    if (is_identical) {
        s_stuck_samples_count++;
        if (s_stuck_samples_count >= MAX_STUCK_SAMPLES) {
            s_sensor_stuck = true;
            s_stuck_event_count++;
            health_flags.fetch_or(HEALTH_FLAG_ANOMALY_DETECTED, std::memory_order_relaxed);
            anomaly_in_this_sample = true;
            sample_valid = false;
        }
    } else {
        s_stuck_samples_count = 0;
        s_sensor_stuck = false;
    }

    // 4. Filtro de desenganche automatico de anomalias
    if (!anomaly_in_this_sample) {
        s_nominal_samples_count++;
        if (s_nominal_samples_count >= NOMINAL_STABILITY_SAMPLES) {
            health_flags.fetch_and(~static_cast<uint32_t>(HEALTH_FLAG_ANOMALY_DETECTED), std::memory_order_relaxed);
        }
    } else {
        s_nominal_samples_count = 0;
    }

    return sample_valid;
}

bool FDIRManager::should_fuse_accelerometer(const drivers::InertialScaledData& scaled, std::atomic<uint32_t>& health_flags) {
    const auto* p_cfg = get_profile_config(s_active_profile);
    if (p_cfg == nullptr) {
        return false;
    }

    float accel_mag_g = std::sqrt(scaled.accel_g[0] * scaled.accel_g[0] +
                                  scaled.accel_g[1] * scaled.accel_g[1] +
                                  scaled.accel_g[2] * scaled.accel_g[2]);

    // Comprobacion de NaN, Infinito o caida libre
    if (std::isnan(accel_mag_g) || std::isinf(accel_mag_g) || accel_mag_g < 0.1f) {
        health_flags.fetch_or(HEALTH_FLAG_HIGH_G_REJECTION, std::memory_order_relaxed);
        return false;
    }

    // G-Gating: evaluacion de la desviacion absoluta respecto al vector de 1.0g
    float g_error = std::abs(accel_mag_g - 1.0f);
    if (g_error >= p_cfg->high_g_threshold_g) {
        s_high_g_event_count = s_high_g_event_count + 1;
        health_flags.fetch_or(HEALTH_FLAG_HIGH_G_REJECTION, std::memory_order_relaxed);
        return false; // Desacoplar acelerometro del EKF
    }

    // La medicion se encuentra dentro de la envolvente gravitatoria nominal
    health_flags.fetch_and(~static_cast<uint32_t>(HEALTH_FLAG_HIGH_G_REJECTION), std::memory_order_relaxed);
    return true;
}

bool FDIRManager::register_i2c_error(std::atomic<uint32_t>& health_flags) {
    s_consecutive_i2c_errors = s_consecutive_i2c_errors + 1;
    health_flags.fetch_and(~static_cast<uint32_t>(HEALTH_FLAG_IMU_OK), std::memory_order_relaxed);

    if (s_consecutive_i2c_errors >= MAX_CONSECUTIVE_I2C_ERRORS) {
        ESP_LOGE(TAG, "CRITICO: %u fallos consecutivos de lectura en bus I2C! Declarando HARD_FAULT",
                 static_cast<unsigned>(s_consecutive_i2c_errors));
        health_flags.fetch_or(HEALTH_FLAG_HARD_FAULT, std::memory_order_relaxed);
        return true; // Requiere transicion al estado de bloqueo HARD_FAULT_LOCK
    }

    return false;
}

void FDIRManager::register_i2c_success() {
    s_consecutive_i2c_errors = 0;
}

bool FDIRManager::is_sensor_stuck() {
    return s_sensor_stuck;
}

void FDIRManager::notify_recovery_performed() {
    s_stuck_samples_count = 0;
    s_sensor_stuck = false;
    s_nominal_samples_count = 0;
    s_last_gyro_dps[0] = -9999.0f;
    s_last_gyro_dps[1] = -9999.0f;
    s_last_gyro_dps[2] = -9999.0f;
    s_last_accel_g[0]  = -9999.0f;
    s_last_accel_g[1]  = -9999.0f;
    s_last_accel_g[2]  = -9999.0f;
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

uint32_t FDIRManager::get_stuck_data_count() {
    return s_stuck_event_count;
}

void FDIRManager::reset_diagnostics() {
    s_consecutive_i2c_errors = 0;
    s_high_g_event_count     = 0;
    s_anomaly_count          = 0;
    s_jitter_warning_count   = 0;
    s_stuck_samples_count    = 0;
    s_nominal_samples_count  = 0;
    s_stuck_event_count      = 0;
    s_sensor_stuck           = false;
    s_last_gyro_dps[0] = -9999.0f;
    s_last_gyro_dps[1] = -9999.0f;
    s_last_gyro_dps[2] = -9999.0f;
    s_last_accel_g[0]  = -9999.0f;
    s_last_accel_g[1]  = -9999.0f;
    s_last_accel_g[2]  = -9999.0f;
}

} // namespace flight

