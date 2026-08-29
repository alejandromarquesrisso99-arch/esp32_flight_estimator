#pragma once

#include <cstdint>

namespace flight {

/**
 * @brief Banderas de salud y estado del sistema de alta integridad
 */
enum HealthFlagBits : uint32_t {
    HEALTH_FLAG_NONE                  = 0,
    HEALTH_FLAG_IMU_OK                = (1U << 0),  ///< IMU respondiendo dentro de límites físicos válidos
    HEALTH_FLAG_EKF_CONVERGED         = (1U << 1),  ///< Covarianza y estado del EKF nominales
    HEALTH_FLAG_TIMER_FALLBACK_ACTIVE = (1U << 2),  ///< Pérdida de pulso DRDY, activado respaldo por GPTimer
    HEALTH_FLAG_HIGH_G_REJECTION      = (1U << 3),  ///< Aceleración dinámica elevada; actualización por gravedad rechazada
    HEALTH_FLAG_ANOMALY_DETECTED      = (1U << 4),  ///< Anomalía o transitorio espurio detectado por FDIR
    HEALTH_FLAG_BIST_PASSED           = (1U << 5),  ///< Autodiagnóstico (BIST) completado con éxito
    HEALTH_FLAG_HARD_FAULT            = (1U << 6),  ///< Fallo irrecuperable del sistema; entrada en estado de bloqueo
    HEALTH_FLAG_TELEMETRY_STREAMING   = (1U << 7),  ///< Transmisión activa de telemetría hacia estación terrena
};

/**
 * @brief Códigos de resultado del autodiagnóstico en arranque (BIST)
 */
enum class BistCode : uint8_t {
    OK                  = 0x00,
    ALU_TEST_FAIL       = 0x01,
    IMU_COMM_FAIL       = 0x02,
    IMU_NOISE_EXCESSIVE = 0x03,
    TIMER_FAIL          = 0x04,
    CALIBRATION_TIMEOUT = 0x05,
    UNKNOWN_ERROR       = 0xFF
};

} // namespace flight

