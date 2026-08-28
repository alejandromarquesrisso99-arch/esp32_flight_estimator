#pragma once

#include <cstdint>

namespace flight {

/**
 * @brief High-Integrity Health & Status Flags
 */
enum HealthFlagBits : uint32_t {
    HEALTH_FLAG_NONE                  = 0,
    HEALTH_FLAG_IMU_OK                = (1U << 0),  ///< IMU responding and within valid physical bounds
    HEALTH_FLAG_EKF_CONVERGED         = (1U << 1),  ///< EKF covariance and state nominal
    HEALTH_FLAG_TIMER_FALLBACK_ACTIVE = (1U << 2),  ///< DRDY missed, triggered by GPTimer backup
    HEALTH_FLAG_HIGH_G_REJECTION      = (1U << 3),  ///< High dynamic acceleration; gravity vector update rejected
    HEALTH_FLAG_ANOMALY_DETECTED      = (1U << 4),  ///< Anomaly or transient glitch detected by FDIR
    HEALTH_FLAG_BIST_PASSED           = (1U << 5),  ///< Built-in Self-Test completed successfully
    HEALTH_FLAG_HARD_FAULT            = (1U << 6),  ///< Unrecoverable system fault; entered lock state
    HEALTH_FLAG_TELEMETRY_STREAMING   = (1U << 7),  ///< Active telemetry stream to ground station
};

/**
 * @brief Built-In Self-Test (BIST) Result Codes
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
