#pragma once

#include <cstdint>

namespace flight {

namespace PhysicsConstants {
    constexpr float GRAVITY_MSS = 9.80665f;   ///< Standard gravity acceleration in m/s^2
    constexpr float DEG_TO_RAD  = 0.017453292519943295f;
    constexpr float RAD_TO_DEG  = 57.29577951308232f;
}

/**
 * @brief Formal Flight Profile Identifiers
 */
enum class FlightProfileId : uint8_t {
    UNKNOWN        = 0,
    DRONE_HOVER    = 1,
    DRONE_ACRO     = 2,
    ROCKET_LAUNCH  = 3,
    MISSILE_HIGH_G = 4
};

/**
 * @brief Static configuration parameters for a flight profile
 */
struct FlightProfileConfig {
    FlightProfileId id;
    const char*     name;
    uint16_t        rate_hz;              ///< Target loop rate in Hz
    float           sample_time_s;        ///< Sampling period Ts in seconds
    uint32_t        sample_time_us;       ///< Sampling period Ts in microseconds
    uint16_t        gyro_fs_dps;          ///< Gyroscope full scale (dps: 250, 500, 1000, 2000)
    uint8_t         accel_fs_g;           ///< Accelerometer full scale (g: 2, 4, 8, 16)
    uint8_t         dlpf_cfg;             ///< MPU6050 DLPF_CFG register value (0-6)
    float           q_process_gyro;       ///< EKF process noise variance for gyro (rad/s)^2
    float           r_measure_accel;      ///< EKF measurement noise variance for accel
    float           high_g_threshold_g;   ///< Threshold to reject accel gravity updates
};

/**
 * @brief Compile-time flight profile configuration table
 */
inline constexpr FlightProfileConfig PROFILES[] = {
    {
        FlightProfileId::DRONE_HOVER,
        "DRONE_HOVER",
        200,          // 200 Hz
        0.0050f,      // Ts = 5.0 ms
        5000,         // 5000 us
        250,          // +/- 250 dps
        2,            // +/- 2 g
        2,            // DLPF ~98 Hz
        0.001f,       // Q gyro
        0.050f,       // R accel
        1.35f         // High-G threshold (g)
    },
    {
        FlightProfileId::DRONE_ACRO,
        "DRONE_ACRO",
        500,          // 500 Hz
        0.0020f,      // Ts = 2.0 ms
        2000,         // 2000 us
        1000,         // +/- 1000 dps
        8,            // +/- 8 g
        1,            // DLPF ~188 Hz
        0.005f,       // Q gyro
        0.150f,       // R accel
        2.50f         // High-G threshold (g)
    },
    {
        FlightProfileId::ROCKET_LAUNCH,
        "ROCKET_LAUNCH",
        500,          // 500 Hz
        0.0020f,      // Ts = 2.0 ms
        2000,         // 2000 us
        1000,         // +/- 1000 dps
        16,           // +/- 16 g
        1,            // DLPF ~188 Hz
        0.010f,       // Q gyro
        0.500f,       // R accel
        3.00f         // High-G threshold (g)
    },
    {
        FlightProfileId::MISSILE_HIGH_G,
        "MISSILE_HIGH_G",
        1000,         // 1000 Hz
        0.0010f,      // Ts = 1.0 ms
        1000,         // 1000 us
        2000,         // +/- 2000 dps
        16,           // +/- 16 g
        0,            // DLPF ~256 Hz
        0.020f,       // Q gyro
        0.800f,       // R accel
        5.00f         // High-G threshold (g)
    }
};

/**
 * @brief Retrieve flight profile configuration by identifier
 */
constexpr const FlightProfileConfig* get_profile_config(FlightProfileId id) {
    for (const auto& profile : PROFILES) {
        if (profile.id == id) {
            return &profile;
        }
    }
    return nullptr;
}

} // namespace flight
