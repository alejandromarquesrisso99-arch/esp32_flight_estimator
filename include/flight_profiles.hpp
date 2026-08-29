#pragma once

#include <cstdint>

namespace flight {

namespace PhysicsConstants {
    constexpr float GRAVITY_MSS = 9.80665f;   ///< Aceleración estándar de la gravedad en m/s^2
    constexpr float DEG_TO_RAD  = 0.017453292519943295f;
    constexpr float RAD_TO_DEG  = 57.29577951308232f;
}

/**
 * @brief Identificadores formales de los perfiles de vuelo
 */
enum class FlightProfileId : uint8_t {
    UNKNOWN        = 0,
    DRONE_HOVER    = 1,
    DRONE_ACRO     = 2,
    ROCKET_LAUNCH  = 3,
    MISSILE_HIGH_G = 4
};

/**
 * @brief Parámetros estáticos de configuración para un perfil de vuelo
 */
struct FlightProfileConfig {
    FlightProfileId id;
    const char*     name;
    uint16_t        rate_hz;              ///< Frecuencia objetivo del lazo en Hz
    float           sample_time_s;        ///< Periodo de muestreo Ts en segundos
    uint32_t        sample_time_us;       ///< Periodo de muestreo Ts en microsegundos
    uint16_t        gyro_fs_dps;          ///< Fondo de escala del giróscopo (dps: 250, 500, 1000, 2000)
    uint8_t         accel_fs_g;           ///< Fondo de escala del acelerómetro (g: 2, 4, 8, 16)
    uint8_t         dlpf_cfg;             ///< Valor del registro DLPF_CFG del MPU6050 (0-6)
    float           q_process_gyro;       ///< Varianza de ruido de proceso del EKF para giróscopo (rad/s)^2
    float           r_measure_accel;      ///< Varianza de ruido de medición del EKF para acelerómetro
    float           high_g_threshold_g;   ///< Umbral para rechazar actualizaciones de gravedad del acelerómetro
};

/**
 * @brief Tabla de configuración de perfiles de vuelo en tiempo de compilación
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
        0.001f,       // Q giróscopo
        0.050f,       // R acelerómetro
        0.35f         // Umbral High-G (g)
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
        0.005f,       // Q giróscopo
        0.150f,       // R acelerómetro
        1.50f         // Umbral High-G (g)
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
        0.010f,       // Q giróscopo
        0.500f,       // R acelerómetro
        3.00f         // Umbral High-G (g)
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
        0.020f,       // Q giróscopo
        0.800f,       // R acelerómetro
        5.00f         // Umbral High-G (g)
    }
};

/**
 * @brief Obtener la configuración del perfil de vuelo mediante su identificador
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

