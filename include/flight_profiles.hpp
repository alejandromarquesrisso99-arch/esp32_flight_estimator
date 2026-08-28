#ifndef FLIGHT_PROFILES_HPP
#define FLIGHT_PROFILES_HPP

#include <cstdint>
#include <cstddef>

/**
 * @file flight_profiles.hpp
 * @brief Definiciones de perfiles de vuelo, constantes fisicas y parametros operativos.
 */

namespace PhysicsConstants {
    constexpr float GRAVITY_MSS = 9.80665f;
    constexpr float DEG_TO_RAD  = 0.017453292519943295f;
    constexpr float RAD_TO_DEG  = 57.29577951308232087f;
}

/**
 * @brief Identificador de perfil de vuelo.
 *        Inmutable una vez seleccionado tras el handshake de arranque.
 */
enum class FlightProfile : uint8_t {
    DRONE_HOVER   = 1, // +-250 deg/s,  +-2g,  200 Hz
    DRONE_ACRO    = 2, // +-1000 deg/s, +-8g,  500 Hz
    ROCKET_LAUNCH = 3, // +-1000 deg/s, +-16g, 500 Hz
    MISSILE_HIGH_G= 4, // +-2000 deg/s, +-16g, 1000 Hz
    CUSTOM        = 5
};

/**
 * @brief Estructura de configuracion estatica de perfil de vuelo
 */
struct ProfileConfig {
    FlightProfile id;
    const char*   name;
    float         sampleRateHz;       // Frecuencia nominal de muestreo GNC (fs)
    float         dt;                 // Periodo de muestreo (Ts = 1 / fs)
    float         gyroFullScaleDps;   // Rango maximo del giroscopo (+- deg/s)
    float         accelFullScaleG;    // Rango maximo del acelerometro (+- g)
    uint8_t       dlpfConfig;         // Configuracion del filtro digital paso bajo (DLPF)
    float         q_gyro;             // Varianza del ruido de proceso del giroscopio
    float         q_bias;             // Varianza del ruido de proceso del sesgo
    float         r_accel;            // Varianza base del acelerometro
    float         adaptiveAlpha;      // Factor de rechazo de aceleracion no gravitatoria
};

namespace FlightProfiles {
    constexpr ProfileConfig PROFILE_DRONE_HOVER = {
        FlightProfile::DRONE_HOVER,
        "DRONE_HOVER",
        200.0f,
        1.0f / 200.0f,
        250.0f,
        2.0f,
        0x03, // DLPF ~42-44 Hz
        0.001f,
        0.000001f,
        0.01f,
        15.0f
    };

    constexpr ProfileConfig PROFILE_DRONE_ACRO = {
        FlightProfile::DRONE_ACRO,
        "DRONE_ACRO",
        500.0f,
        1.0f / 500.0f,
        1000.0f,
        8.0f,
        0x02, // DLPF ~94-98 Hz
        0.005f,
        0.0001f,
        0.50f,
        25.0f
    };

    constexpr ProfileConfig PROFILE_ROCKET_LAUNCH = {
        FlightProfile::ROCKET_LAUNCH,
        "ROCKET_LAUNCH",
        500.0f,
        1.0f / 500.0f,
        1000.0f,
        16.0f,
        0x02, // DLPF ~94-98 Hz
        0.001f,
        0.00001f,
        5.00f,
        50.0f
    };

    constexpr ProfileConfig PROFILE_MISSILE_HIGH_G = {
        FlightProfile::MISSILE_HIGH_G,
        "MISSILE_HIGH_G",
        1000.0f,
        1.0f / 1000.0f,
        2000.0f,
        16.0f,
        0x01, // DLPF ~184-188 Hz
        0.010f,
        0.0005f,
        2.00f,
        40.0f
    };

    inline constexpr const ProfileConfig& getConfig(FlightProfile profile) {
        switch (profile) {
            case FlightProfile::DRONE_ACRO:    return PROFILE_DRONE_ACRO;
            case FlightProfile::ROCKET_LAUNCH: return PROFILE_ROCKET_LAUNCH;
            case FlightProfile::MISSILE_HIGH_G:return PROFILE_MISSILE_HIGH_G;
            case FlightProfile::DRONE_HOVER:
            case FlightProfile::CUSTOM:
            default:                           return PROFILE_DRONE_HOVER;
        }
    }
}

#endif // FLIGHT_PROFILES_HPP
