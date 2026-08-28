#ifndef EKF_HPP
#define EKF_HPP

#include "matrix.hpp"
#include "flight_profiles.hpp"

/**
 * @file ekf.hpp
 * @brief Filtro de Kalman Extendido (EKF 7D) para Estimación de Actitud (AHRS).
 *        Diseño baremetal determinista sin memoria dinámica (MISRA C++ / DO-178C Principles).
 * 
 * Vector de estado (7x1):
 * x = [q0, q1, q2, q3, bx, by, bz]^T
 * - q = [q0, q1, q2, q3]: Cuaternión de orientación espacial (cuaternión unitario)
 * - b = [bx, by, bz]: Sesgos (biases) del giróscopo en rad/s
 */

class ExtendedKalmanFilter {
public:
    Vector7f x{};      // Vector de estado estimado
    Matrix7f P{};      // Matriz de covarianza de error de estimacion
    Matrix7f Q{};      // Matriz de covarianza del ruido de proceso
    Matrix3f R{};      // Matriz de covarianza de medicion efectiva
    Matrix3f R_base{}; // Matriz de covarianza base del acelerometro

    bool useAdaptiveR{true};    // Habilitar escalado dinamico de R (rechazo de aceleraciones parasitas)
    float adaptiveAlpha{15.0f}; // Factor de penalizacion ante aceleraciones no gravitatorias

    ExtendedKalmanFilter();

    // Configura las matrices Q y R segun el perfil de vuelo seleccionado
    void setProfile(flight::FlightProfileId profile);

    // Configura el comportamiento adaptativo de R
    void setAdaptiveR(bool enable, float alpha = 15.0f);

    // Paso de Prediccion: integra mediciones del giroscopio (gx, gy, gz en rad/s) durante un intervalo dt
    void predict(const Vector3f& gyro, float dt);

    // Paso de Actualizacion / Correccion: corrige el estado con las mediciones del acelerometro (ax, ay, az en m/s^2 o g normalizado)
    void update(const Vector3f& accel);

    // Normaliza el cuaternion de orientacion (q0, q1, q2, q3) a norma unitaria
    void normalizeQuaternion();
};

#endif // EKF_HPP
