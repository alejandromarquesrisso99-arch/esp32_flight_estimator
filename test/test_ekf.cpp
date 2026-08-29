#include <iostream>
#include <cassert>
#include <cmath>
#include "ekf.hpp"
#include "flight_profiles.hpp"

using namespace flight;

void test_ekf_initialization() {
    std::cout << "[PRUEBA EKF] Inicializacion y conmutacion de perfiles..." << std::endl;

    ExtendedKalmanFilter ekf;
    // Estado inicial por defecto: [1, 0, 0, 0, 0, 0, 0]^T
    assert(ekf.x(0) == 1.0f);
    assert(ekf.x(1) == 0.0f);
    assert(ekf.x(2) == 0.0f);
    assert(ekf.x(3) == 0.0f);
    assert(ekf.x(4) == 0.0f);
    assert(ekf.x(5) == 0.0f);
    assert(ekf.x(6) == 0.0f);

    // Covarianza inicial P: 0.1 * I
    for (size_t i = 0; i < 7; ++i) {
        assert(std::fabs(ekf.P(i, i) - 0.1f) < 1e-6f);
    }

    // Conmutar a MISSILE_HIGH_G y verificar ruido de proceso Q y R adaptativa
    ekf.setProfile(FlightProfileId::MISSILE_HIGH_G);
    assert(ekf.Q(0, 0) == 0.010f);
    assert(ekf.Q(4, 4) == 0.0005f);
    assert(ekf.R_base(0, 0) == 2.00f);
    assert(ekf.adaptiveAlpha == 40.0f);

    // Conmutar nuevamente a DRONE_HOVER
    ekf.setProfile(FlightProfileId::DRONE_HOVER);
    assert(ekf.Q(0, 0) == 0.001f);
    assert(ekf.R_base(0, 0) == 0.01f);

    std::cout << "  -> Inicializacion y perfiles superados con exito." << std::endl;
}

void test_ekf_kinematics_prediction() {
    std::cout << "[PRUEBA EKF] Paso de prediccion cinematica de cuaterniones..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.setProfile(FlightProfileId::DRONE_HOVER);

    // 1. Rotacion pura de alabeo (Roll): 90 deg/s en X durante 200 pasos de dt = 0.005s (Total 1.0s = 90 deg)
    // Cuaternion esperado a 90 deg Roll: [cos(45), sin(45), 0, 0] = [0.7071, 0.7071, 0, 0]
    float roll_rate_rads = 90.0f * PhysicsConstants::DEG_TO_RAD; // pi/2 rad/s
    Vector3f gyro_x{roll_rate_rads, 0.0f, 0.0f};

    const float dt = 0.005f;
    const size_t steps = 200; // 200 * 0.005 = 1.0s

    for (size_t i = 0; i < steps; ++i) {
        ekf.predict(gyro_x, dt);
    }

    float q0 = ekf.x(0), q1 = ekf.x(1), q2 = ekf.x(2), q3 = ekf.x(3);
    float expected_q0 = std::cos(45.0f * PhysicsConstants::DEG_TO_RAD);
    float expected_q1 = std::sin(45.0f * PhysicsConstants::DEG_TO_RAD);

    assert(std::fabs(q0 - expected_q0) < 0.01f);
    assert(std::fabs(q1 - expected_q1) < 0.01f);
    assert(std::fabs(q2) < 0.01f);
    assert(std::fabs(q3) < 0.01f);

    // Verificar que el cuaternion mantenga norma unitaria
    float norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    assert(std::fabs(norm - 1.0f) < 1e-5f);

    std::cout << "  -> Prediccion cinematica superada con exito." << std::endl;
}

void test_ekf_covariance_propagation_at_rest() {
    std::cout << "[PRUEBA EKF] Crecimiento de covarianza en reposo (prevencion de ceguera)..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.setProfile(FlightProfileId::DRONE_HOVER);

    float p0_diag = ekf.P(0, 0);

    // Predecir 100 pasos con giroscopo = 0 (Reposo estatico)
    Vector3f gyro_zero{0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < 100; ++i) {
        ekf.predict(gyro_zero, 0.005f);
    }

    // La covarianza debe crecer estrictamente debido a la integracion de Q * dt
    assert(ekf.P(0, 0) > p0_diag);
    assert(ekf.P(1, 1) > p0_diag);
    assert(ekf.P(4, 4) > p0_diag);

    std::cout << "  -> Propagacion de covarianza en reposo superada con exito." << std::endl;
}

void test_ekf_gravity_update_convergence() {
    std::cout << "[PRUEBA EKF] Actualizacion por gravedad y convergencia de nivelacion (Forma Joseph)..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.setProfile(FlightProfileId::DRONE_HOVER);

    // Simular vehiculo nivelado con lectura nominal de 1g en Z (+9.80665 m/s^2)
    Vector3f level_accel{0.0f, 0.0f, PhysicsConstants::GRAVITY_MSS};

    // Perturbar cuaternion inicial con error artificial de cabeceo (Pitch = 30 deg)
    float angle_rad = 30.0f * PhysicsConstants::DEG_TO_RAD;
    ekf.x(0) = std::cos(angle_rad * 0.5f);
    ekf.x(1) = 0.0f;
    ekf.x(2) = std::sin(angle_rad * 0.5f);
    ekf.x(3) = 0.0f;

    // Ejecutar multiples ciclos de actualizacion
    for (size_t i = 0; i < 50; ++i) {
        ekf.predict(Vector3f{0.0f, 0.0f, 0.0f}, 0.005f);
        ekf.update(level_accel);
    }

    // El filtro debe converger a la actitud nivelada q = [1, 0, 0, 0]
    assert(ekf.x(0) > 0.98f);
    assert(std::fabs(ekf.x(1)) < 0.05f);
    assert(std::fabs(ekf.x(2)) < 0.05f);
    assert(std::fabs(ekf.x(3)) < 0.05f);

    // Verificar simetria y condicion definida positiva de la covarianza de Joseph: P == P^T
    for (size_t i = 0; i < 7; ++i) {
        for (size_t j = i + 1; j < 7; ++j) {
            assert(std::fabs(ekf.P(i, j) - ekf.P(j, i)) < 1e-6f);
        }
        assert(ekf.P(i, i) >= 1e-6f);
    }

    std::cout << "  -> Convergencia de actualizacion y simetria de Joseph superadas con exito." << std::endl;
}

void test_ekf_anti_nan_robustness() {
    std::cout << "[PRUEBA EKF] Recuperacion y robustez Anti-NaN ante colapso numerico..." << std::endl;

    ExtendedKalmanFilter ekf;

    // Inyectar NaN en el cuaternion
    ekf.x(0) = std::numeric_limits<float>::quiet_NaN();
    ekf.x(1) = 0.5f;

    ekf.normalizeQuaternion();

    // Debe restablecerse al cuaternion identidad [1, 0, 0, 0]
    assert(ekf.x(0) == 1.0f);
    assert(ekf.x(1) == 0.0f);
    assert(ekf.x(2) == 0.0f);
    assert(ekf.x(3) == 0.0f);

    // Probar vector de norma cero
    ekf.x(0) = 0.0f;
    ekf.x(1) = 0.0f;
    ekf.x(2) = 0.0f;
    ekf.x(3) = 0.0f;
    ekf.normalizeQuaternion();
    assert(ekf.x(0) == 1.0f);

    std::cout << "  -> Recuperacion Anti-NaN superada con exito." << std::endl;
}

void test_ekf_domain_guards() {
    std::cout << "[PRUEBA EKF] Guardas de dominio DO-178C (dt nulo, dt excesivo, aceleracion nula)..." << std::endl;

    ExtendedKalmanFilter ekf;
    Vector3f gyro{1.0f, 2.0f, 3.0f};

    // dt <= 0 debe ser ignorado de forma segura
    ekf.predict(gyro, 0.0f);
    assert(ekf.x(0) == 1.0f);
    ekf.predict(gyro, -0.01f);
    assert(ekf.x(0) == 1.0f);

    // dt > 1.0 debe ser ignorado de forma segura
    ekf.predict(gyro, 1.5f);
    assert(ekf.x(0) == 1.0f);

    // Caida libre o aceleracion nula (norma < 1e-4) no debe provocar division por cero
    Vector3f zero_accel{0.0f, 0.0f, 0.0f};
    ekf.update(zero_accel);
    assert(ekf.x(0) == 1.0f);

    std::cout << "  -> Guardas de dominio superadas con exito." << std::endl;
}

void test_ekf_full_3d_attitude_tracking() {
    std::cout << "[PRUEBA EKF] Simulacion de vuelo continuo 3D multieje de 1000 pasos..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.setProfile(FlightProfileId::DRONE_ACRO);

    const float dt = 0.002f; // 500 Hz

    for (size_t step = 0; step < 1000; ++step) {
        float t = static_cast<float>(step) * dt;

        // Velocidades angulares dinamicas
        Vector3f gyro{
            std::sin(t * 2.0f) * 0.5f,
            std::cos(t * 1.5f) * 0.4f,
            0.2f
        };

        ekf.predict(gyro, dt);

        // Aceleracion gravitatoria en marco del cuerpo
        float q0 = ekf.x(0), q1 = ekf.x(1), q2 = ekf.x(2), q3 = ekf.x(3);
        Vector3f accel{
            2.0f * (q1 * q3 - q0 * q2) * PhysicsConstants::GRAVITY_MSS,
            2.0f * (q0 * q1 + q2 * q3) * PhysicsConstants::GRAVITY_MSS,
            (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * PhysicsConstants::GRAVITY_MSS
        };

        ekf.update(accel);

        // Verificar invariante de norma unitaria del cuaternion
        float norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        assert(std::fabs(norm - 1.0f) < 1e-4f);

        // Verificar limites de covarianza
        for (size_t i = 0; i < 7; ++i) {
            assert(ekf.P(i, i) >= 1e-6f && ekf.P(i, i) <= 5.0f);
        }
    }

    std::cout << "  -> Simulacion de vuelo 3D superada con exito." << std::endl;
}

int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "    Pruebas Unitarias del EKF 7D (PC)       " << std::endl;
    std::cout << "============================================" << std::endl;

    test_ekf_initialization();
    test_ekf_kinematics_prediction();
    test_ekf_covariance_propagation_at_rest();
    test_ekf_gravity_update_convergence();
    test_ekf_anti_nan_robustness();
    test_ekf_domain_guards();
    test_ekf_full_3d_attitude_tracking();

    std::cout << "\n>>> TODAS LAS PRUEBAS DEL EKF PASARON CON EXITO! <<<\n" << std::endl;
    return 0;
}

