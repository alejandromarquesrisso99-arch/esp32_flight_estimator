#include <iostream>
#include <cassert>
#include <cmath>
#include "ekf.hpp"
#include "flight_profiles.hpp"

using namespace flight;

void test_ekf_initialization() {
    std::cout << "[TEST EKF] Initialization & Profile switching..." << std::endl;

    ExtendedKalmanFilter ekf;
    // Default state: [1, 0, 0, 0, 0, 0, 0]^T
    assert(ekf.x(0) == 1.0f);
    assert(ekf.x(1) == 0.0f);
    assert(ekf.x(2) == 0.0f);
    assert(ekf.x(3) == 0.0f);
    assert(ekf.x(4) == 0.0f);
    assert(ekf.x(5) == 0.0f);
    assert(ekf.x(6) == 0.0f);

    // Initial covariance P: 0.1 * I
    for (size_t i = 0; i < 7; ++i) {
        assert(std::fabs(ekf.P(i, i) - 0.1f) < 1e-6f);
    }

    // Switch to MISSILE_HIGH_G and verify process noise Q and adaptive R
    ekf.setProfile(FlightProfileId::MISSILE_HIGH_G);
    assert(ekf.Q(0, 0) == 0.010f);
    assert(ekf.Q(4, 4) == 0.0005f);
    assert(ekf.R_base(0, 0) == 2.00f);
    assert(ekf.adaptiveAlpha == 40.0f);

    // Switch back to DRONE_HOVER
    ekf.setProfile(FlightProfileId::DRONE_HOVER);
    assert(ekf.Q(0, 0) == 0.001f);
    assert(ekf.R_base(0, 0) == 0.01f);

    std::cout << "  -> Initialization & Profiles passed." << std::endl;
}

void test_ekf_kinematics_prediction() {
    std::cout << "[TEST EKF] Quaternion Kinematics Predict Step..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.setProfile(FlightProfileId::DRONE_HOVER);

    // 1. Pure Roll rotation: 90 deg/s around X for 100 steps of dt = 0.01s (Total 1.0s = 90 deg)
    // Roll 90 deg quaternion: [cos(45), sin(45), 0, 0] = [0.7071, 0.7071, 0, 0]
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

    // Verify quaternion is unit norm
    float norm = std::sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    assert(std::fabs(norm - 1.0f) < 1e-5f);

    std::cout << "  -> Kinematics prediction passed." << std::endl;
}

void test_ekf_covariance_propagation_at_rest() {
    std::cout << "[TEST EKF] Covariance growth in rest condition (anti-blindness check)..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.setProfile(FlightProfileId::DRONE_HOVER);

    float p0_diag = ekf.P(0, 0);

    // Predict 100 steps with gyro = 0 (Rest)
    Vector3f gyro_zero{0.0f, 0.0f, 0.0f};
    for (size_t i = 0; i < 100; ++i) {
        ekf.predict(gyro_zero, 0.005f);
    }

    // Covariance must strictly increase due to Q * dt integration
    assert(ekf.P(0, 0) > p0_diag);
    assert(ekf.P(1, 1) > p0_diag);
    assert(ekf.P(4, 4) > p0_diag);

    std::cout << "  -> Rest covariance propagation passed." << std::endl;
}

void test_ekf_gravity_update_convergence() {
    std::cout << "[TEST EKF] Gravity Update & Leveling Convergence (Joseph Form)..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.setProfile(FlightProfileId::DRONE_HOVER);

    // Simulate level vehicle with sensor in standard 1g down (Z = +9.80665 m/s^2)
    Vector3f level_accel{0.0f, 0.0f, PhysicsConstants::GRAVITY_MSS};

    // Perturb initial quaternion with artificial pitch error (q = [cos(15 deg), 0, sin(15 deg), 0])
    float angle_rad = 30.0f * PhysicsConstants::DEG_TO_RAD;
    ekf.x(0) = std::cos(angle_rad * 0.5f);
    ekf.x(1) = 0.0f;
    ekf.x(2) = std::sin(angle_rad * 0.5f);
    ekf.x(3) = 0.0f;

    // Execute multiple update cycles
    for (size_t i = 0; i < 50; ++i) {
        ekf.predict(Vector3f{0.0f, 0.0f, 0.0f}, 0.005f);
        ekf.update(level_accel);
    }

    // Filter should converge back towards level attitude q = [1, 0, 0, 0]
    assert(ekf.x(0) > 0.98f);
    assert(std::fabs(ekf.x(1)) < 0.05f);
    assert(std::fabs(ekf.x(2)) < 0.05f);
    assert(std::fabs(ekf.x(3)) < 0.05f);

    // Check Joseph covariance symmetry: P == P^T
    for (size_t i = 0; i < 7; ++i) {
        for (size_t j = i + 1; j < 7; ++j) {
            assert(std::fabs(ekf.P(i, j) - ekf.P(j, i)) < 1e-6f);
        }
        assert(ekf.P(i, i) >= 1e-6f);
    }

    std::cout << "  -> Gravity update convergence and Joseph form symmetry passed." << std::endl;
}

void test_ekf_anti_nan_robustness() {
    std::cout << "[TEST EKF] Anti-NaN and numerical collapse recovery..." << std::endl;

    ExtendedKalmanFilter ekf;

    // Inject NaN into quaternion state
    ekf.x(0) = std::numeric_limits<float>::quiet_NaN();
    ekf.x(1) = 0.5f;

    ekf.normalizeQuaternion();

    // Must reset to identity quaternion [1, 0, 0, 0]
    assert(ekf.x(0) == 1.0f);
    assert(ekf.x(1) == 0.0f);
    assert(ekf.x(2) == 0.0f);
    assert(ekf.x(3) == 0.0f);

    // Test zero-norm vector
    ekf.x(0) = 0.0f;
    ekf.x(1) = 0.0f;
    ekf.x(2) = 0.0f;
    ekf.x(3) = 0.0f;
    ekf.normalizeQuaternion();
    assert(ekf.x(0) == 1.0f);

    std::cout << "  -> Anti-NaN recovery passed." << std::endl;
}

void test_ekf_domain_guards() {
    std::cout << "[TEST EKF] DO-178C domain guards (zero dt, excessive dt, zero accel)..." << std::endl;

    ExtendedKalmanFilter ekf;
    Vector3f gyro{1.0f, 2.0f, 3.0f};

    // dt <= 0 should be safely ignored
    ekf.predict(gyro, 0.0f);
    assert(ekf.x(0) == 1.0f);
    ekf.predict(gyro, -0.01f);
    assert(ekf.x(0) == 1.0f);

    // dt > 1.0 should be safely ignored
    ekf.predict(gyro, 1.5f);
    assert(ekf.x(0) == 1.0f);

    // Free fall / zero accel (norm < 1e-4) should not divide by zero
    Vector3f zero_accel{0.0f, 0.0f, 0.0f};
    ekf.update(zero_accel);
    assert(ekf.x(0) == 1.0f);

    std::cout << "  -> Domain guards passed." << std::endl;
}

void test_ekf_full_3d_attitude_tracking() {
    std::cout << "[TEST EKF] 1000-Step continuous 3D multi-axis flight simulation..." << std::endl;

    ExtendedKalmanFilter ekf;
    ekf.setProfile(FlightProfileId::DRONE_ACRO);

    const float dt = 0.002f; // 500 Hz

    for (size_t step = 0; step < 1000; ++step) {
        float t = static_cast<float>(step) * dt;

        // Dynamic angular rates
        Vector3f gyro{
            std::sin(t * 2.0f) * 0.5f,
            std::cos(t * 1.5f) * 0.4f,
            0.2f
        };

        ekf.predict(gyro, dt);

        // Approximate gravity in body frame
        float q0 = ekf.x(0), q1 = ekf.x(1), q2 = ekf.x(2), q3 = ekf.x(3);
        Vector3f accel{
            2.0f * (q1 * q3 - q0 * q2) * PhysicsConstants::GRAVITY_MSS,
            2.0f * (q0 * q1 + q2 * q3) * PhysicsConstants::GRAVITY_MSS,
            (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * PhysicsConstants::GRAVITY_MSS
        };

        ekf.update(accel);

        // Verify quaternion unit norm invariant
        float norm = std::sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        assert(std::fabs(norm - 1.0f) < 1e-4f);

        // Verify covariance bounds
        for (size_t i = 0; i < 7; ++i) {
            assert(ekf.P(i, i) >= 1e-6f && ekf.P(i, i) <= 5.0f);
        }
    }

    std::cout << "  -> Continuous 3D flight simulation passed." << std::endl;
}

int main() {
    std::cout << "============================================" << std::endl;
    std::cout << "    Running 7-State EKF Unit Tests (PC)     " << std::endl;
    std::cout << "============================================" << std::endl;

    test_ekf_initialization();
    test_ekf_kinematics_prediction();
    test_ekf_covariance_propagation_at_rest();
    test_ekf_gravity_update_convergence();
    test_ekf_anti_nan_robustness();
    test_ekf_domain_guards();
    test_ekf_full_3d_attitude_tracking();

    std::cout << "\n>>> ALL EKF ENGINE TESTS PASSED SUCCESSFULLY! <<<\n" << std::endl;
    return 0;
}
