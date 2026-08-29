#pragma once

#include <atomic>
#include "safety_types.hpp"
#include "flight_profiles.hpp"
#include "mpu6050_driver.hpp"

namespace flight {

/**
 * @brief Fault Detection, Isolation and Recovery (FDIR) Manager
 *        Mission-critical health supervisor running alongside GNC loop on Core 1.
 */
class FDIRManager {
public:
    static constexpr uint32_t MAX_CONSECUTIVE_I2C_ERRORS = 5;

    /**
     * @brief Initialize FDIR monitoring thresholds according to active flight profile
     */
    static void init(FlightProfileId profile);

    /**
     * @brief Process an incoming IMU sensor sample and evaluate health metrics
     * @param scaled Calibrated sensor measurements
     * @param dt Delta time since last sample
     * @param health_flags Output/in-out health flags bitmask
     * @return true if sample is safe to process, false if corrupted
     */
    static bool process_sample(const drivers::InertialScaledData& scaled, float dt, std::atomic<uint32_t>& health_flags);

    /**
     * @brief Evaluate whether the accelerometer gravity measurement should be fused into the EKF
     * @param scaled Calibrated sensor measurements
     * @param health_flags Output health flags bitmask
     * @return true if gravity vector is valid (accelerometer within nominal G-window)
     */
    static bool should_fuse_accelerometer(const drivers::InertialScaledData& scaled, std::atomic<uint32_t>& health_flags);

    /**
     * @brief Register an I2C communication error event
     * @param health_flags Output health flags bitmask
     * @return true if threshold exceeded (requires HARD_FAULT_LOCK transition)
     */
    static bool register_i2c_error(std::atomic<uint32_t>& health_flags);

    /**
     * @brief Register a successful I2C communication event (resets error counter)
     */
    static void register_i2c_success();

    /**
     * @brief Get diagnostics counters
     */
    static uint32_t get_consecutive_i2c_errors();
    static uint32_t get_high_g_event_count();
    static uint32_t get_anomaly_count();
    static uint32_t get_jitter_warning_count();
    static void     reset_diagnostics();

private:
    static FlightProfileId s_active_profile;
    static uint32_t        s_consecutive_i2c_errors;
    static uint32_t        s_high_g_event_count;
    static uint32_t        s_anomaly_count;
    static uint32_t        s_jitter_warning_count;
};

} // namespace flight
