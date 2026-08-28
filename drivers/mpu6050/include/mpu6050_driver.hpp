#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_err.h"
#include "driver/gpio.h"
#include "flight_profiles.hpp"
#include "safety_types.hpp"

namespace flight {
namespace drivers {

// MPU6050 Standard I2C Addresses
constexpr uint8_t MPU6050_I2C_ADDR_DEFAULT = 0x68; ///< AD0 connected to GND
constexpr uint8_t MPU6050_I2C_ADDR_ALT     = 0x69; ///< AD0 connected to VCC

// MPU6050 Register Map
constexpr uint8_t REG_AUX_VDDIO     = 0x01;
constexpr uint8_t REG_SMPLRT_DIV    = 0x19;
constexpr uint8_t REG_CONFIG        = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
constexpr uint8_t REG_FIFO_EN       = 0x23;
constexpr uint8_t REG_INT_PIN_CFG   = 0x37;
constexpr uint8_t REG_INT_ENABLE    = 0x38;
constexpr uint8_t REG_INT_STATUS    = 0x3A;
constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;
constexpr uint8_t REG_TEMP_OUT_H    = 0x41;
constexpr uint8_t REG_GYRO_XOUT_H   = 0x43;
constexpr uint8_t REG_SIGNAL_PATH_RESET = 0x68;
constexpr uint8_t REG_USER_CTRL     = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2    = 0x6C;
constexpr uint8_t REG_WHO_AM_I      = 0x75;

// Expected WHO_AM_I value
constexpr uint8_t WHO_AM_I_VAL_6050 = 0x68;

/**
 * @brief Raw 16-bit sensor readings from atomic 14-byte burst read
 */
struct InertialRawData {
    int16_t accel[3];   ///< Raw accelerometer [X, Y, Z]
    int16_t temp;       ///< Raw internal die temperature
    int16_t gyro[3];    ///< Raw gyroscope [X, Y, Z]
};

/**
 * @brief Physical engineering unit representations
 */
struct InertialScaledData {
    float accel_g[3];       ///< Acceleration in g [X, Y, Z]
    float accel_mss[3];     ///< Acceleration in m/s^2 [X, Y, Z]
    float temp_c;           ///< Temperature in degrees Celsius
    float gyro_dps[3];      ///< Angular velocity in deg/s [X, Y, Z]
    float gyro_rads[3];     ///< Angular velocity in rad/s [X, Y, Z]
};

/**
 * @brief Sensor calibration bias offsets
 */
struct CalibrationData {
    float gyro_bias_dps[3];     ///< Gyro bias in deg/s
    float gyro_bias_rads[3];    ///< Gyro bias in rad/s
    float accel_bias_g[3];      ///< Accel bias in g
    float accel_bias_mss[3];    ///< Accel bias in m/s^2
    bool  is_calibrated;        ///< Calibration validity flag
};

/**
 * @brief Progress callback signature for calibration updates
 */
using CalibrationProgressCallback = void (*)(uint8_t progress_pct, const float gyro_bias_rads[3]);

/**
 * @brief Bare-Metal High-Integrity MPU6050 Driver
 */
class MPU6050Driver {
public:
    /**
     * @brief Initialize I2C bus at 400kHz and configure MPU6050 for selected flight profile
     */
    static esp_err_t init(FlightProfileId profile, 
                          gpio_num_t sda_pin = GPIO_NUM_21, 
                          gpio_num_t scl_pin = GPIO_NUM_22);

    /**
     * @brief Verify device identity via WHO_AM_I register
     */
    static esp_err_t verify_who_am_i();

    /**
     * @brief Execute atomic 14-byte burst read from hardware registers
     */
    static esp_err_t read_burst_raw(InertialRawData& raw);

    /**
     * @brief Convert raw integers to scaled SI and engineering units applying calibration
     */
    static void scale_data(const InertialRawData& raw, InertialScaledData& scaled);

    /**
     * @brief Perform static rest calibration acquiring sample_count samples
     */
    static esp_err_t calibrate_biases(size_t sample_count = 500, 
                                     CalibrationProgressCallback cb = nullptr);

    /**
     * @brief Get active calibration parameters
     */
    static const CalibrationData& get_calibration();

    /**
     * @brief Check if I2C driver is initialized
     */
    static bool is_initialized();

private:
    static esp_err_t write_reg(uint8_t reg, uint8_t val);
    static esp_err_t read_regs(uint8_t reg, uint8_t* buffer, size_t len);
};

} // namespace drivers
} // namespace flight
