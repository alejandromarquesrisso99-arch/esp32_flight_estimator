#include "mpu6050_driver.hpp"

namespace flight {
namespace drivers {

esp_err_t MPU6050Driver::init(FlightProfileId profile_id) {
    (void)profile_id;
    return ESP_OK;
}

esp_err_t MPU6050Driver::read_raw(int16_t accel_raw[3], int16_t gyro_raw[3]) {
    (void)accel_raw;
    (void)gyro_raw;
    return ESP_OK;
}

} // namespace drivers
} // namespace flight
