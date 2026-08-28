#pragma once

#include <cstdint>
#include "esp_err.h"
#include "flight_profiles.hpp"

namespace flight {
namespace drivers {

class MPU6050Driver {
public:
    static esp_err_t init(FlightProfileId profile_id);
    static esp_err_t read_raw(int16_t accel_raw[3], int16_t gyro_raw[3]);
};

} // namespace drivers
} // namespace flight
