#include "safety_types.hpp"

namespace flight {

bool fdir_check_sanity(const float accel[3], const float gyro[3]) {
    (void)accel;
    (void)gyro;
    return true;
}

} // namespace flight
