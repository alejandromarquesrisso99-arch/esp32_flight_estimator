#include "esp_err.h"

namespace flight {

esp_err_t timer_watchdog_init(uint32_t timeout_us) {
    (void)timeout_us;
    return ESP_OK;
}

} // namespace flight
