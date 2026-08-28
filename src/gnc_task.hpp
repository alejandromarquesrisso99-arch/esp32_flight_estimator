#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "flight_profiles.hpp"

namespace flight {

/**
 * @brief Initialize static resources for GNC task pinned to Core 1 (Priority 24)
 */
void gnc_task_init();

/**
 * @brief GNC Task execution loop on Core 1 (Hard Real-Time 200 Hz - 1000 Hz)
 */
void gnc_task_run(void* pvParameters);

/**
 * @brief Get the FreeRTOS TaskHandle of the GNC task (used for DRDY/GPTimer ISR notifications)
 */
TaskHandle_t gnc_task_get_handle();

} // namespace flight
