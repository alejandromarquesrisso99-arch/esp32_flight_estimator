#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "telemetry_protocol.hpp"

namespace flight {

void gnc_task_run(void* pvParameters) {
    (void)pvParameters;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

} // namespace flight
