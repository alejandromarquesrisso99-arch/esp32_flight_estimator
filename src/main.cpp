#include "esp_log.h"
#include "nvs_flash.h"
#include "telemetry_task.hpp"
#include "gnc_task.hpp"
#include "safety_types.hpp"
#include "flight_fsm.hpp"

static const char* TAG = "MAIN";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "  High-Integrity Attitude Estimator V2 (ESP32 Xtensa)  ");
    ESP_LOGI(TAG, "  Philosophy: Zero-Heap / Static Allocation / Lock-Free");
    ESP_LOGI(TAG, "=======================================================");

    // 1. Initialize Non-Volatile Storage (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Set initial system state to AWAITING_PROFILE
    flight::g_health_flags.store(flight::HEALTH_FLAG_TELEMETRY_STREAMING, std::memory_order_relaxed);
    flight::g_system_state = flight::SystemState::AWAITING_PROFILE;

    ESP_LOGI(TAG, "System State: %s (Waiting for Ground Station Profile Handshake)",
             flight::state_to_string(flight::g_system_state));

    // 3. Initialize Core 0 Telemetry Task (Priority 3)
    flight::telemetry_task_init();

    // 4. Initialize Core 1 GNC Hard Real-Time Task (Priority 24)
    flight::gnc_task_init();

    // 5. Suspend main task to release FreeRTOS main stack resources
    vTaskSuspend(nullptr);
}
