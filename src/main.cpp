#include "esp_log.h"
#include "nvs_flash.h"
#include "telemetry_task.hpp"
#include "safety_types.hpp"
#include "flight_fsm.hpp"

static const char* TAG = "MAIN";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "  High-Integrity Attitude Estimator V2 (ESP32 Xtensa)  ");
    ESP_LOGI(TAG, "  Philosophy: Zero-Heap / Static Allocation / Lock-Free");
    ESP_LOGI(TAG, "=======================================================");

    // 1. Initialize Non-Volatile Storage (NVS) for calibration / system parameters
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Set initial system state to AWAITING_PROFILE
    flight::g_health_flags = flight::HEALTH_FLAG_TELEMETRY_STREAMING;
    flight::g_system_state = flight::SystemState::AWAITING_PROFILE;

    ESP_LOGI(TAG, "System State: %s (Waiting for Ground Station Profile Handshake)",
             flight::state_to_string(flight::g_system_state));

    // 3. Initialize Core 0 Telemetry Task
    flight::telemetry_task_init();

    // 4. Main task will delete/suspend itself, releasing any remaining main stack resources
    vTaskSuspend(nullptr);
}
