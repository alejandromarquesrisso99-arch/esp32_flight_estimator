#include "esp_log.h"
#include "nvs_flash.h"
#include "telemetry_task.hpp"
#include "gnc_task.hpp"
#include "safety_types.hpp"
#include "flight_fsm.hpp"

static const char* TAG = "MAIN";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=======================================================");
    ESP_LOGI(TAG, "  Estimador de Actitud de Alta Integridad V2 (ESP32)  ");
    ESP_LOGI(TAG, "  Filosofia: Zero-Heap / Memoria Estatica / Lock-Free  ");
    ESP_LOGI(TAG, "=======================================================");

    // 1. Inicializar almacenamiento no volatil (NVS)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Establecer estado inicial del sistema en AWAITING_PROFILE
    flight::g_health_flags.store(flight::HEALTH_FLAG_TELEMETRY_STREAMING, std::memory_order_relaxed);
    flight::g_system_state = flight::SystemState::AWAITING_PROFILE;

    ESP_LOGI(TAG, "Estado del Sistema: %s (Esperando seleccion de perfil desde Estacion Terrena)",
             flight::state_to_string(flight::g_system_state));

    // 3. Inicializar tarea de telemetria en Nucleo 0 (Prioridad 3)
    flight::telemetry_task_init();

    // 4. Inicializar tarea GNC de tiempo real duro en Nucleo 1 (Prioridad 24)
    flight::gnc_task_init();

    // 5. Suspender tarea main para liberar recursos de la pila principal de FreeRTOS
    vTaskSuspend(nullptr);
}

