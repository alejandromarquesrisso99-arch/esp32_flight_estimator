#include "drdy_sync.hpp"
#include "esp_log.h"
#include "esp_attr.h"

static const char* TAG = "DRDY_SYNC";

namespace flight {
namespace drivers {

static TaskHandle_t      s_target_task     = nullptr;
static gpio_num_t        s_drdy_pin        = DEFAULT_DRDY_GPIO;
static volatile uint32_t s_interrupt_count = 0;
static bool              s_is_enabled      = false;

/**
 * @brief Rutina de servicio de interrupción (ISR) de alta velocidad residente en IRAM
 */
static void IRAM_ATTR drdy_gpio_isr_handler(void* arg) {
    s_interrupt_count = s_interrupt_count + 1;
    if (s_target_task != nullptr) {
        BaseType_t high_task_wakeup = pdFALSE;
        vTaskNotifyGiveFromISR(s_target_task, &high_task_wakeup);
        if (high_task_wakeup == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t drdy_sync_init(gpio_num_t drdy_pin, TaskHandle_t target_task) {
    s_drdy_pin = drdy_pin;
    s_target_task = target_task;

    ESP_LOGI(TAG, "Configurando interrupcion DRDY en GPIO %d (ISR en IRAM)...", s_drdy_pin);

    // 1. Configurar pin GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_POSEDGE; // Disparo por flanco de subida (Data Ready)
    io_conf.pin_bit_mask = (1ULL << s_drdy_pin);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // Pull-down para nivel 0 limpio en reposo

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al configurar GPIO %d: %s", s_drdy_pin, esp_err_to_name(err));
        return err;
    }

    // 2. Instalar servicio global de ISRs si no ha sido instalado previamente
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE indica que el servicio ya fue instalado en otro modulo, lo cual es correcto
        ESP_LOGE(TAG, "Fallo al instalar servicio GPIO ISR: %s", esp_err_to_name(err));
        return err;
    }

    // 3. Vincular manejador ISR al pin DRDY
    err = gpio_isr_handler_add(s_drdy_pin, drdy_gpio_isr_handler, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al asociar manejador ISR al GPIO %d: %s", s_drdy_pin, esp_err_to_name(err));
        return err;
    }

    s_is_enabled = true;
    ESP_LOGI(TAG, "Interrupcion DRDY inicializada con exito en GPIO %d", s_drdy_pin);
    return ESP_OK;
}

void drdy_sync_enable() {
    if (!s_is_enabled) {
        gpio_intr_enable(s_drdy_pin);
        s_is_enabled = true;
    }
}

void drdy_sync_disable() {
    if (s_is_enabled) {
        gpio_intr_disable(s_drdy_pin);
        s_is_enabled = false;
    }
}

uint32_t drdy_sync_get_interrupt_count() {
    return s_interrupt_count;
}

void drdy_sync_reset_counter() {
    s_interrupt_count = 0;
}

} // namespace drivers
} // namespace flight

