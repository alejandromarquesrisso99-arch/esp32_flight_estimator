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
 * @brief High-speed IRAM ISR for MPU6050 Data Ready interrupt
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

    ESP_LOGI(TAG, "Configuring DRDY interrupt on GPIO %d (IRAM ISR)...", s_drdy_pin);

    // 1. Configure GPIO pin
    gpio_config_t io_conf = {};
    io_conf.intr_type    = GPIO_INTR_POSEDGE; // Trigger on rising edge (Data Ready)
    io_conf.pin_bit_mask = (1ULL << s_drdy_pin);
    io_conf.mode         = GPIO_MODE_INPUT;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE; // Pull-down so pin is clean 0 when idle

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d: %s", s_drdy_pin, esp_err_to_name(err));
        return err;
    }

    // 2. Install global ISR service if not already installed
    err = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means ISR service was already installed elsewhere, which is fine
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(err));
        return err;
    }

    // 3. Attach handler to DRDY pin
    err = gpio_isr_handler_add(s_drdy_pin, drdy_gpio_isr_handler, nullptr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler to GPIO %d: %s", s_drdy_pin, esp_err_to_name(err));
        return err;
    }

    s_is_enabled = true;
    ESP_LOGI(TAG, "DRDY interrupt initialized successfully on GPIO %d", s_drdy_pin);
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
