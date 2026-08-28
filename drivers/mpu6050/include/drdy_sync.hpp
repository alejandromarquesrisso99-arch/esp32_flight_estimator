#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace flight {
namespace drivers {

// Default hardware GPIO used for MPU6050 INT / DRDY pin
constexpr gpio_num_t DEFAULT_DRDY_GPIO = GPIO_NUM_19;

/**
 * @brief Initialize GPIO interrupt service for MPU6050 Data Ready (DRDY)
 * @param drdy_pin GPIO pin connected to MPU6050 INT pin
 * @param target_task FreeRTOS task handle to notify on each rising edge
 * @return ESP_OK on success
 */
esp_err_t drdy_sync_init(gpio_num_t drdy_pin, TaskHandle_t target_task);

/**
 * @brief Enable DRDY hardware interrupt
 */
void drdy_sync_enable();

/**
 * @brief Disable DRDY hardware interrupt
 */
void drdy_sync_disable();

/**
 * @brief Get total number of DRDY hardware pulses captured
 */
uint32_t drdy_sync_get_interrupt_count();

/**
 * @brief Reset interrupt diagnostics counter
 */
void drdy_sync_reset_counter();

} // namespace drivers
} // namespace flight
