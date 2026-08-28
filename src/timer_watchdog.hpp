#pragma once

#include "esp_err.h"
#include "flight_profiles.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace flight {

/**
 * @brief Initialize the hardware GPTimer watchdog for GNC task fail-safe backup
 * @param profile Active flight profile ID to determine timeout period (1.5x T_sample)
 * @param target_task FreeRTOS task handle to notify on timeout
 * @return ESP_OK on success
 */
esp_err_t timer_watchdog_init(FlightProfileId profile, TaskHandle_t target_task);

/**
 * @brief Start the hardware watchdog timer
 */
esp_err_t timer_watchdog_start();

/**
 * @brief Stop the hardware watchdog timer
 */
esp_err_t timer_watchdog_stop();

/**
 * @brief Feed / Reset the watchdog timer counter to 0 (called on each valid DRDY interrupt)
 */
esp_err_t timer_watchdog_feed();

/**
 * @brief Get total number of timer fallback timeout events triggered
 */
uint32_t timer_watchdog_get_timeout_count();

/**
 * @brief Get the configured alarm timeout period in microseconds
 */
uint32_t timer_watchdog_get_period_us();

} // namespace flight
