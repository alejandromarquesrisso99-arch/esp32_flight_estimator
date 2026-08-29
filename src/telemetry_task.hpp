#pragma once

#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "telemetry_protocol.hpp"

namespace flight {

// Shared queue handle for Core 1 -> Core 0 communication
extern QueueHandle_t g_telemetry_queue;

// Global system state & active profile
extern volatile SystemState     g_system_state;
extern volatile FlightProfileId g_active_profile;
extern std::atomic<uint32_t>    g_health_flags;

/**
 * @brief Initialize static resources for telemetry task on Core 0
 */
void telemetry_task_init();

/**
 * @brief Telemetry task loop executed on Core 0
 */
void telemetry_task_run(void* pvParameters);

/**
 * @brief Send BIST and sensor calibration report to Ground Station
 */
void telemetry_send_bist_report(uint8_t bist_code, 
                               uint8_t progress_pct, 
                               const float gyro_bias_rads[3], 
                               const float accel_bias_mss[3]);

} // namespace flight
