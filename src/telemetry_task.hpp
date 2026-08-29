#pragma once

#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "telemetry_protocol.hpp"

namespace flight {

// Descriptor de cola compartida para comunicación del Núcleo 1 al Núcleo 0
extern QueueHandle_t g_telemetry_queue;

// Variables globales de estado del sistema y perfil activo
extern volatile SystemState     g_system_state;
extern volatile FlightProfileId g_active_profile;
extern std::atomic<uint32_t>    g_health_flags;

/**
 * @brief Inicializa recursos estáticos para la tarea de telemetría en el Núcleo 0
 */
void telemetry_task_init();

/**
 * @brief Bucle de ejecución de la tarea de telemetría en el Núcleo 0
 */
void telemetry_task_run(void* pvParameters);

/**
 * @brief Envía informe de autodiagnóstico (BIST) y calibración hacia la estación terrena
 */
void telemetry_send_bist_report(uint8_t bist_code, 
                               uint8_t progress_pct, 
                               const float gyro_bias_rads[3], 
                               const float accel_bias_mss[3]);

} // namespace flight

