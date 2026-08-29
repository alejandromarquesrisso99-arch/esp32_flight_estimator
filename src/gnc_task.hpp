#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "flight_profiles.hpp"

namespace flight {

/**
 * @brief Inicializa recursos estaticos para la tarea GNC anclada al Nucleo 1 (Prioridad 24)
 */
void gnc_task_init();

/**
 * @brief Bucle de ejecucion de la tarea GNC en el Nucleo 1 (Tiempo Real Duro 200 Hz - 1000 Hz)
 */
void gnc_task_run(void* pvParameters);

/**
 * @brief Obtiene el descriptor TaskHandle de la tarea GNC (utilizado para notificaciones ISR de DRDY/GPTimer)
 */
TaskHandle_t gnc_task_get_handle();

} // namespace flight

