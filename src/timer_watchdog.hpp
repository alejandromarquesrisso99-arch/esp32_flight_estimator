#pragma once

#include "esp_err.h"
#include "flight_profiles.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace flight {

/**
 * @brief Inicializa el temporizador por hardware GPTimer como watchdog de respaldo para la tarea GNC
 * @param profile ID del perfil de vuelo activo para determinar el periodo de timeout (1.5x T_muestreo)
 * @param target_task Descriptor de la tarea FreeRTOS a notificar en caso de timeout
 * @return ESP_OK en caso de exito
 */
esp_err_t timer_watchdog_init(FlightProfileId profile, TaskHandle_t target_task);

/**
 * @brief Inicia el temporizador hardware watchdog
 */
esp_err_t timer_watchdog_start();

/**
 * @brief Detiene el temporizador hardware watchdog
 */
esp_err_t timer_watchdog_stop();

/**
 * @brief Alimenta / Restablece el contador del watchdog a cero (ejecutado tras cada ciclo valido)
 */
esp_err_t timer_watchdog_feed();

/**
 * @brief Obtiene el numero total de eventos de timeout por respaldo de temporizador activados
 */
uint32_t timer_watchdog_get_timeout_count();

/**
 * @brief Obtiene el periodo configurado de alarma en microsegundos
 */
uint32_t timer_watchdog_get_period_us();

} // namespace flight

