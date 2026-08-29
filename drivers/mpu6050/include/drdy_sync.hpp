#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace flight {
namespace drivers {

// Pin GPIO hardware por defecto para la linea INT / DRDY del MPU6050
constexpr gpio_num_t DEFAULT_DRDY_GPIO = GPIO_NUM_19;

/**
 * @brief Inicializa el servicio de interrupciones GPIO para Data Ready (DRDY) del MPU6050
 * @param drdy_pin Pin GPIO conectado a la senal INT del sensor
 * @param target_task Descriptor de la tarea FreeRTOS a notificar en cada flanco de subida
 * @return ESP_OK en caso de exito
 */
esp_err_t drdy_sync_init(gpio_num_t drdy_pin, TaskHandle_t target_task);

/**
 * @brief Habilita la interrupcion hardware DRDY
 */
void drdy_sync_enable();

/**
 * @brief Deshabilita la interrupcion hardware DRDY
 */
void drdy_sync_disable();

/**
 * @brief Obtiene el numero total de pulsos hardware DRDY capturados
 */
uint32_t drdy_sync_get_interrupt_count();

/**
 * @brief Restablece el contador de diagnostico de interrupciones
 */
void drdy_sync_reset_counter();

} // namespace drivers
} // namespace flight

