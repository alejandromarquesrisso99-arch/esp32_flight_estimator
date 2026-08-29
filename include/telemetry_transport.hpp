#pragma once

#include <cstdint>
#include <cstddef>

#ifdef __has_include
    #if __has_include("esp_err.h")
        #include "esp_err.h"
    #else
        typedef int esp_err_t;
        #define ESP_OK 0
        #define ESP_FAIL -1
    #endif
#else
    typedef int esp_err_t;
    #define ESP_OK 0
    #define ESP_FAIL -1
#endif

namespace flight {

/**
 * @brief Identificador del medio de transmisión físico
 */
enum class TransportType : uint8_t {
    UART_SERIAL = 0,
    WIFI_UDP    = 1
};

/**
 * @brief Interfaz abstracta para la capa de transporte de telemetría
 * Cumple con el principio de segregación de interfaz y la política Zero-Heap.
 */
class ITelemetryTransport {
public:
    virtual ~ITelemetryTransport() = default;

    /**
     * @brief Inicializa el hardware, pila de red o puertos asociados
     * @return ESP_OK si la inicialización fue exitosa
     */
    virtual esp_err_t init() = 0;

    /**
     * @brief Envía un bloque de bytes hacia la estación terrena
     * @param data Puntero a los datos
     * @param length Longitud en bytes
     * @return Número de bytes transmitidos
     */
    virtual size_t send(const void* data, size_t length) = 0;

    /**
     * @brief Lee bytes disponibles recibidos desde la estación terrena
     * @param buffer Búfer de destino
     * @param max_len Capacidad máxima del búfer
     * @return Número de bytes leídos (0 si no hay datos disponibles)
     */
    virtual size_t receive(void* buffer, size_t max_len) = 0;

    /**
     * @brief Indica si el canal de comunicación está activo
     */
    virtual bool is_connected() const = 0;

    /**
     * @brief Retorna el tipo de transporte implementado
     */
    virtual TransportType get_type() const = 0;

    /**
     * @brief Retorna una cadena descriptiva del transporte
     */
    virtual const char* get_name() const = 0;
};

} // namespace flight
