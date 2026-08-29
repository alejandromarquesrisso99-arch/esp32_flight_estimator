#pragma once

#include <cstdint>
#include <cstddef>

#if __has_include("esp_err.h")
#include "esp_err.h"
#include "driver/gpio.h"
#else
typedef int esp_err_t;
typedef int gpio_num_t;
constexpr gpio_num_t GPIO_NUM_21 = 21;
constexpr gpio_num_t GPIO_NUM_22 = 22;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_INVALID_ARG -2
#define ESP_ERR_INVALID_STATE -3
#endif

#include "flight_profiles.hpp"
#include "safety_types.hpp"

namespace flight {
namespace drivers {

// Direcciones estándar I2C del sensor MPU6050
constexpr uint8_t MPU6050_I2C_ADDR_DEFAULT = 0x68; ///< Pin AD0 conectado a GND
constexpr uint8_t MPU6050_I2C_ADDR_ALT     = 0x69; ///< Pin AD0 conectado a VCC

// Mapa de registros del MPU6050
constexpr uint8_t REG_AUX_VDDIO     = 0x01;
constexpr uint8_t REG_SMPLRT_DIV    = 0x19;
constexpr uint8_t REG_CONFIG        = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
constexpr uint8_t REG_FIFO_EN       = 0x23;
constexpr uint8_t REG_INT_PIN_CFG   = 0x37;
constexpr uint8_t REG_INT_ENABLE    = 0x38;
constexpr uint8_t REG_INT_STATUS    = 0x3A;
constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;
constexpr uint8_t REG_TEMP_OUT_H    = 0x41;
constexpr uint8_t REG_GYRO_XOUT_H   = 0x43;
constexpr uint8_t REG_SIGNAL_PATH_RESET = 0x68;
constexpr uint8_t REG_USER_CTRL     = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2    = 0x6C;
constexpr uint8_t REG_WHO_AM_I      = 0x75;

// Valor esperado del registro WHO_AM_I
constexpr uint8_t WHO_AM_I_VAL_6050 = 0x68;

/**
 * @brief Lecturas crudas de 16 bits obtenidas en la lectura atómica en ráfaga de 14 bytes
 */
struct InertialRawData {
    int16_t accel[3];   ///< Aceleración cruda [X, Y, Z]
    int16_t temp;       ///< Temperatura cruda del chip
    int16_t gyro[3];    ///< Velocidad angular cruda [X, Y, Z]
};

/**
 * @brief Datos inerciales escalados en unidades físicas del Sistema Internacional
 */
struct InertialScaledData {
    float accel_g[3];       ///< Aceleración en unidades g [X, Y, Z]
    float accel_mss[3];     ///< Aceleración en m/s^2 [X, Y, Z]
    float temp_c;           ///< Temperatura en grados Celsius
    float gyro_dps[3];      ///< Velocidad angular en deg/s [X, Y, Z]
    float gyro_rads[3];     ///< Velocidad angular en rad/s [X, Y, Z]
};

/**
 * @brief Estructura de almacenamiento de sesgos de calibración
 */
struct CalibrationData {
    float gyro_bias_dps[3];     ///< Sesgo del giróscopo en deg/s
    float gyro_bias_rads[3];    ///< Sesgo del giróscopo en rad/s
    float accel_bias_g[3];      ///< Sesgo del acelerómetro en g
    float accel_bias_mss[3];    ///< Sesgo del acelerómetro en m/s^2
    bool  is_calibrated;        ///< Bandera de validez de la calibración
};

/**
 * @brief Firma de la función de devolución de llamada (callback) para el progreso de calibración
 */
using CalibrationProgressCallback = void (*)(uint8_t progress_pct, const float gyro_bias_rads[3]);

/**
 * @brief Controlador bare-metal de alta integridad para el sensor MPU6050
 */
class MPU6050Driver {
public:
    /**
     * @brief Inicializa el bus I2C a 400 kHz y configura el MPU6050 para el perfil de vuelo seleccionado
     */
    static esp_err_t init(FlightProfileId profile, 
                          gpio_num_t sda_pin = GPIO_NUM_21, 
                          gpio_num_t scl_pin = GPIO_NUM_22);

    /**
     * @brief Verifica la identidad del dispositivo mediante el registro WHO_AM_I
     */
    static esp_err_t verify_who_am_i();

    /**
     * @brief Ejecuta la lectura atómica en ráfaga de 14 bytes de los registros del sensor
     */
    static esp_err_t read_burst_raw(InertialRawData& raw);

    /**
     * @brief Convierte valores enteros crudos a unidades físicas aplicando la calibración
     */
    static void scale_data(const InertialRawData& raw, InertialScaledData& scaled);

    /**
     * @brief Realiza la calibración estática en reposo promediando sample_count muestras
     */
    static esp_err_t calibrate_biases(size_t sample_count = 500, 
                                     CalibrationProgressCallback cb = nullptr);

    /**
     * @brief Obtiene los parámetros activos de calibración
     */
    static const CalibrationData& get_calibration();

    /**
     * @brief Comprueba si el driver I2C se encuentra inicializado
     */
    static bool is_initialized();

    /**
     * @brief Reinicia la ruta de señal analógica y digital del sensor sin perder la configuración de registros
     */
    static esp_err_t reset_signal_path();

private:
    static esp_err_t write_reg(uint8_t reg, uint8_t val);
    static esp_err_t read_regs(uint8_t reg, uint8_t* buffer, size_t len);
};

} // namespace drivers
} // namespace flight

