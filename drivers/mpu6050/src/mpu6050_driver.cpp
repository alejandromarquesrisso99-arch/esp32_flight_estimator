#include "mpu6050_driver.hpp"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>

static const char* TAG = "MPU6050";

namespace flight {
namespace drivers {

// Descriptores estáticos de hardware
static i2c_master_bus_handle_t s_bus_handle = nullptr;
static i2c_master_dev_handle_t s_dev_handle = nullptr;
static uint8_t                 s_device_addr = MPU6050_I2C_ADDR_DEFAULT;
static bool                    s_is_initialized = false;

// Factores de conversión de sensibilidad dinámicos según el perfil de vuelo
static float s_gyro_scale_to_dps = 1.0f / 131.0f;     // Por defecto +/- 250 dps
static float s_accel_scale_to_g  = 1.0f / 16384.0f;   // Por defecto +/- 2 g

// Estructura estática de calibración
static CalibrationData s_calibration = {
    .gyro_bias_dps  = {0.0f, 0.0f, 0.0f},
    .gyro_bias_rads = {0.0f, 0.0f, 0.0f},
    .accel_bias_g   = {0.0f, 0.0f, 0.0f},
    .accel_bias_mss = {0.0f, 0.0f, 0.0f},
    .is_calibrated  = false
};

esp_err_t MPU6050Driver::write_reg(uint8_t reg, uint8_t val) {
    if (s_dev_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    const uint8_t write_buf[2] = {reg, val};
    return i2c_master_transmit(s_dev_handle, write_buf, sizeof(write_buf), 50);
}

esp_err_t MPU6050Driver::read_regs(uint8_t reg, uint8_t* buffer, size_t len) {
    if (s_dev_handle == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(s_dev_handle, &reg, 1, buffer, len, 50);
}

esp_err_t MPU6050Driver::verify_who_am_i() {
    uint8_t who_am_i = 0;
    esp_err_t err = read_regs(REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al leer registro WHO_AM_I (Error I2C: %s)", esp_err_to_name(err));
        return err;
    }

    if (who_am_i != WHO_AM_I_VAL_6050 && who_am_i != 0x69) {
        ESP_LOGE(TAG, "Discrepancia en WHO_AM_I! Esperado 0x68/0x69, recibido: 0x%02X", who_am_i);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "MPU6050 detectado con exito (WHO_AM_I: 0x%02X)", who_am_i);
    return ESP_OK;
}

esp_err_t MPU6050Driver::init(FlightProfileId profile_id, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    ESP_LOGI(TAG, "Inicializando MPU6050 en I2C Fast-Mode (400 kHz) [SDA: GPIO%d, SCL: GPIO%d]...",
             sda_pin, scl_pin);

    // 1. Inicializar bus maestro I2C si no ha sido creado previamente
    if (s_bus_handle == nullptr) {
        i2c_master_bus_config_t bus_config = {};
        bus_config.i2c_port = I2C_NUM_0;
        bus_config.sda_io_num = sda_pin;
        bus_config.scl_io_num = scl_pin;
        bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
        bus_config.glitch_ignore_cnt = 7;
        bus_config.flags.enable_internal_pullup = true;

        esp_err_t err = i2c_new_master_bus(&bus_config, &s_bus_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al inicializar bus maestro I2C: %s", esp_err_to_name(err));
            return err;
        }
    }

    // 2. Anadir descriptor del dispositivo MPU6050
    if (s_dev_handle == nullptr) {
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = s_device_addr;
        dev_cfg.scl_speed_hz = 400000; // 400 kHz Fast-Mode

        esp_err_t err = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Fallo al anadir dispositivo I2C: %s", esp_err_to_name(err));
            return err;
        }
    }

    // 3. Reiniciar el dispositivo MPU6050
    ESP_LOGI(TAG, "Emitiendo reinicio de hardware al sensor...");
    write_reg(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(100));

    // 4. Despertar sensor y seleccionar PLL con referencia al giroscopo del eje X
    write_reg(REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    // 5. Verificar identidad del dispositivo
    esp_err_t err = verify_who_am_i();
    if (err != ESP_OK) {
        return err;
    }

    // 6. Obtener configuracion del perfil de vuelo
    const auto* p_cfg = get_profile_config(profile_id);
    if (p_cfg == nullptr) {
        ESP_LOGE(TAG, "ID de perfil invalido: %u", static_cast<unsigned>(profile_id));
        return ESP_ERR_INVALID_ARG;
    }

    // 7. Configurar filtro pasa-bajos digital (registro CONFIG 0x1A)
    write_reg(REG_CONFIG, p_cfg->dlpf_cfg & 0x07);

    // 8. Configurar divisor de tasa de muestreo (registro SMPLRT_DIV 0x19)
    // Tasa = 1000Hz / (1 + SMPLRT_DIV)
    uint8_t smplrt_div = 0;
    if (p_cfg->rate_hz == 200) {
        smplrt_div = 4; // 1000 / (1 + 4) = 200 Hz
    } else if (p_cfg->rate_hz == 500) {
        smplrt_div = 1; // 1000 / (1 + 1) = 500 Hz
    } else if (p_cfg->rate_hz == 1000) {
        smplrt_div = 0; // 1000 / (1 + 0) = 1000 Hz
    }
    write_reg(REG_SMPLRT_DIV, smplrt_div);

    // 9. Configurar fondo de escala del giroscopo (registro GYRO_CONFIG 0x1B)
    uint8_t gyro_fs_sel = 0;
    switch (p_cfg->gyro_fs_dps) {
        case 250:  gyro_fs_sel = 0; s_gyro_scale_to_dps = 1.0f / 131.0f; break;
        case 500:  gyro_fs_sel = 1; s_gyro_scale_to_dps = 1.0f / 65.5f;  break;
        case 1000: gyro_fs_sel = 2; s_gyro_scale_to_dps = 1.0f / 32.8f;  break;
        case 2000: gyro_fs_sel = 3; s_gyro_scale_to_dps = 1.0f / 16.4f;  break;
        default:   gyro_fs_sel = 0; s_gyro_scale_to_dps = 1.0f / 131.0f; break;
    }
    write_reg(REG_GYRO_CONFIG, static_cast<uint8_t>(gyro_fs_sel << 3));

    // 10. Configurar fondo de escala del acelerometro (registro ACCEL_CONFIG 0x1C)
    uint8_t accel_fs_sel = 0;
    switch (p_cfg->accel_fs_g) {
        case 2:  accel_fs_sel = 0; s_accel_scale_to_g = 1.0f / 16384.0f; break;
        case 4:  accel_fs_sel = 1; s_accel_scale_to_g = 1.0f / 8192.0f;  break;
        case 8:  accel_fs_sel = 2; s_accel_scale_to_g = 1.0f / 4096.0f;  break;
        case 16: accel_fs_sel = 3; s_accel_scale_to_g = 1.0f / 2048.0f;  break;
        default: accel_fs_sel = 0; s_accel_scale_to_g = 1.0f / 16384.0f; break;
    }
    write_reg(REG_ACCEL_CONFIG, static_cast<uint8_t>(accel_fs_sel << 3));

    // 11. Configurar pin de interrupcion hardware (registro INT_PIN_CFG 0x37)
    // Bit 5 = LATCH_INT_EN (0 = pulso de 50us), Bit 4 = INT_RD_CLEAR (1 = limpiar con cualquier lectura)
    write_reg(REG_INT_PIN_CFG, 0x10);

    // 12. Habilitar interrupcion por dato listo Data Ready (registro INT_ENABLE 0x38)
    write_reg(REG_INT_ENABLE, 0x01);

    s_is_initialized = true;
    ESP_LOGI(TAG, "MPU6050 configurado para perfil %s (%u Hz, Giro: +/-%u dps, Acel: +/-%u g)",
             p_cfg->name, p_cfg->rate_hz, p_cfg->gyro_fs_dps, p_cfg->accel_fs_g);

    return ESP_OK;
}

esp_err_t MPU6050Driver::read_burst_raw(InertialRawData& raw) {
    uint8_t buf[14];
    esp_err_t err = read_regs(REG_ACCEL_XOUT_H, buf, 14);
    if (err != ESP_OK) {
        return err;
    }

    raw.accel[0] = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    raw.accel[1] = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    raw.accel[2] = static_cast<int16_t>((buf[4] << 8) | buf[5]);
    raw.temp     = static_cast<int16_t>((buf[6] << 8) | buf[7]);
    raw.gyro[0]  = static_cast<int16_t>((buf[8] << 8) | buf[9]);
    raw.gyro[1]  = static_cast<int16_t>((buf[10] << 8) | buf[11]);
    raw.gyro[2]  = static_cast<int16_t>((buf[12] << 8) | buf[13]);

    return ESP_OK;
}

void MPU6050Driver::scale_data(const InertialRawData& raw, InertialScaledData& scaled) {
    // 1. Escalado del acelerometro (restando los sesgos estaticos calibrados)
    scaled.accel_g[0] = (static_cast<float>(raw.accel[0]) * s_accel_scale_to_g) - s_calibration.accel_bias_g[0];
    scaled.accel_g[1] = (static_cast<float>(raw.accel[1]) * s_accel_scale_to_g) - s_calibration.accel_bias_g[1];
    scaled.accel_g[2] = (static_cast<float>(raw.accel[2]) * s_accel_scale_to_g) - s_calibration.accel_bias_g[2];

    scaled.accel_mss[0] = scaled.accel_g[0] * PhysicsConstants::GRAVITY_MSS;
    scaled.accel_mss[1] = scaled.accel_g[1] * PhysicsConstants::GRAVITY_MSS;
    scaled.accel_mss[2] = scaled.accel_g[2] * PhysicsConstants::GRAVITY_MSS;

    // 2. Temperatura interna del die (grados Celsius)
    scaled.temp_c = (static_cast<float>(raw.temp) / 340.0f) + 36.53f;

    // 3. Escalado del giroscopo (restando los sesgos estaticos calibrados)
    float gx_dps = (static_cast<float>(raw.gyro[0]) * s_gyro_scale_to_dps) - s_calibration.gyro_bias_dps[0];
    float gy_dps = (static_cast<float>(raw.gyro[1]) * s_gyro_scale_to_dps) - s_calibration.gyro_bias_dps[1];
    float gz_dps = (static_cast<float>(raw.gyro[2]) * s_gyro_scale_to_dps) - s_calibration.gyro_bias_dps[2];

    scaled.gyro_dps[0] = gx_dps;
    scaled.gyro_dps[1] = gy_dps;
    scaled.gyro_dps[2] = gz_dps;

    scaled.gyro_rads[0] = gx_dps * PhysicsConstants::DEG_TO_RAD;
    scaled.gyro_rads[1] = gy_dps * PhysicsConstants::DEG_TO_RAD;
    scaled.gyro_rads[2] = gz_dps * PhysicsConstants::DEG_TO_RAD;
}

esp_err_t MPU6050Driver::calibrate_biases(size_t sample_count, CalibrationProgressCallback cb) {
    if (!s_is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "Iniciando calibracion estatica del sensor (%u muestras)... Mantener vehiculo en reposo!", 
             static_cast<unsigned>(sample_count));

    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    double sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;
    size_t valid_samples = 0;

    InertialRawData raw{};
    const size_t report_interval = sample_count / 20; // Pasos de 5% de progreso

    for (size_t i = 0; i < sample_count; ++i) {
        esp_err_t err = read_burst_raw(raw);
        if (err == ESP_OK) {
            float gx_dps = static_cast<float>(raw.gyro[0]) * s_gyro_scale_to_dps;
            float gy_dps = static_cast<float>(raw.gyro[1]) * s_gyro_scale_to_dps;
            float gz_dps = static_cast<float>(raw.gyro[2]) * s_gyro_scale_to_dps;

            float ax_g = static_cast<float>(raw.accel[0]) * s_accel_scale_to_g;
            float ay_g = static_cast<float>(raw.accel[1]) * s_accel_scale_to_g;
            float az_g = static_cast<float>(raw.accel[2]) * s_accel_scale_to_g;

            sum_gx += gx_dps;
            sum_gy += gy_dps;
            sum_gz += gz_dps;

            sum_ax += ax_g;
            sum_ay += ay_g;
            sum_az += az_g;

            valid_samples++;
        }

        // Notificar progreso periodicamente
        if (cb != nullptr && (i % report_interval == 0 || i == sample_count - 1)) {
            uint8_t pct = static_cast<uint8_t>((i * 100) / sample_count);
            float current_bias_rads[3] = {
                valid_samples > 0 ? static_cast<float>((sum_gx / valid_samples) * PhysicsConstants::DEG_TO_RAD) : 0.0f,
                valid_samples > 0 ? static_cast<float>((sum_gy / valid_samples) * PhysicsConstants::DEG_TO_RAD) : 0.0f,
                valid_samples > 0 ? static_cast<float>((sum_gz / valid_samples) * PhysicsConstants::DEG_TO_RAD) : 0.0f
            };
            cb(pct, current_bias_rads);
        }

        vTaskDelay(pdMS_TO_TICKS(4));
    }

    if (valid_samples < sample_count / 2) {
        ESP_LOGE(TAG, "Fallo de calibracion: Exceso de errores de lectura I2C (%u/%u)",
                 static_cast<unsigned>(valid_samples), static_cast<unsigned>(sample_count));
        return ESP_FAIL;
    }

    s_calibration.gyro_bias_dps[0] = static_cast<float>(sum_gx / valid_samples);
    s_calibration.gyro_bias_dps[1] = static_cast<float>(sum_gy / valid_samples);
    s_calibration.gyro_bias_dps[2] = static_cast<float>(sum_gz / valid_samples);

    s_calibration.gyro_bias_rads[0] = s_calibration.gyro_bias_dps[0] * PhysicsConstants::DEG_TO_RAD;
    s_calibration.gyro_bias_rads[1] = s_calibration.gyro_bias_dps[1] * PhysicsConstants::DEG_TO_RAD;
    s_calibration.gyro_bias_rads[2] = s_calibration.gyro_bias_dps[2] * PhysicsConstants::DEG_TO_RAD;

    s_calibration.accel_bias_g[0] = static_cast<float>(sum_ax / valid_samples);
    s_calibration.accel_bias_g[1] = static_cast<float>(sum_ay / valid_samples);
    s_calibration.accel_bias_g[2] = static_cast<float>(sum_az / valid_samples) - 1.0f; // Compensar 1g de gravedad sobre Z

    s_calibration.accel_bias_mss[0] = s_calibration.accel_bias_g[0] * PhysicsConstants::GRAVITY_MSS;
    s_calibration.accel_bias_mss[1] = s_calibration.accel_bias_g[1] * PhysicsConstants::GRAVITY_MSS;
    s_calibration.accel_bias_mss[2] = s_calibration.accel_bias_g[2] * PhysicsConstants::GRAVITY_MSS;

    s_calibration.is_calibrated = true;

    ESP_LOGI(TAG, "Calibracion COMPLETADA. Sesgos Giroscopo (rad/s): [%.5f, %.5f, %.5f]",
             s_calibration.gyro_bias_rads[0],
             s_calibration.gyro_bias_rads[1],
             s_calibration.gyro_bias_rads[2]);

    return ESP_OK;
}

const CalibrationData& MPU6050Driver::get_calibration() {
    return s_calibration;
}

bool MPU6050Driver::is_initialized() {
    return s_is_initialized;
}

} // namespace drivers
} // namespace flight
