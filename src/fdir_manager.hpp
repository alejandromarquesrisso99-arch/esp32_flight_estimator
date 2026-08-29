#pragma once

#include <atomic>
#include "safety_types.hpp"
#include "flight_profiles.hpp"
#include "mpu6050_driver.hpp"

namespace flight {

/**
 * @brief Gestor de Detección, Aislamiento y Recuperación de Fallos (FDIR)
 *        Supervisor de salud de misión crítica que se ejecuta junto al lazo GNC en el Núcleo 1.
 */
class FDIRManager {
public:
    static constexpr uint32_t MAX_CONSECUTIVE_I2C_ERRORS = 5;

    /**
     * @brief Inicializa los umbrales de supervisión FDIR según el perfil de vuelo activo
     */
    static void init(FlightProfileId profile);

    /**
     * @brief Procesa una muestra inercial y evalúa métricas de salud y consistencia física
     * @param scaled Mediciones calibradas del sensor
     * @param dt Intervalo de tiempo transcurrido desde la última muestra
     * @param health_flags Máscara atómica de banderas de salud del sistema
     * @return true si la muestra es segura para procesar, false si está corrupta
     */
    static bool process_sample(const drivers::InertialScaledData& scaled, float dt, std::atomic<uint32_t>& health_flags);

    /**
     * @brief Evalúa si la medición de gravedad del acelerómetro debe fusionarse en el EKF
     * @param scaled Mediciones calibradas del sensor
     * @param health_flags Máscara atómica de banderas de salud del sistema
     * @return true si el vector de gravedad es válido (acelerómetro dentro de la ventana de 1g)
     */
    static bool should_fuse_accelerometer(const drivers::InertialScaledData& scaled, std::atomic<uint32_t>& health_flags);

    /**
     * @brief Registra un fallo de comunicación en el bus I2C
     * @param health_flags Máscara atómica de banderas de salud del sistema
     * @return true si se supera el umbral crítico (requiere transición a HARD_FAULT_LOCK)
     */
    static bool register_i2c_error(std::atomic<uint32_t>& health_flags);

    /**
     * @brief Registra una transacción I2C exitosa (restablece el contador de errores consecutivos)
     */
    static void register_i2c_success();

    /**
     * @brief Métodos de acceso a contadores de diagnóstico
     */
    static uint32_t get_consecutive_i2c_errors();
    static uint32_t get_high_g_event_count();
    static uint32_t get_anomaly_count();
    static uint32_t get_jitter_warning_count();
    static void     reset_diagnostics();

private:
    static FlightProfileId s_active_profile;
    static float           s_high_g_threshold;
    static float           s_max_gyro_rate_dps;
    static uint32_t        s_consecutive_i2c_errors;
    static uint32_t        s_high_g_event_count;
    static uint32_t        s_anomaly_count;
    static uint32_t        s_jitter_warning_count;
};

} // namespace flight
