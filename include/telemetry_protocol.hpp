#pragma once

#include <cstdint>
#include <cstddef>
#include "safety_types.hpp"
#include "flight_profiles.hpp"
#include "flight_fsm.hpp"

namespace flight {
namespace protocol {

// Preámbulos de sincronización de trama
constexpr uint8_t PREAMBLE_0 = 0xAA;
constexpr uint8_t PREAMBLE_1 = 0x55;

/**
 * @brief Identificador de mensaje / paquete
 */
enum class MsgId : uint8_t {
    HEARTBEAT_AWAIT_PROFILE = 0x01,  ///< ESP32 -> Processing (10 Hz en estado AWAITING_PROFILE)
    CMD_SET_PROFILE         = 0x02,  ///< Processing -> ESP32 (Selección de perfil)
    BIST_REPORT             = 0x03,  ///< ESP32 -> Processing (Informe de calibración y autodiagnóstico)
    ESTIMATOR_TELEMETRY     = 0x04,  ///< ESP32 -> Processing (30-50 Hz en estado RUNNING_ESTIMATOR)
    ACK_NACK                = 0x05,  ///< ESP32 <-> Processing (Acuse de recibo de trama)
    CMD_SYSTEM_RESET        = 0x06   ///< Processing -> ESP32 (Disparo de reinicio por software)
};

/**
 * @brief Códigos de estado para tramas ACK / NACK
 */
enum class AckStatus : uint8_t {
    ACK                 = 0x00,
    NACK_CHECKSUM_ERROR = 0x01,
    NACK_INVALID_STATE  = 0x02,
    NACK_INVALID_PARAM  = 0x03,
    NACK_UNKNOWN_CMD    = 0x04
};

#pragma pack(push, 1)

/**
 * @brief Cabecera estándar de trama
 */
struct FrameHeader {
    uint8_t preamble[2];    ///< {0xAA, 0x55}
    uint8_t msg_id;         ///< Enumeración MsgId
    uint8_t payload_len;    ///< Longitud de la carga útil siguiente
};

/**
 * @brief Carga útil para MSG_HEARTBEAT_AWAIT_PROFILE (0x01)
 */
struct PayloadHeartbeat {
    uint32_t uptime_ms;
    uint8_t  system_state;  ///< Enumeración SystemState
    uint32_t health_flags;  ///< Máscara de bits HealthFlags
};

/**
 * @brief Carga útil para MSG_CMD_SET_PROFILE (0x02)
 */
struct PayloadCmdSetProfile {
    uint8_t profile_id;     ///< FlightProfileId (1, 2, 3, 4)
};

/**
 * @brief Carga útil para MSG_BIST_REPORT (0x03)
 */
struct PayloadBistReport {
    uint8_t bist_code;      ///< Enumeración BistCode
    uint8_t progress_pct;   ///< Porcentaje de progreso de calibración (0-100%)
    float   gyro_bias[3];   ///< Sesgo calibrado de giróscopo en rad/s (X, Y, Z)
    float   accel_bias[3];  ///< Sesgo calibrado de acelerómetro en m/s^2 (X, Y, Z)
};

/**
 * @brief Carga útil para MSG_ESTIMATOR_TELEMETRY (0x04)
 */
struct PayloadTelemetry {
    uint32_t timestamp_us;      ///< Marca de tiempo del hardware timer en microsegundos
    float    q[4];              ///< Cuaternión normalizado [qw, qx, qy, qz]
    float    euler_deg[3];      ///< Ángulos de Euler [Roll, Pitch, Yaw] en grados
    float    gyro_dps[3];       ///< Velocidad angular calibrada [wx, wy, wz] en deg/s
    float    accel_g[3];        ///< Aceleración calibrada [ax, ay, az] en g
    uint32_t wcet_cycles;       ///< Peor recuento de ciclos de CPU en el Núcleo 1
    float    wcet_us;           ///< Tiempo de ejecución en el peor caso en microsegundos
    float    loop_freq_hz;      ///< Frecuencia de ejecución medida de la tarea GNC en Hz
    uint32_t health_flags;      ///< Máscara de bits HealthFlags
    uint8_t  system_state;      ///< Enumeración SystemState
    uint8_t  active_profile_id; ///< Identificador de FlightProfileId activo
};

/**
 * @brief Carga útil para MSG_ACK_NACK (0x05)
 */
struct PayloadAckNack {
    uint8_t ref_msg_id;     ///< MsgId del mensaje al que responde
    uint8_t status;         ///< Enumeración AckStatus
};

/**
 * @brief Estructura completa de paquete de telemetría en el canal físico
 */
template <typename TPayload>
struct Packet {
    FrameHeader header;
    TPayload    payload;
    uint16_t    checksum;   ///< Fletcher-16 (little-endian)
};

#pragma pack(pop)

/**
 * @brief Cálculo de la suma de control estándar Fletcher-16
 * Se calcula sobre msg_id, payload_len y los bytes de la carga útil.
 */
inline uint16_t calculate_fletcher16(const uint8_t* data, size_t length) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    for (size_t i = 0; i < length; ++i) {
        sum1 = (sum1 + data[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }
    return static_cast<uint16_t>((sum2 << 8) | sum1);
}

/**
 * @brief Inicializa un paquete estándar con preámbulos y longitud correspondiente
 */
template <typename TPayload>
inline void init_packet(Packet<TPayload>& pkt, MsgId id) {
    pkt.header.preamble[0] = PREAMBLE_0;
    pkt.header.preamble[1] = PREAMBLE_1;
    pkt.header.msg_id      = static_cast<uint8_t>(id);
    pkt.header.payload_len = static_cast<uint8_t>(sizeof(TPayload));
}

/**
 * @brief Finaliza el paquete calculando y escribiendo la suma de control Fletcher-16
 */
template <typename TPayload>
inline void finalize_packet(Packet<TPayload>& pkt) {
    // La suma de control se calcula sobre msg_id, payload_len y los datos de payload
    const uint8_t* start_ptr = &pkt.header.msg_id;
    const size_t len = 2 + sizeof(TPayload); // msg_id + payload_len + sizeof(payload)
    pkt.checksum = calculate_fletcher16(start_ptr, len);
}

/**
 * @brief Verifica la suma de control de una trama entrante
 */
inline bool verify_checksum(uint8_t msg_id, uint8_t payload_len, const uint8_t* payload, uint16_t received_checksum) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    
    // Procesar msg_id
    sum1 = (sum1 + msg_id) % 255;
    sum2 = (sum2 + sum1) % 255;

    // Procesar payload_len
    sum1 = (sum1 + payload_len) % 255;
    sum2 = (sum2 + sum1) % 255;

    // Procesar carga útil
    for (size_t i = 0; i < payload_len; ++i) {
        sum1 = (sum1 + payload[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    const uint16_t calculated = static_cast<uint16_t>((sum2 << 8) | sum1);
    return (calculated == received_checksum);
}

} // namespace protocol
} // namespace flight

