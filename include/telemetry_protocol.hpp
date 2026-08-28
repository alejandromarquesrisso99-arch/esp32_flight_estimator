#pragma once

#include <cstdint>
#include <cstddef>
#include "safety_types.hpp"
#include "flight_profiles.hpp"
#include "flight_fsm.hpp"

namespace flight {
namespace protocol {

// Preambles for synchronization
constexpr uint8_t PREAMBLE_0 = 0xAA;
constexpr uint8_t PREAMBLE_1 = 0x55;

/**
 * @brief Message / Packet Identifier
 */
enum class MsgId : uint8_t {
    HEARTBEAT_AWAIT_PROFILE = 0x01,  ///< ESP32 -> Processing (10 Hz in AWAITING_PROFILE)
    CMD_SET_PROFILE         = 0x02,  ///< Processing -> ESP32 (Select profile)
    BIST_REPORT             = 0x03,  ///< ESP32 -> Processing (Calibration & Self-Test info)
    ESTIMATOR_TELEMETRY     = 0x04,  ///< ESP32 -> Processing (30-50 Hz in RUNNING_ESTIMATOR)
    ACK_NACK                = 0x05,  ///< ESP32 <-> Processing (Acknowledge packet)
    CMD_SYSTEM_RESET        = 0x06   ///< Processing -> ESP32 (Trigger software reset)
};

/**
 * @brief ACK / NACK Status Codes
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
 * @brief Standard Frame Header
 */
struct FrameHeader {
    uint8_t preamble[2];    ///< {0xAA, 0x55}
    uint8_t msg_id;         ///< MsgId enum
    uint8_t payload_len;    ///< Length of payload following header
};

/**
 * @brief Payload for MSG_HEARTBEAT_AWAIT_PROFILE (0x01)
 */
struct PayloadHeartbeat {
    uint32_t uptime_ms;
    uint8_t  system_state;  ///< SystemState enum
    uint32_t health_flags;  ///< HealthFlags bitmask
};

/**
 * @brief Payload for MSG_CMD_SET_PROFILE (0x02)
 */
struct PayloadCmdSetProfile {
    uint8_t profile_id;     ///< FlightProfileId (1, 2, 3, 4)
};

/**
 * @brief Payload for MSG_BIST_REPORT (0x03)
 */
struct PayloadBistReport {
    uint8_t bist_code;      ///< BistCode enum
    uint8_t progress_pct;   ///< Calibration progress 0-100%
    float   gyro_bias[3];   ///< Calibrated gyro bias in rad/s (X, Y, Z)
    float   accel_bias[3];  ///< Calibrated accel bias in m/s^2 (X, Y, Z)
};

/**
 * @brief Payload for MSG_ESTIMATOR_TELEMETRY (0x04)
 */
struct PayloadTelemetry {
    uint32_t timestamp_us;      ///< Hardware timestamp in microseconds
    float    q[4];              ///< Normalized quaternion [qw, qx, qy, qz]
    float    euler_deg[3];      ///< Euler angles [Roll, Pitch, Yaw] in degrees
    float    gyro_dps[3];       ///< Calibrated gyro rate [wx, wy, wz] in deg/s
    float    accel_g[3];        ///< Calibrated accel [ax, ay, az] in g
    uint32_t wcet_cycles;       ///< Worst-case cycle count on Core 1
    float    wcet_us;           ///< Worst-case execution time in us
    float    loop_freq_hz;      ///< Measured GNC task frequency in Hz
    uint32_t health_flags;      ///< HealthFlags bitmask
    uint8_t  system_state;      ///< SystemState enum
    uint8_t  active_profile_id; ///< Active FlightProfileId
};

/**
 * @brief Payload for MSG_ACK_NACK (0x05)
 */
struct PayloadAckNack {
    uint8_t ref_msg_id;     ///< MsgId being acknowledged
    uint8_t status;         ///< AckStatus enum
};

/**
 * @brief Complete Telemetry Packet Wire Representation
 */
template <typename TPayload>
struct Packet {
    FrameHeader header;
    TPayload    payload;
    uint16_t    checksum;   ///< Fletcher-16 (little-endian)
};

#pragma pack(pop)

/**
 * @brief Standard Fletcher-16 checksum computation
 * Computes over the msg_id, payload_len, and payload bytes.
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
 * @brief Helper to initialize a standard packet with preambles and correct length
 */
template <typename TPayload>
inline void init_packet(Packet<TPayload>& pkt, MsgId id) {
    pkt.header.preamble[0] = PREAMBLE_0;
    pkt.header.preamble[1] = PREAMBLE_1;
    pkt.header.msg_id      = static_cast<uint8_t>(id);
    pkt.header.payload_len = static_cast<uint8_t>(sizeof(TPayload));
}

/**
 * @brief Helper to finalize packet by computing and writing the Fletcher-16 checksum
 */
template <typename TPayload>
inline void finalize_packet(Packet<TPayload>& pkt) {
    // Checksum calculated over msg_id, payload_len and payload data
    const uint8_t* start_ptr = &pkt.header.msg_id;
    const size_t len = 2 + sizeof(TPayload); // msg_id + payload_len + sizeof(payload)
    pkt.checksum = calculate_fletcher16(start_ptr, len);
}

/**
 * @brief Helper to verify checksum of an incoming frame
 */
inline bool verify_checksum(uint8_t msg_id, uint8_t payload_len, const uint8_t* payload, uint16_t received_checksum) {
    uint16_t sum1 = 0;
    uint16_t sum2 = 0;
    
    // Process msg_id
    sum1 = (sum1 + msg_id) % 255;
    sum2 = (sum2 + sum1) % 255;

    // Process payload_len
    sum1 = (sum1 + payload_len) % 255;
    sum2 = (sum2 + sum1) % 255;

    // Process payload
    for (size_t i = 0; i < payload_len; ++i) {
        sum1 = (sum1 + payload[i]) % 255;
        sum2 = (sum2 + sum1) % 255;
    }

    const uint16_t calculated = static_cast<uint16_t>((sum2 << 8) | sum1);
    return (calculated == received_checksum);
}

} // namespace protocol
} // namespace flight
