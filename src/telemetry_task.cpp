#include "telemetry_task.hpp"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "TELEMETRY";

namespace flight {

// UART configuration constants
static constexpr uart_port_t UART_PORT_NUM = UART_NUM_0;
static constexpr int UART_BAUD_RATE        = 115200;
static constexpr size_t UART_RX_BUF_SIZE   = 256;
static constexpr size_t TELEMETRY_STACK_SZ = 4096;

// FreeRTOS Static Allocation Buffers (Zero-Heap Policy)
static StackType_t   s_telemetry_stack[TELEMETRY_STACK_SZ];
static StaticTask_t  s_telemetry_tcb;
static uint8_t       s_queue_storage[sizeof(protocol::PayloadTelemetry)];
static StaticQueue_t s_queue_struct;

// Global exported handles and state variables
QueueHandle_t           g_telemetry_queue = nullptr;
volatile SystemState     g_system_state    = SystemState::UNINITIALIZED;
volatile FlightProfileId g_active_profile  = FlightProfileId::UNKNOWN;
volatile uint32_t        g_health_flags    = HEALTH_FLAG_NONE;

// Internal packet buffers
static protocol::Packet<protocol::PayloadHeartbeat> s_hb_packet;
static protocol::Packet<protocol::PayloadAckNack>   s_ack_packet;
static protocol::Packet<protocol::PayloadTelemetry> s_telem_packet;

// UART RX Parser State Machine
enum class RxState {
    WAIT_PREAMBLE_0,
    WAIT_PREAMBLE_1,
    READ_MSG_ID,
    READ_LENGTH,
    READ_PAYLOAD,
    READ_CHECKSUM_LOW,
    READ_CHECKSUM_HIGH
};

static void uart_send_raw(const void* data, size_t length) {
    uart_write_bytes(UART_PORT_NUM, reinterpret_cast<const char*>(data), length);
}

static void send_ack_nack(protocol::MsgId ref_msg, protocol::AckStatus status) {
    protocol::init_packet(s_ack_packet, protocol::MsgId::ACK_NACK);
    s_ack_packet.payload.ref_msg_id = static_cast<uint8_t>(ref_msg);
    s_ack_packet.payload.status     = static_cast<uint8_t>(status);
    protocol::finalize_packet(s_ack_packet);
    uart_send_raw(&s_ack_packet, sizeof(s_ack_packet));
}

static void handle_incoming_command(uint8_t msg_id, uint8_t len, const uint8_t* payload) {
    auto id = static_cast<protocol::MsgId>(msg_id);

    switch (id) {
        case protocol::MsgId::CMD_SET_PROFILE: {
            if (len != sizeof(protocol::PayloadCmdSetProfile)) {
                send_ack_nack(id, protocol::AckStatus::NACK_INVALID_PARAM);
                return;
            }

            const auto* cmd = reinterpret_cast<const protocol::PayloadCmdSetProfile*>(payload);
            const auto requested_profile = static_cast<FlightProfileId>(cmd->profile_id);

            // Rule of In-flight Immutability
            if (g_system_state != SystemState::AWAITING_PROFILE) {
                ESP_LOGW(TAG, "Profile change rejected: In-flight immutability active (Current state: %s)",
                         state_to_string(g_system_state));
                send_ack_nack(id, protocol::AckStatus::NACK_INVALID_STATE);
                return;
            }

            // Validate requested profile ID
            if (requested_profile < FlightProfileId::DRONE_HOVER || requested_profile > FlightProfileId::MISSILE_HIGH_G) {
                ESP_LOGE(TAG, "Invalid profile ID: %u", static_cast<unsigned>(cmd->profile_id));
                send_ack_nack(id, protocol::AckStatus::NACK_INVALID_PARAM);
                return;
            }

            // Lock profile and advance state machine
            g_active_profile = requested_profile;
            ESP_LOGI(TAG, "Profile locked: %u (%s)", 
                     static_cast<unsigned>(g_active_profile),
                     get_profile_config(g_active_profile)->name);

            send_ack_nack(id, protocol::AckStatus::ACK);

            if (FlightFSM::is_transition_valid(g_system_state, SystemState::BIST_AND_CALIBRATION)) {
                g_system_state = SystemState::BIST_AND_CALIBRATION;
            }
            break;
        }

        case protocol::MsgId::CMD_SYSTEM_RESET: {
            ESP_LOGW(TAG, "Operator requested system reset. Rebooting in 50ms...");
            send_ack_nack(id, protocol::AckStatus::ACK);
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_restart();
            break;
        }

        default:
            ESP_LOGW(TAG, "Unknown MsgId received: 0x%02X", msg_id);
            send_ack_nack(id, protocol::AckStatus::NACK_UNKNOWN_CMD);
            break;
    }
}

static void process_uart_rx() {
    static RxState rx_state = RxState::WAIT_PREAMBLE_0;
    static uint8_t rx_msg_id = 0;
    static uint8_t rx_len = 0;
    static uint8_t rx_payload[128];
    static uint8_t rx_payload_idx = 0;
    static uint8_t rx_chk_low = 0;

    uint8_t byte = 0;
    while (uart_read_bytes(UART_PORT_NUM, &byte, 1, 0) > 0) {
        switch (rx_state) {
            case RxState::WAIT_PREAMBLE_0:
                if (byte == protocol::PREAMBLE_0) {
                    rx_state = RxState::WAIT_PREAMBLE_1;
                }
                break;

            case RxState::WAIT_PREAMBLE_1:
                if (byte == protocol::PREAMBLE_1) {
                    rx_state = RxState::READ_MSG_ID;
                } else if (byte != protocol::PREAMBLE_0) {
                    rx_state = RxState::WAIT_PREAMBLE_0;
                }
                break;

            case RxState::READ_MSG_ID:
                rx_msg_id = byte;
                rx_state = RxState::READ_LENGTH;
                break;

            case RxState::READ_LENGTH:
                rx_len = byte;
                rx_payload_idx = 0;
                if (rx_len > sizeof(rx_payload)) {
                    // Invalid packet size: reset parser
                    rx_state = RxState::WAIT_PREAMBLE_0;
                } else if (rx_len == 0) {
                    rx_state = RxState::READ_CHECKSUM_LOW;
                } else {
                    rx_state = RxState::READ_PAYLOAD;
                }
                break;

            case RxState::READ_PAYLOAD:
                rx_payload[rx_payload_idx++] = byte;
                if (rx_payload_idx >= rx_len) {
                    rx_state = RxState::READ_CHECKSUM_LOW;
                }
                break;

            case RxState::READ_CHECKSUM_LOW:
                rx_chk_low = byte;
                rx_state = RxState::READ_CHECKSUM_HIGH;
                break;

            case RxState::READ_CHECKSUM_HIGH: {
                uint16_t received_chk = static_cast<uint16_t>((byte << 8) | rx_chk_low);
                if (protocol::verify_checksum(rx_msg_id, rx_len, rx_payload, received_chk)) {
                    handle_incoming_command(rx_msg_id, rx_len, rx_payload);
                } else {
                    ESP_LOGE(TAG, "Checksum error on MsgId: 0x%02X", rx_msg_id);
                    send_ack_nack(static_cast<protocol::MsgId>(rx_msg_id), protocol::AckStatus::NACK_CHECKSUM_ERROR);
                }
                rx_state = RxState::WAIT_PREAMBLE_0;
                break;
            }
        }
    }
}

void telemetry_task_init() {
    // 1. Configure and install UART driver
    uart_config_t uart_config = {};
    uart_config.baud_rate           = UART_BAUD_RATE;
    uart_config.data_bits           = UART_DATA_8_BITS;
    uart_config.parity              = UART_PARITY_DISABLE;
    uart_config.stop_bits           = UART_STOP_BITS_1;
    uart_config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk          = UART_SCLK_DEFAULT;
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_RX_BUF_SIZE, 0, 0, nullptr, 0));

    // 2. Create static telemetry queue (1 element, overwrite policy)
    g_telemetry_queue = xQueueCreateStatic(
        1,
        sizeof(protocol::PayloadTelemetry),
        s_queue_storage,
        &s_queue_struct
    );
    assert(g_telemetry_queue != nullptr);

    // 3. Create static FreeRTOS task pinned to Core 0 (Priority 3)
    TaskHandle_t task_handle = xTaskCreateStaticPinnedToCore(
        telemetry_task_run,
        "TelemetryTask",
        TELEMETRY_STACK_SZ,
        nullptr,
        3,                    // Priority 3 (Service Domain)
        s_telemetry_stack,
        &s_telemetry_tcb,
        0                     // Pinned strictly to Core 0
    );
    assert(task_handle != nullptr);

    ESP_LOGI(TAG, "Telemetry task initialized on Core 0 (Zero-Heap Static Allocation)");
}

void telemetry_task_run(void* pvParameters) {
    TickType_t last_heartbeat_tick = xTaskGetTickCount();
    const TickType_t heartbeat_period_ticks = pdMS_TO_TICKS(100); // 10 Hz

    while (true) {
        // 1. Process all pending incoming UART bytes
        process_uart_rx();

        // 2. Handle state-specific telemetry output
        switch (g_system_state) {
            case SystemState::AWAITING_PROFILE: {
                // Emit heartbeat at 10 Hz
                TickType_t now = xTaskGetTickCount();
                if ((now - last_heartbeat_tick) >= heartbeat_period_ticks) {
                    last_heartbeat_tick = now;

                    protocol::init_packet(s_hb_packet, protocol::MsgId::HEARTBEAT_AWAIT_PROFILE);
                    s_hb_packet.payload.uptime_ms    = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
                    s_hb_packet.payload.system_state = static_cast<uint8_t>(g_system_state);
                    s_hb_packet.payload.health_flags = g_health_flags;
                    protocol::finalize_packet(s_hb_packet);

                    uart_send_raw(&s_hb_packet, sizeof(s_hb_packet));
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }

            case SystemState::BIST_AND_CALIBRATION: {
                // Handled cooperatively with Core 1 calibration process
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            }

            case SystemState::RUNNING_ESTIMATOR: {
                // Check if new telemetry packet arrived from Core 1
                protocol::PayloadTelemetry incoming_payload;
                if (xQueueReceive(g_telemetry_queue, &incoming_payload, pdMS_TO_TICKS(10)) == pdTRUE) {
                    protocol::init_packet(s_telem_packet, protocol::MsgId::ESTIMATOR_TELEMETRY);
                    s_telem_packet.payload = incoming_payload;
                    protocol::finalize_packet(s_telem_packet);

                    uart_send_raw(&s_telem_packet, sizeof(s_telem_packet));
                }
                break;
            }

            case SystemState::HARD_FAULT_LOCK:
            default:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;
        }
    }
}

} // namespace flight
