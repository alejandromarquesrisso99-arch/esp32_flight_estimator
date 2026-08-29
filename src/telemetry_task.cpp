#include "telemetry_task.hpp"
#include "transport_uart.hpp"
#include "transport_wifi_udp.hpp"
#include "gnc_task.hpp"
#include "mpu6050_driver.hpp"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include <cstring>
#include <cmath>

static const char* TAG = "TELEMETRY";

namespace flight {

static constexpr size_t TELEMETRY_STACK_SZ = 8192;

// Instancias estáticas de transporte físico (Dual Simultáneo: UART + Wi-Fi UDP)
static UartTransport    s_uart_transport;
static WifiUdpTransport s_wifi_transport;

// Búferes de asignación estática de FreeRTOS (Política Zero-Heap)
static StackType_t   s_telemetry_stack[TELEMETRY_STACK_SZ];
static StaticTask_t  s_telemetry_tcb;
static uint8_t       s_queue_storage[sizeof(protocol::PayloadTelemetry)];
static StaticQueue_t s_queue_struct;

// Descriptores globales y variables de estado
QueueHandle_t           g_telemetry_queue = nullptr;
volatile SystemState     g_system_state    = SystemState::UNINITIALIZED;
volatile FlightProfileId g_active_profile  = FlightProfileId::UNKNOWN;
std::atomic<uint32_t>    g_health_flags{HEALTH_FLAG_NONE};

// Búferes estáticos internos para tramas
static protocol::Packet<protocol::PayloadHeartbeat>  s_hb_packet;
static protocol::Packet<protocol::PayloadAckNack>    s_ack_packet;
static protocol::Packet<protocol::PayloadBistReport> s_bist_packet;
static protocol::Packet<protocol::PayloadTelemetry>  s_telem_packet;

// Máquina de estados del analizador de tramas entrantes
enum class RxState {
    WAIT_PREAMBLE_0,
    WAIT_PREAMBLE_1,
    READ_MSG_ID,
    READ_LENGTH,
    READ_PAYLOAD,
    READ_CHECKSUM_LOW,
    READ_CHECKSUM_HIGH
};

static void transport_send_raw(const void* data, size_t length) {
    // Emisión simultánea en ambos canales físicos para soporte universal transparente
    s_uart_transport.send(data, length);
    s_wifi_transport.send(data, length);
}

static void send_ack_nack(protocol::MsgId ref_msg, protocol::AckStatus status) {
    protocol::init_packet(s_ack_packet, protocol::MsgId::ACK_NACK);
    s_ack_packet.payload.ref_msg_id = static_cast<uint8_t>(ref_msg);
    s_ack_packet.payload.status     = static_cast<uint8_t>(status);
    protocol::finalize_packet(s_ack_packet);
    transport_send_raw(&s_ack_packet, sizeof(s_ack_packet));
}

void telemetry_send_bist_report(uint8_t bist_code, 
                               uint8_t progress_pct, 
                               const float gyro_bias_rads[3], 
                               const float accel_bias_mss[3]) {
    protocol::init_packet(s_bist_packet, protocol::MsgId::BIST_REPORT);
    s_bist_packet.payload.bist_code     = bist_code;
    s_bist_packet.payload.progress_pct = progress_pct;

    if (gyro_bias_rads != nullptr) {
        s_bist_packet.payload.gyro_bias[0] = gyro_bias_rads[0];
        s_bist_packet.payload.gyro_bias[1] = gyro_bias_rads[1];
        s_bist_packet.payload.gyro_bias[2] = gyro_bias_rads[2];
    } else {
        s_bist_packet.payload.gyro_bias[0] = 0.0f;
        s_bist_packet.payload.gyro_bias[1] = 0.0f;
        s_bist_packet.payload.gyro_bias[2] = 0.0f;
    }

    if (accel_bias_mss != nullptr) {
        s_bist_packet.payload.accel_bias[0] = accel_bias_mss[0];
        s_bist_packet.payload.accel_bias[1] = accel_bias_mss[1];
        s_bist_packet.payload.accel_bias[2] = accel_bias_mss[2];
    } else {
        s_bist_packet.payload.accel_bias[0] = 0.0f;
        s_bist_packet.payload.accel_bias[1] = 0.0f;
        s_bist_packet.payload.accel_bias[2] = 0.0f;
    }

    protocol::finalize_packet(s_bist_packet);
    transport_send_raw(&s_bist_packet, sizeof(s_bist_packet));
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

            // Regla de Inmutabilidad en vuelo
            if (g_system_state != SystemState::AWAITING_PROFILE) {
                ESP_LOGW(TAG, "Cambio de perfil rechazado: Inmutabilidad en vuelo activa (Estado actual: %s)",
                         state_to_string(g_system_state));
                send_ack_nack(id, protocol::AckStatus::NACK_INVALID_STATE);
                return;
            }

            // Validar ID del perfil solicitado
            if (requested_profile < FlightProfileId::DRONE_HOVER || requested_profile > FlightProfileId::MISSILE_HIGH_G) {
                ESP_LOGE(TAG, "ID de perfil invalido: %u", static_cast<unsigned>(cmd->profile_id));
                send_ack_nack(id, protocol::AckStatus::NACK_INVALID_PARAM);
                return;
            }

            // Fijar perfil y avanzar la maquina de estados
            g_active_profile = requested_profile;
            ESP_LOGI(TAG, "Perfil fijado: %u (%s)", 
                     static_cast<unsigned>(g_active_profile),
                     get_profile_config(g_active_profile)->name);

            send_ack_nack(id, protocol::AckStatus::ACK);

            if (FlightFSM::is_transition_valid(g_system_state, SystemState::BIST_AND_CALIBRATION)) {
                g_system_state = SystemState::BIST_AND_CALIBRATION;
            }
            break;
        }

        case protocol::MsgId::CMD_SYSTEM_RESET: {
            ESP_LOGW(TAG, "Reinicio solicitado por el operador. Reiniciando en 50ms...");
            send_ack_nack(id, protocol::AckStatus::ACK);
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_restart();
            break;
        }

        default:
            ESP_LOGW(TAG, "MsgId desconocido recibido: 0x%02X", msg_id);
            send_ack_nack(id, protocol::AckStatus::NACK_UNKNOWN_CMD);
            break;
    }
}

struct FrameParser {
    RxState state = RxState::WAIT_PREAMBLE_0;
    uint8_t msg_id = 0;
    uint8_t len = 0;
    uint8_t payload[128];
    uint8_t payload_idx = 0;
    uint8_t chk_low = 0;

    void process_byte(uint8_t byte) {
        switch (state) {
            case RxState::WAIT_PREAMBLE_0:
                if (byte == protocol::PREAMBLE_0) {
                    state = RxState::WAIT_PREAMBLE_1;
                }
                break;

            case RxState::WAIT_PREAMBLE_1:
                if (byte == protocol::PREAMBLE_1) {
                    state = RxState::READ_MSG_ID;
                } else if (byte != protocol::PREAMBLE_0) {
                    state = RxState::WAIT_PREAMBLE_0;
                }
                break;

            case RxState::READ_MSG_ID:
                msg_id = byte;
                state = RxState::READ_LENGTH;
                break;

            case RxState::READ_LENGTH:
                len = byte;
                payload_idx = 0;
                if (len > sizeof(payload)) {
                    state = RxState::WAIT_PREAMBLE_0;
                } else if (len == 0) {
                    state = RxState::READ_CHECKSUM_LOW;
                } else {
                    state = RxState::READ_PAYLOAD;
                }
                break;

            case RxState::READ_PAYLOAD:
                payload[payload_idx++] = byte;
                if (payload_idx >= len) {
                    state = RxState::READ_CHECKSUM_LOW;
                }
                break;

            case RxState::READ_CHECKSUM_LOW:
                chk_low = byte;
                state = RxState::READ_CHECKSUM_HIGH;
                break;

            case RxState::READ_CHECKSUM_HIGH: {
                uint16_t received_chk = static_cast<uint16_t>((byte << 8) | chk_low);
                if (protocol::verify_checksum(msg_id, len, payload, received_chk)) {
                    handle_incoming_command(msg_id, len, payload);
                } else {
                    ESP_LOGE(TAG, "Error de suma de control en MsgId: 0x%02X", msg_id);
                    send_ack_nack(static_cast<protocol::MsgId>(msg_id), protocol::AckStatus::NACK_CHECKSUM_ERROR);
                }
                state = RxState::WAIT_PREAMBLE_0;
                break;
            }
        }
    }
};

static FrameParser s_uart_parser;
static FrameParser s_wifi_parser;

static void process_incoming_stream() {
    static uint8_t rx_buf[128];

    // 1. Leer comandos desde UART (Cable Serie)
    size_t uart_bytes = s_uart_transport.receive(rx_buf, sizeof(rx_buf));
    for (size_t i = 0; i < uart_bytes; ++i) {
        s_uart_parser.process_byte(rx_buf[i]);
    }

    // 2. Leer comandos desde Wi-Fi UDP (Inalámbrico)
    size_t wifi_bytes = s_wifi_transport.receive(rx_buf, sizeof(rx_buf));
    for (size_t i = 0; i < wifi_bytes; ++i) {
        s_wifi_parser.process_byte(rx_buf[i]);
    }
}

ITelemetryTransport* telemetry_get_active_transport() {
    return &s_wifi_transport;
}

void telemetry_set_active_transport(ITelemetryTransport* transport) {
    // Dual transport siempre activo
}

void telemetry_task_init() {
    // 1. Inicializar ambos transportes físicos (UART y Wi-Fi UDP)
    ESP_LOGI(TAG, "Inicializando transporte UART (115200 baudios)...");
    ESP_ERROR_CHECK(s_uart_transport.init());

    ESP_LOGI(TAG, "Inicializando transporte Wi-Fi SoftAP UDP...");
    ESP_ERROR_CHECK(s_wifi_transport.init());

    // 2. Crear cola estatica de telemetria (1 elemento, politica de sobrescritura)
    g_telemetry_queue = xQueueCreateStatic(
        1,
        sizeof(protocol::PayloadTelemetry),
        s_queue_storage,
        &s_queue_struct
    );
    assert(g_telemetry_queue != nullptr);

    // 3. Crear tarea estatica FreeRTOS anclada al Nucleo 0 (Prioridad 3)
    TaskHandle_t task_handle = xTaskCreateStaticPinnedToCore(
        telemetry_task_run,
        "TelemetryTask",
        TELEMETRY_STACK_SZ,
        nullptr,
        3,                    // Prioridad 3 (Dominio de Servicio)
        s_telemetry_stack,
        &s_telemetry_tcb,
        0                     // Anclada estrictamente al Nucleo 0
    );
    assert(task_handle != nullptr);

    ESP_LOGI(TAG, "Tarea de telemetria inicializada en Nucleo 0 con soporte DUAL simultaneo (UART + Wi-Fi UDP)");
}

void telemetry_task_run(void* pvParameters) {
    TickType_t last_heartbeat_tick = xTaskGetTickCount();
    const TickType_t heartbeat_period_ticks = pdMS_TO_TICKS(100); // 10 Hz

    while (true) {
        // 1. Procesar trafico entrante de ambos transportes
        process_incoming_stream();

        // 2. Manejar emision de telemetria segun estado del sistema
        switch (g_system_state) {
            case SystemState::AWAITING_PROFILE: {
                // Emitir latido de corazon (Heartbeat) a 10 Hz en ambos canales
                TickType_t now = xTaskGetTickCount();
                if ((now - last_heartbeat_tick) >= heartbeat_period_ticks) {
                    last_heartbeat_tick = now;

                    protocol::init_packet(s_hb_packet, protocol::MsgId::HEARTBEAT_AWAIT_PROFILE);
                    s_hb_packet.payload.uptime_ms    = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
                    s_hb_packet.payload.system_state = static_cast<uint8_t>(g_system_state);
                    s_hb_packet.payload.health_flags = g_health_flags.load(std::memory_order_relaxed);
                    protocol::finalize_packet(s_hb_packet);

                    transport_send_raw(&s_hb_packet, sizeof(s_hb_packet));
                }
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }

            case SystemState::BIST_AND_CALIBRATION: {
                ESP_LOGI(TAG, "Ejecutando BIST y Calibracion del MPU6050 para perfil %u...",
                         static_cast<unsigned>(g_active_profile));

                // Paso 1: Inicializar driver hardware MPU6050
                esp_err_t err = drivers::MPU6050Driver::init(g_active_profile);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Fallo al inicializar hardware MPU6050 (%s)! Entrando en HARD_FAULT_LOCK", esp_err_to_name(err));
                    telemetry_send_bist_report(static_cast<uint8_t>(BistCode::IMU_COMM_FAIL), 0, nullptr, nullptr);
                    g_health_flags.fetch_or(HEALTH_FLAG_HARD_FAULT, std::memory_order_relaxed);
                    g_system_state = SystemState::HARD_FAULT_LOCK;
                    break;
                }

                // Paso 2: Calibrar sesgos en reposo estatico
                err = drivers::MPU6050Driver::calibrate_biases(300, [](uint8_t progress_pct, const float gyro_bias_rads[3]) {
                    telemetry_send_bist_report(static_cast<uint8_t>(BistCode::OK), progress_pct, gyro_bias_rads, nullptr);
                });

                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Fallo en calibracion de sesgos del MPU6050! Entrando en HARD_FAULT_LOCK");
                    telemetry_send_bist_report(static_cast<uint8_t>(BistCode::IMU_NOISE_EXCESSIVE), 0, nullptr, nullptr);
                    g_health_flags.fetch_or(HEALTH_FLAG_HARD_FAULT, std::memory_order_relaxed);
                    g_system_state = SystemState::HARD_FAULT_LOCK;
                    break;
                }

                // Paso 3: Emitir informe final y transicionar a RUNNING_ESTIMATOR
                const auto& calib = drivers::MPU6050Driver::get_calibration();
                telemetry_send_bist_report(static_cast<uint8_t>(BistCode::OK), 100, calib.gyro_bias_rads, calib.accel_bias_mss);

                g_health_flags.fetch_or(HEALTH_FLAG_IMU_OK | HEALTH_FLAG_BIST_PASSED | HEALTH_FLAG_EKF_CONVERGED, std::memory_order_relaxed);
                g_system_state = SystemState::RUNNING_ESTIMATOR;
                ESP_LOGI(TAG, "Calibracion del sensor completada! Transicion a RUNNING_ESTIMATOR");

                // Despertar la tarea GNC en Nucleo 1 inmediatamente
                TaskHandle_t gnc_h = gnc_task_get_handle();
                if (gnc_h != nullptr) {
                    xTaskNotifyGive(gnc_h);
                }
                break;
            }

            case SystemState::RUNNING_ESTIMATOR: {
                // Recibir paquete de telemetria EKF producido por la tarea GNC en Nucleo 1
                protocol::PayloadTelemetry incoming_payload;
                if (xQueueReceive(g_telemetry_queue, &incoming_payload, pdMS_TO_TICKS(20)) == pdTRUE) {
                    protocol::init_packet(s_telem_packet, protocol::MsgId::ESTIMATOR_TELEMETRY);
                    s_telem_packet.payload = incoming_payload;
                    protocol::finalize_packet(s_telem_packet);

                    transport_send_raw(&s_telem_packet, sizeof(s_telem_packet));
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
