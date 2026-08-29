#include "transport_uart.hpp"
#include "esp_log.h"

static const char* TAG = "UART_TRANSPORT";

namespace flight {

UartTransport::UartTransport(uart_port_t port, int baud_rate)
    : m_port(port), m_baud_rate(baud_rate), m_initialized(false) {}

esp_err_t UartTransport::init() {
    if (m_initialized) {
        return ESP_OK;
    }

    uart_config_t uart_config = {};
    uart_config.baud_rate           = m_baud_rate;
    uart_config.data_bits           = UART_DATA_8_BITS;
    uart_config.parity              = UART_PARITY_DISABLE;
    uart_config.stop_bits           = UART_STOP_BITS_1;
    uart_config.flow_ctrl           = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk          = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(m_port, &uart_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al configurar parametros UART (%s)", esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(m_port, RX_BUFFER_SIZE, 0, 0, nullptr, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al instalar driver UART (%s)", esp_err_to_name(err));
        return err;
    }

    m_initialized = true;
    ESP_LOGI(TAG, "Transporte UART inicializado en puerto %d a %d baudios", m_port, m_baud_rate);
    return ESP_OK;
}

size_t UartTransport::send(const void* data, size_t length) {
    if (!m_initialized || data == nullptr || length == 0) {
        return 0;
    }
    int written = uart_write_bytes(m_port, reinterpret_cast<const char*>(data), length);
    return (written > 0) ? static_cast<size_t>(written) : 0;
}

size_t UartTransport::receive(void* buffer, size_t max_len) {
    if (!m_initialized || buffer == nullptr || max_len == 0) {
        return 0;
    }
    int read_bytes = uart_read_bytes(m_port, buffer, max_len, 0);
    return (read_bytes > 0) ? static_cast<size_t>(read_bytes) : 0;
}

bool UartTransport::is_connected() const {
    return m_initialized;
}

} // namespace flight
