#pragma once

#include "telemetry_transport.hpp"
#include "driver/uart.h"

namespace flight {

class UartTransport final : public ITelemetryTransport {
public:
    static constexpr uart_port_t DEFAULT_PORT = UART_NUM_0;
    static constexpr int DEFAULT_BAUD_RATE    = 115200;
    static constexpr size_t RX_BUFFER_SIZE    = 256;

    UartTransport(uart_port_t port = DEFAULT_PORT, int baud_rate = DEFAULT_BAUD_RATE);
    ~UartTransport() override = default;

    esp_err_t init() override;
    size_t send(const void* data, size_t length) override;
    size_t receive(void* buffer, size_t max_len) override;
    bool is_connected() const override;
    TransportType get_type() const override { return TransportType::UART_SERIAL; }
    const char* get_name() const override { return "UART_SERIAL"; }

private:
    uart_port_t m_port;
    int m_baud_rate;
    bool m_initialized;
};

} // namespace flight
