#pragma once

#include "telemetry_transport.hpp"
#include "esp_event.h"
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace flight {

class WifiUdpTransport final : public ITelemetryTransport {
public:
    static constexpr const char* DEFAULT_SSID     = "ESP32_ATTITUDE_GNC";
    static constexpr const char* DEFAULT_PASS     = ""; // Red abierta para conexion directa
    static constexpr uint8_t     DEFAULT_CHANNEL  = 1;
    static constexpr uint8_t     MAX_STA_CONN     = 4;
    static constexpr uint16_t    LOCAL_RX_PORT    = 5000; ///< Puerto donde el ESP32 escucha comandos
    static constexpr uint16_t    REMOTE_TX_PORT   = 5005; ///< Puerto donde Processing escucha telemetria

    WifiUdpTransport(const char* ssid = DEFAULT_SSID, 
                     const char* password = DEFAULT_PASS, 
                     uint16_t rx_port = LOCAL_RX_PORT, 
                     uint16_t tx_port = REMOTE_TX_PORT);
    ~WifiUdpTransport() override;

    esp_err_t init() override;
    size_t send(const void* data, size_t length) override;
    size_t receive(void* buffer, size_t max_len) override;
    bool is_connected() const override;
    TransportType get_type() const override { return TransportType::WIFI_UDP; }
    const char* get_name() const override { return "WIFI_UDP_SOFTAP"; }

    uint16_t get_connected_stations_count() const;
    const char* get_ssid() const { return m_ssid; }

private:
    const char* m_ssid;
    const char* m_password;
    uint16_t    m_rx_port;
    uint16_t    m_tx_port;

    int         m_sock;
    sockaddr_in m_local_addr;
    sockaddr_in m_target_addr;
    bool        m_has_unicast_target;
    bool        m_initialized;
    
    std::atomic<uint16_t> m_connected_clients{0};

    static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
};

} // namespace flight
