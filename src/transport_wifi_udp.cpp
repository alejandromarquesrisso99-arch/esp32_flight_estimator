#include "transport_wifi_udp.hpp"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"
#include <cstring>
#include <fcntl.h>

static const char* TAG = "WIFI_TRANSPORT";

namespace flight {

WifiUdpTransport::WifiUdpTransport(const char* ssid, const char* password, uint16_t rx_port, uint16_t tx_port)
    : m_ssid(ssid),
      m_password(password),
      m_rx_port(rx_port),
      m_tx_port(tx_port),
      m_sock(-1),
      m_has_unicast_target(false),
      m_initialized(false) {
    std::memset(&m_local_addr, 0, sizeof(m_local_addr));
    std::memset(&m_target_addr, 0, sizeof(m_target_addr));
}

WifiUdpTransport::~WifiUdpTransport() {
    if (m_sock >= 0) {
        close(m_sock);
        m_sock = -1;
    }
}

void WifiUdpTransport::wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    auto* self = static_cast<WifiUdpTransport*>(arg);

    if (event_base == WIFI_EVENT) {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            auto* event = static_cast<wifi_event_ap_staconnected_t*>(event_data);
            self->m_connected_clients.fetch_add(1, std::memory_order_relaxed);
            ESP_LOGI(TAG, "Cliente Wi-Fi conectado! MAC: " MACSTR " AID: %d (Total clientes: %u)",
                     MAC2STR(event->mac), event->aid, self->m_connected_clients.load());
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            auto* event = static_cast<wifi_event_ap_stadisconnected_t*>(event_data);
            if (self->m_connected_clients.load() > 0) {
                self->m_connected_clients.fetch_sub(1, std::memory_order_relaxed);
            }
            ESP_LOGW(TAG, "Cliente Wi-Fi desconectado! MAC: " MACSTR " AID: %d (Total clientes: %u)",
                     MAC2STR(event->mac), event->aid, self->m_connected_clients.load());
        }
    }
}

esp_err_t WifiUdpTransport::init() {
    if (m_initialized) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Inicializando subsistema Wi-Fi SoftAP...");

    // 1. Inicializar capa de red TCP/IP lwIP y bucle de eventos
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t event_err = esp_event_loop_create_default();
    if (event_err != ESP_OK && event_err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Fallo al crear bucle de eventos (%s)", esp_err_to_name(event_err));
        return event_err;
    }

    // 2. Crear interfaz de red por defecto para Punto de Acceso (SoftAP)
    esp_netif_t* netif_ap = esp_netif_create_default_wifi_ap();
    if (netif_ap == nullptr) {
        ESP_LOGE(TAG, "Fallo al crear interfaz esp_netif para SoftAP");
        return ESP_FAIL;
    }

    // 3. Inicializar driver Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_wifi_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Fallo al inicializar Wi-Fi driver (%s)", esp_err_to_name(err));
        return err;
    }

    // Registrar manejador de eventos para monitorizar clientes conectados
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &WifiUdpTransport::wifi_event_handler,
        this,
        nullptr
    ));

    // 4. Configurar parametros de la red SoftAP
    wifi_config_t wifi_config = {};
    std::strncpy(reinterpret_cast<char*>(wifi_config.ap.ssid), m_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = std::strlen(m_ssid);
    wifi_config.ap.channel = DEFAULT_CHANNEL;
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.beacon_interval = 100;

    if (m_password != nullptr && std::strlen(m_password) >= 8) {
        std::strncpy(reinterpret_cast<char*>(wifi_config.ap.password), m_password, sizeof(wifi_config.ap.password) - 1);
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        wifi_config.ap.password[0] = '\0';
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "==================================================");
    ESP_LOGI(TAG, "  Wi-Fi SoftAP Iniciado con Exito                 ");
    ESP_LOGI(TAG, "  SSID: %s                                        ", m_ssid);
    ESP_LOGI(TAG, "  IP del ESP32: 192.168.4.1                       ");
    ESP_LOGI(TAG, "  Puerto RX Comandos: %u | Puerto TX Telem: %u   ", m_rx_port, m_tx_port);
    ESP_LOGI(TAG, "==================================================");

    // 5. Crear socket UDP
    m_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (m_sock < 0) {
        ESP_LOGE(TAG, "Fallo al crear socket UDP (errno: %d)", errno);
        return ESP_FAIL;
    }

    // Configurar socket en modo NO BLOQUEANTE para determinismo temporal
    int flags = fcntl(m_sock, F_GETFL, 0);
    if (flags < 0 || fcntl(m_sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        ESP_LOGE(TAG, "Fallo al configurar socket UDP en modo no bloqueante");
        close(m_sock);
        m_sock = -1;
        return ESP_FAIL;
    }

    // Habilitar opcion de Broadcast en el socket
    int broadcast_enable = 1;
    setsockopt(m_sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    // Enlazar socket al puerto local (RX)
    std::memset(&m_local_addr, 0, sizeof(m_local_addr));
    m_local_addr.sin_family      = AF_INET;
    m_local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    m_local_addr.sin_port        = htons(m_rx_port);

    if (bind(m_sock, reinterpret_cast<struct sockaddr*>(&m_local_addr), sizeof(m_local_addr)) < 0) {
        ESP_LOGE(TAG, "Fallo al enlazar socket UDP en puerto %u (errno: %d)", m_rx_port, errno);
        close(m_sock);
        m_sock = -1;
        return ESP_FAIL;
    }

    // Destino inicial de emision: Broadcast de la subred SoftAP (192.168.4.255:5005)
    std::memset(&m_target_addr, 0, sizeof(m_target_addr));
    m_target_addr.sin_family = AF_INET;
    m_target_addr.sin_port   = htons(m_tx_port);
    inet_aton("192.168.4.255", &m_target_addr.sin_addr);

    m_initialized = true;
    return ESP_OK;
}

size_t WifiUdpTransport::send(const void* data, size_t length) {
    if (!m_initialized || m_sock < 0 || data == nullptr || length == 0) {
        return 0;
    }

    int sent = sendto(
        m_sock,
        data,
        length,
        0,
        reinterpret_cast<struct sockaddr*>(&m_target_addr),
        sizeof(m_target_addr)
    );

    return (sent > 0) ? static_cast<size_t>(sent) : 0;
}

size_t WifiUdpTransport::receive(void* buffer, size_t max_len) {
    if (!m_initialized || m_sock < 0 || buffer == nullptr || max_len == 0) {
        return 0;
    }

    sockaddr_in src_addr{};
    socklen_t addr_len = sizeof(src_addr);

    int received = recvfrom(
        m_sock,
        buffer,
        max_len,
        0,
        reinterpret_cast<struct sockaddr*>(&src_addr),
        &addr_len
    );

    if (received > 0) {
        // Al recibir un paquete valido de un cliente (ej. comando de Processing),
        // fijamos la IP del cliente como destino unicast para optimizar el canal RF
        if (!m_has_unicast_target || m_target_addr.sin_addr.s_addr != src_addr.sin_addr.s_addr) {
            m_target_addr.sin_addr = src_addr.sin_addr;
            m_has_unicast_target = true;
            char ip_str[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(src_addr.sin_addr), ip_str, INET_ADDRSTRLEN);
            ESP_LOGI(TAG, "Estacion Terrena vinculada en IP: %s (Puerto TX: %u)", ip_str, m_tx_port);
        }
        return static_cast<size_t>(received);
    }

    return 0;
}

bool WifiUdpTransport::is_connected() const {
    return m_initialized && (m_connected_clients.load() > 0 || m_has_unicast_target);
}

uint16_t WifiUdpTransport::get_connected_stations_count() const {
    return m_connected_clients.load(std::memory_order_relaxed);
}

} // namespace flight
