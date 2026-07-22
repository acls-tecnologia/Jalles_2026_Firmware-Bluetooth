#include "wifi.h"
#include "config.h"

#include <string.h>
#include <errno.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "esp_wifi_types.h" // para wifi_event_sta_disconnected_t
#include "esp_http_client.h"

static const char *TAG = "WiFi";

static EventGroupHandle_t s_wifi_event_group = NULL;
static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_FAIL_BIT = BIT1;

static bool s_netif_inited = false;
static bool s_wifi_inited = false;
static bool s_wifi_started = false;
static bool s_handlers_registered = false;

static esp_netif_t *s_sta_netif = NULL;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (!s_wifi_event_group)
        return;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // Reconexão automática
        wifi_event_sta_disconnected_t *e = (wifi_event_sta_disconnected_t *)data; // pode ser NULL às vezes
        ESP_LOGW(TAG, "STA_DISCONNECTED reason=%d", e ? e->reason : -1);

        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // só pra garantir
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);        // sinaliza falha de conexão

        return;
    }
    else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
    {
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);    // só pra garantir
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT); // sinaliza conexão OK
        ESP_LOGI(TAG, "GOT_IP");
        return;
    }
}

static esp_err_t ensure_system_inited(void)
{
    // NVS (boa prática antes de Wi-Fi)
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    if (r != ESP_OK)
        return r;

    // netif + loop
    if (!s_netif_inited)
    {
        r = esp_netif_init();
        if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
            return r;

        r = esp_event_loop_create_default();
        if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
            return r;

        s_netif_inited = true;
    }

    if (!s_wifi_event_group)
    {
        s_wifi_event_group = xEventGroupCreate();
        if (!s_wifi_event_group)
            return ESP_ERR_NO_MEM;
    }

    if (!s_sta_netif)
    {
        s_sta_netif = esp_netif_create_default_wifi_sta();
        if (!s_sta_netif)
            return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t wifi_start_driver(void)
{
    esp_err_t r = ensure_system_inited();
    if (r != ESP_OK)
        return r;

    if (!s_wifi_inited)
    {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        r = esp_wifi_init(&cfg);
        if (r != ESP_OK)
            return r;

        r = esp_wifi_set_storage(WIFI_STORAGE_RAM);
        if (r != ESP_OK)
            return r;

        r = esp_wifi_set_mode(WIFI_MODE_STA);
        if (r != ESP_OK)
            return r;

        // r = esp_wifi_set_ps(WIFI_PS_NONE);
        r = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);

        if (r != ESP_OK)
            return r;

        s_wifi_inited = true;
    }

    if (!s_handlers_registered)
    {
        r = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
        if (r != ESP_OK)
            return r;

        r = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);
        if (r != ESP_OK)
            return r;

        s_handlers_registered = true;
    }

    // limpa bits antigos ANTES de start/conectar
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    if (s_wifi_started)
    {
        ESP_LOGI(TAG, "Driver Wi-Fi já STARTED");
        return ESP_OK;
    }
    else
    {
        r = esp_wifi_start();
        if (r != ESP_OK)
            return r;

        s_wifi_started = true;
        ESP_LOGI(TAG, "Driver Wi-Fi STARTED");
    }

    return ESP_OK;
}

esp_err_t wifi_stop_driver(void)
{
    if (!s_wifi_started)
        return ESP_OK;

    ESP_LOGW(TAG, "Parando Wi-Fi (stop) ...");
    esp_wifi_disconnect();
    esp_err_t r = esp_wifi_stop();

    if (r == ESP_OK)
    {
        s_wifi_started = false;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    }
    return r;
}

// use só se você realmente quiser “zerar tudo”
esp_err_t wifi_deinit_driver(void)
{
    ESP_LOGW(TAG, "Deinit completo do Wi-Fi...");

    if (s_wifi_started)
    {
        esp_wifi_disconnect();
        esp_wifi_stop();
        s_wifi_started = false;
    }

    if (s_handlers_registered)
    {
        esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler);
        esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler);
        s_handlers_registered = false;
    }

    if (s_wifi_inited)
    {
        esp_err_t r = esp_wifi_deinit();
        if (r != ESP_OK)
            return r;
        s_wifi_inited = false;
    }

    if (s_sta_netif)
    {
        esp_netif_destroy(s_sta_netif);
        s_sta_netif = NULL;
    }

    if (s_wifi_event_group)
    {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    return ESP_OK;
}

bool wifi_is_active(void)
{
    return s_wifi_started;
}

bool wifi_sta_connected(void)
{
    if (!s_wifi_started)
        return false;
    wifi_ap_record_t ap;
    return (esp_wifi_sta_get_ap_info(&ap) == ESP_OK);
}

esp_err_t wifi_connect_credentials(const char *ssid, const char *pass, int timeout_ms)
{
    if (!ssid || !pass)
        return ESP_ERR_INVALID_ARG;

    esp_err_t r = wifi_start_driver();
    if (r != ESP_OK)
        return r;

    // limpa bits antes de esperar
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    // garante que não está "connecting" ao mudar config
    esp_wifi_disconnect();         // ignora erro
    vTaskDelay(pdMS_TO_TICKS(50)); // curto, só pra dar tempo do state mudar

    wifi_config_t cfg = {0};                                           // zera toda a struct
    strlcpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));         // copia SSID
    strlcpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password)); // copia senha

    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK; // mínimo WPA2
    cfg.sta.pmf_cfg.capable = true;                  // PMF capaz
    cfg.sta.pmf_cfg.required = false;                // PMF não obrigatório

    // aplica config
    r = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (r != ESP_OK)
        return r;

    // conecta
    r = esp_wifi_connect();

    if (r == ESP_ERR_WIFI_STATE)
    {
        ESP_LOGW(TAG, "wifi_connect_credentials: conexão já em andamento; aguardando resultado");
    }
    else if (r != ESP_OK)
        return r;

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(timeout_ms));

    if (bits & WIFI_CONNECTED_BIT)
        return ESP_OK;
    if (bits & WIFI_FAIL_BIT)
        return ESP_FAIL;
    return ESP_ERR_TIMEOUT;
}

bool wifi_has_ip(void)
{
    if (!s_sta_netif)
        return false;

    esp_netif_ip_info_t ip;
    if (esp_netif_get_ip_info(s_sta_netif, &ip) != ESP_OK)
        return false;

    return (ip.ip.addr != 0);
}

esp_err_t wifi_wait_ip(TickType_t timeout)
{
    // Se já tem IP, não espera bit nenhum
    if (wifi_has_ip())
        return ESP_OK;

    if (!s_wifi_event_group)
        return ESP_ERR_INVALID_STATE;

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        timeout);

    if (bits & WIFI_CONNECTED_BIT)
        return ESP_OK;
    if (bits & WIFI_FAIL_BIT)
        return ESP_FAIL;
    return ESP_ERR_TIMEOUT;
}

// Check simples por TCP (atenção: redes podem bloquear porta 53 TCP, então escolha bem ip/porta)
static bool parse_host_port_from_url(const char *url, char *host_out, size_t host_len, int *port_out)
{
    if (!url || !host_out || !port_out)
        return false;

    const char *p = url;
    int default_port = 80;

    if (strncmp(p, "http://", 7) == 0)
    {
        p += 7;
        default_port = 80;
    }
    else if (strncmp(p, "https://", 8) == 0)
    {
        p += 8;
        default_port = 443;
    }

    const char *host_start = p;

    while (*p && *p != ':' && *p != '/')
        p++;

    size_t host_sz = (size_t)(p - host_start);
    if (host_sz == 0 || host_sz >= host_len)
        return false;

    memcpy(host_out, host_start, host_sz);
    host_out[host_sz] = '\0';

    int port = default_port;

    if (*p == ':')
    {
        p++;
        port = 0;

        while (*p && *p >= '0' && *p <= '9')
        {
            port = port * 10 + (*p - '0');
            p++;
        }

        if (port <= 0 || port > 65535)
            return false;
    }

    *port_out = port;
    return true;
}
static bool tcp_connect_timeout(const char *host, int port, int timeout_ms)
{
    struct addrinfo hints = {0}, *res = NULL;
    char port_str[8];
    snprintf(port_str, sizeof(port_str), "%d", port);

    hints.ai_family = AF_INET; // IPv4 (pode mudar pra AF_UNSPEC se quiser)
    hints.ai_socktype = SOCK_STREAM;

    if (getaddrinfo(host, port_str, &hints, &res) != 0 || !res)
    {
        return false;
    }

    int sock = socket(res->ai_family, res->ai_socktype, 0);
    if (sock < 0)
    {
        freeaddrinfo(res);
        return false;
    }

    // non-blocking
    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    int ret = connect(sock, res->ai_addr, res->ai_addrlen);
    if (ret < 0 && errno != EINPROGRESS)
    {
        close(sock);
        freeaddrinfo(res);
        return false;
    }

    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    ret = select(sock + 1, NULL, &wfds, NULL, &tv);
    if (ret <= 0)
    { // timeout ou erro
        close(sock);
        freeaddrinfo(res);
        return false;
    }

    int so_error = 0;
    socklen_t len = sizeof(so_error);
    getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);

    close(sock);
    freeaddrinfo(res);

    return (so_error == 0);
}

// ✅ MESMA ASSINATURA, AGORA TCP REAL
bool wifi_check_internet_tcp(const char *url, int timeout_ms)
{
    if (!url || url[0] == '\0')
        return false;

    char host[128];
    int port = 0;

    if (!parse_host_port_from_url(url, host, sizeof(host), &port))
    {
        return false;
    }

    return tcp_connect_timeout(host, port, timeout_ms);
}

bool wifi_check_internet_https(const char *url, int timeout_ms)
{
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = timeout_ms,
        .cert_pem = rootCaCerticate,
        .keep_alive_enable = false,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client)
        return false;

    esp_http_client_set_method(client, HTTP_METHOD_GET);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    esp_http_client_cleanup(client);

    if (err != ESP_OK)
        return false;

    return (status > 0);
}
