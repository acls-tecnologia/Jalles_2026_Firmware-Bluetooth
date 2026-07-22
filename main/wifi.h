#pragma once

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include <stdbool.h>

esp_err_t wifi_start_driver(void);
esp_err_t wifi_stop_driver(void);   // stop (mantém init)
esp_err_t wifi_deinit_driver(void); // opcional: desmonta tudo (caso extremo)

esp_err_t wifi_connect_credentials(const char *ssid, const char *pass, int timeout_ms);

bool wifi_is_active(void);     // started?
bool wifi_sta_connected(void); // associado ao AP?
bool wifi_has_ip(void);
esp_err_t wifi_wait_ip(TickType_t timeout); // aguarda IP (evento)

bool wifi_check_internet_tcp(const char *url, int timeout_ms); // check simples
bool wifi_check_internet_https(const char *url, int timeout_ms);
