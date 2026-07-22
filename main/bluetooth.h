#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define BLUETOOTH_CONFIG_TIMEOUT_DEFAULT_MS 0U
#define BLUETOOTH_CONFIG_TIMEOUT_FOREVER_MS UINT32_MAX

#ifdef __cplusplus
extern "C" {
#endif

void bluetooth_init(void);
esp_err_t bluetooth_config_start(uint32_t timeout_ms);
void bluetooth_config_stop(void);
bool bluetooth_config_is_active(void);
void bluetooth_send_message(const char *message);

void bt_client_connected_callback(void);
void bt_client_disconnected_callback(void);

#ifdef __cplusplus
}
#endif

#endif
