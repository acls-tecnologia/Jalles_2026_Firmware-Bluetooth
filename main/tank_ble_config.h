#ifndef TANK_BLE_CONFIG_H
#define TANK_BLE_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef void (*tank_ble_send_fn_t)(const char *message);

#define TANK_BLE_DEVICE_NAME "TANQUE-Jalles-BLE"
#define TANK_BLE_SERVICE_UUID 0xFFF0
#define TANK_BLE_RX_UUID 0xFFF1
#define TANK_BLE_TX_UUID 0xFFF2

void tank_ble_config_send_snapshot(tank_ble_send_fn_t send_fn);
void tank_ble_config_handle_message(const char *message, tank_ble_send_fn_t send_fn);
void tank_ble_config_handle_binary(const uint8_t *data, size_t length, tank_ble_send_fn_t send_fn);

#endif
