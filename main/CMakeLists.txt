idf_component_register(SRCS
    "main.c"
    "config.c"
    "ads1115.c"
    "mcp23017.c"
    "wifi.c"
    "api_client.c"
    "ws_client.c"
    "lora.c"
    "bluetooth.c"
    "tank_ble_config.c"
    "watchdog_rtc_raw.c"
    "firmware_ota.c"
   
    INCLUDE_DIRS "."
    REQUIRES bt
             json
             nvs_flash
             esp_http_client
             app_update
             esp_https_ota
             esp_websocket_client
             esp_wifi
             esp_event
             esp_netif
             esp_system
             esp_timer
             esp_driver_uart
             esp_driver_gpio
             esp_driver_i2c
             esp_driver_ledc
             lwip)
