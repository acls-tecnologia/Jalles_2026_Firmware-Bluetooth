#include "tank_ble_config.h"

#include "config.h"
#include "firmware_ota.h"
#include "cJSON.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "TANK_BLE_CFG";

static void ota_restart_task(void *arg) {
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
}

static void send_text(tank_ble_send_fn_t send_fn, const char *message) {
    if (send_fn) {
        send_fn(message);
        return;
    }

    ESP_LOGI(TAG, "Resposta BLE preparada: %s", message ? message : "");
}

static const cJSON *get_number_item(const cJSON *root, const char *a, const char *b, const char *c) {
    const cJSON *item = cJSON_GetObjectItem(root, a);
    if (!cJSON_IsNumber(item) && b)
        item = cJSON_GetObjectItem(root, b);
    if (!cJSON_IsNumber(item) && c)
        item = cJSON_GetObjectItem(root, c);
    return cJSON_IsNumber(item) ? item : NULL;
}

static const cJSON *get_string_item(const cJSON *root, const char *a, const char *b, const char *c) {
    const cJSON *item = cJSON_GetObjectItem(root, a);
    if (!cJSON_IsString(item) && b)
        item = cJSON_GetObjectItem(root, b);
    if (!cJSON_IsString(item) && c)
        item = cJSON_GetObjectItem(root, c);
    return cJSON_IsString(item) ? item : NULL;
}

static int read_pump_id_from_array(const cJSON *array, int index) {
    const cJSON *item = cJSON_GetArrayItem(array, index);
    if (cJSON_IsNumber(item))
        return item->valueint;

    const cJSON *id = item ? cJSON_GetObjectItem(item, "id") : NULL;
    return cJSON_IsNumber(id) ? id->valueint : 0;
}

static bool fill_pumps_from_json(const cJSON *json, int pump_count, int out_ids[MAX_BOMBAS]) {
    for (int i = 0; i < MAX_BOMBAS; i++)
        out_ids[i] = 0;

    const cJSON *pump_ids = cJSON_GetObjectItem(json, "pump_ids");
    const cJSON *bombas = cJSON_GetObjectItem(json, "bombas");
    const cJSON *bomba_id = cJSON_GetObjectItem(json, "bomba_id");
    const cJSON *array =
        cJSON_IsArray(pump_ids) ? pump_ids : (cJSON_IsArray(bombas) ? bombas : (cJSON_IsArray(bomba_id) ? bomba_id : NULL));

    for (int i = 0; i < pump_count; i++) {
        int id = 0;

        if (array) {
            id = read_pump_id_from_array(array, i);
        } else {
            char key[4];
            snprintf(key, sizeof(key), "b%d", i);
            const cJSON *item = cJSON_GetObjectItem(json, key);
            if (cJSON_IsNumber(item))
                id = item->valueint;
        }

        if (id <= 0)
            return false;

        out_ids[i] = id;
    }

    return true;
}

void tank_ble_config_send_snapshot(tank_ble_send_fn_t send_fn) {
    char device_response[192];
    snprintf(device_response, sizeof(device_response),
             "{\"ok\":true,\"type\":\"ConfigDevice\",\"device_role\":\"TANK\",\"ota_file_id\":%d,"
             "\"ota_protocol\":1,\"firmware_version\":\"%s\"}",
             OTA_FILE_ID, firmware_ota_current_version());
    send_text(send_fn, device_response);
    vTaskDelay(pdMS_TO_TICKS(40));

    cJSON *root = cJSON_CreateObject();
    cJSON *wifi = cJSON_CreateObject();
    cJSON *tank = cJSON_CreateObject();
    cJSON *pump_ids = cJSON_CreateArray();

    if (!root || !wifi || !tank || !pump_ids) {
        cJSON_Delete(root);
        cJSON_Delete(wifi);
        cJSON_Delete(tank);
        cJSON_Delete(pump_ids);
        send_text(send_fn, "{\"ok\":false,\"error\":\"no_mem\"}");
        return;
    }

    cJSON_AddBoolToObject(root, "ok", true);
    cJSON_AddStringToObject(root, "type", "Config");
    cJSON_AddStringToObject(root, "target", "tank");

    cJSON_AddStringToObject(wifi, "ssid", wifi_ssid);
    cJSON_AddStringToObject(wifi, "password", wifi_password);
    cJSON_AddItemToObject(root, "wifi", wifi);

    cJSON_AddStringToObject(tank, "name", g_cfg.nome_tanque);
    cJSON_AddStringToObject(tank, "api_user", API_USER);
    cJSON_AddStringToObject(tank, "api_pass", API_PASS);
    cJSON_AddNumberToObject(tank, "unit_id", g_cfg.unidade_id);
    cJSON_AddNumberToObject(tank, "tank_id", (g_cfg.tanque_id > 0) ? g_cfg.tanque_id : PROVISION_TANQUE_ID);
    cJSON_AddNumberToObject(tank, "lora_gtw_id", LORA_GTW_ID);
    cJSON_AddNumberToObject(tank, "pump_count", g_cfg.qtd_bombas);

    for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++) {
        cJSON_AddItemToArray(pump_ids, cJSON_CreateNumber(g_cfg.bomba_id[i]));
    }

    cJSON_AddItemToObject(tank, "pump_ids", pump_ids);
    cJSON_AddItemToObject(root, "tank", tank);

    char *response = cJSON_PrintUnformatted(root);
    if (response) {
        send_text(send_fn, response);
        cJSON_free(response);
    } else {
        send_text(send_fn, "{\"ok\":false,\"error\":\"json_print\"}");
    }

    cJSON_Delete(root);
}

static void handle_wifi_config(const cJSON *json, tank_ble_send_fn_t send_fn) {
    const cJSON *ssid = cJSON_GetObjectItem(json, "SSID");
    if (!cJSON_IsString(ssid))
        ssid = cJSON_GetObjectItem(json, "ssid");

    const cJSON *password = cJSON_GetObjectItem(json, "password");

    if (!cJSON_IsString(ssid) || !ssid->valuestring || !cJSON_IsString(password) || !password->valuestring) {
        send_text(send_fn, "{\"ok\":false,\"error\":\"invalid_wifi\"}");
        return;
    }

    esp_err_t err = cfg_wifi_save(ssid->valuestring, password->valuestring);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar WiFi: %s", esp_err_to_name(err));
        send_text(send_fn, "{\"ok\":false,\"error\":\"wifi_save_failed\"}");
        return;
    }

    tank_ble_config_send_snapshot(send_fn);
}

static void handle_tank_config(const cJSON *json, tank_ble_send_fn_t send_fn) {
    const cJSON *tank_id = get_number_item(json, "tank_id", "tanque", "tanque_id");
    if (!tank_id)
        tank_id = get_number_item(json, "provision_tanque_id", "PROVISION_TANQUE_ID", NULL);
    const cJSON *unit_id = get_number_item(json, "unit_id", "unidade", "unidade_id");
    const cJSON *qtd = get_number_item(json, "pump_count", "qtd_bombas", "qtdb");
    const cJSON *api_user = get_string_item(json, "api_user", "API_USER", "usuario");
    const cJSON *api_pass = get_string_item(json, "api_pass", "API_PASS", "senha");
    const cJSON *lora_gtw = get_number_item(json, "lora_gtw_id", "LORA_GTW_ID", "gtw_lora");
    const cJSON *name = cJSON_GetObjectItem(json, "name");

    if (!cJSON_IsString(name))
        name = cJSON_GetObjectItem(json, "nome_tanque");
    if (!cJSON_IsString(name))
        name = cJSON_GetObjectItem(json, "nome");

    if (!tank_id || tank_id->valueint <= 0 || !unit_id || unit_id->valueint <= 0 || !qtd) {
        send_text(send_fn, "{\"ok\":false,\"error\":\"invalid_tank\"}");
        return;
    }

    if (!cJSON_IsString(api_user) || !api_user->valuestring || api_user->valuestring[0] == '\0' ||
        strlen(api_user->valuestring) >= API_USER_MAX_LEN) {
        send_text(send_fn, "{\"ok\":false,\"error\":\"invalid_api_user\"}");
        return;
    }

    const char *selected_api_pass = DEFAULT_API_PASSWORD;
    if (cJSON_IsString(api_pass) && api_pass->valuestring && api_pass->valuestring[0] != '\0')
        selected_api_pass = api_pass->valuestring;

    if (strlen(selected_api_pass) >= API_PASS_MAX_LEN) {
        send_text(send_fn, "{\"ok\":false,\"error\":\"invalid_api_pass\"}");
        return;
    }

    if (!lora_gtw || lora_gtw->valueint <= 0) {
        send_text(send_fn, "{\"ok\":false,\"error\":\"invalid_lora_gtw\"}");
        return;
    }

    int pump_count = qtd->valueint;
    if (pump_count < 0 || pump_count > MAX_BOMBAS) {
        send_text(send_fn, "{\"ok\":false,\"error\":\"invalid_pump_count\"}");
        return;
    }

    int pump_ids[MAX_BOMBAS] = {0};
    if (!fill_pumps_from_json(json, pump_count, pump_ids)) {
        send_text(send_fn, "{\"ok\":false,\"error\":\"invalid_pumps\"}");
        return;
    }

    device_cfg_t next = g_cfg;
    bool had_cfg = next.device_id[0] != '\0';

    if (!had_cfg) {
        strlcpy(next.device_id, API_USER, sizeof(next.device_id));
        next.ativo = true;
        next.nivel_maximo = 100;
        next.nivel_minimo = 0;
    }

    next.provisionado = true;
    next.tanque_id = tank_id->valueint;
    next.unidade_id = unit_id->valueint;
    next.lora_gtw_id = lora_gtw->valueint;
    strlcpy(next.api_user, api_user->valuestring, sizeof(next.api_user));
    strlcpy(next.api_pass, selected_api_pass, sizeof(next.api_pass));
    strlcpy(next.device_id, next.api_user, sizeof(next.device_id));
    next.qtd_bombas = (uint8_t)pump_count;
    next.perfil = (pump_count > 0) ? TANQUE_FULL : TANQUE_NIVEL;

    for (int i = 0; i < MAX_BOMBAS; i++)
        next.bomba_id[i] = pump_ids[i];

    if (cJSON_IsString(name) && name->valuestring && name->valuestring[0] != '\0') {
        strlcpy(next.nome_tanque, name->valuestring, sizeof(next.nome_tanque));
    } else if (next.nome_tanque[0] == '\0') {
        cfg_guess_nome_tanque_from_user(next.nome_tanque, sizeof(next.nome_tanque));
    }

    if (next.nivel_maximo <= 0)
        next.nivel_maximo = 100;

    esp_err_t err = cfg_save(&next);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Falha ao salvar configuracao do tanque: %s", esp_err_to_name(err));
        send_text(send_fn, "{\"ok\":false,\"error\":\"tank_save_failed\"}");
        return;
    }

    g_cfg = next;
    cfg_log(&g_cfg);
    tank_ble_config_send_snapshot(send_fn);
}

void tank_ble_config_handle_message(const char *message, tank_ble_send_fn_t send_fn) {
    if (!message || message[0] == '\0') {
        send_text(send_fn, "{\"ok\":false,\"error\":\"empty_message\"}");
        return;
    }

    cJSON *json = cJSON_Parse(message);
    if (!json) {
        send_text(send_fn, "{\"ok\":false,\"error\":\"invalid_json\"}");
        return;
    }

    cJSON *type = cJSON_GetObjectItem(json, "type");
    if (!cJSON_IsString(type))
        type = cJSON_GetObjectItem(json, "cmd");

    const char *cmd = cJSON_GetStringValue(type);

    if (cmd && strcmp(cmd, "GetConfig") == 0) {
        tank_ble_config_send_snapshot(send_fn);
    } else if (cmd && strcmp(cmd, "TankWifi") == 0) {
        handle_wifi_config(json, send_fn);
    } else if (cmd && strcmp(cmd, "TankConfig") == 0) {
        handle_tank_config(json, send_fn);
    } else if (cmd && strcmp(cmd, "BleOtaBegin") == 0) {
        const cJSON *size = cJSON_GetObjectItem(json, "size");
        if (!cJSON_IsNumber(size) || size->valuedouble <= 0 || size->valuedouble > (double)UINT32_MAX ||
            size->valuedouble != (double)(uint32_t)size->valuedouble) {
            send_text(send_fn, "{\"ok\":false,\"type\":\"BleOtaError\",\"error\":\"invalid_size\"}");
        } else {
            esp_err_t err = firmware_ota_ble_begin((uint32_t)size->valuedouble);
            if (err == ESP_OK) {
                send_text(send_fn, "{\"ok\":true,\"type\":\"BleOtaReady\",\"offset\":0}");
            } else {
                char response[128];
                snprintf(response, sizeof(response),
                         "{\"ok\":false,\"type\":\"BleOtaError\",\"error\":\"%s\"}", esp_err_to_name(err));
                send_text(send_fn, response);
            }
        }
    } else if (cmd && strcmp(cmd, "BleOtaFinish") == 0) {
        char version[FW_VERSION_TEXT_MAX] = {0};
        esp_err_t err = firmware_ota_ble_finish(version, sizeof(version));
        if (err == ESP_OK) {
            char response[160];
            snprintf(response, sizeof(response),
                     "{\"ok\":true,\"type\":\"BleOtaComplete\",\"version\":\"%s\",\"restarting\":true}",
                     version);
            send_text(send_fn, response);
            if (xTaskCreate(ota_restart_task, "ble_ota_restart", 2048, NULL, 5, NULL) != pdPASS) {
                ESP_LOGE(TAG, "Falha ao criar task de reinicio OTA");
            }
        } else {
            char response[128];
            snprintf(response, sizeof(response),
                     "{\"ok\":false,\"type\":\"BleOtaError\",\"error\":\"%s\"}", esp_err_to_name(err));
            send_text(send_fn, response);
        }
    } else if (cmd && strcmp(cmd, "BleOtaCancel") == 0) {
        firmware_ota_ble_abort();
        send_text(send_fn, "{\"ok\":true,\"type\":\"BleOtaCancelled\"}");
    } else if (cmd && strcmp(cmd, "Reset") == 0) {
        send_text(send_fn, "{\"ok\":true,\"status\":\"reset\"}");
        cJSON_Delete(json);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
        return;
    } else {
        send_text(send_fn, "{\"ok\":false,\"error\":\"unknown_command\"}");
    }

    cJSON_Delete(json);
}

void tank_ble_config_handle_binary(const uint8_t *data, size_t length, tank_ble_send_fn_t send_fn) {
    if (!data || length <= 5 || data[0] != 0xA1) {
        send_text(send_fn, "{\"ok\":false,\"type\":\"BleOtaError\",\"error\":\"invalid_packet\"}");
        return;
    }

    uint32_t offset = (uint32_t)data[1] | ((uint32_t)data[2] << 8) | ((uint32_t)data[3] << 16) |
                      ((uint32_t)data[4] << 24);
    uint8_t percent = 0;
    esp_err_t err = firmware_ota_ble_write(offset, data + 5, length - 5, &percent);
    if (err != ESP_OK) {
        char response[128];
        snprintf(response, sizeof(response),
                 "{\"ok\":false,\"type\":\"BleOtaError\",\"error\":\"%s\",\"offset\":%lu}",
                 esp_err_to_name(err), (unsigned long)offset);
        send_text(send_fn, response);
    }
}
