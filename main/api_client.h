#pragma once
#include "esp_err.h"
#include "esp_http_client.h"
#include <stdbool.h>
#include "cJSON.h"

esp_err_t api_login(const char *usuario, const char *senha);

bool api_has_token(void);
const char *api_get_token(void);
void api_clear_token(void);

// Reutilizar em qualquer request autenticada:
void api_set_auth_header(esp_http_client_handle_t client);

esp_err_t api_post_tanque(const char *mensagem, int unidade);

esp_err_t api_patch_tanque(int tanque_id, const cJSON *patch_obj);

esp_err_t api_patch_bomba_status(int bomba_id, bool ligada);

esp_err_t api_patch_bomba(int bomba_id, const cJSON *patch_obj);

esp_err_t api_patch_bomba_controle_false(int controle_id);

esp_err_t api_get_tanque_leitura(int tanque_id, cJSON **out_root);
