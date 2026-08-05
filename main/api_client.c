#include "api_client.h"
#include "esp_log.h"
#include "cJSON.h"
#include "esp_task_wdt.h"
#include <string.h>
#include <stdlib.h>

#include "config.h" // onde ficam API_LOGIN_URL, API_DEBUG_PRINT_TOKEN, etc.

static const char *TAG = "api_client";
static char s_access_token[700] = {0}; // JWT pode ser grande
#define HTTP_RX_MAX_BYTES 8192

bool api_has_token(void) { return s_access_token[0] != '\0'; }
const char *api_get_token(void) { return s_access_token; }
void api_clear_token(void) { s_access_token[0] = '\0'; }

static void api_handle_auth_status(int status)
{
    if (status != 401 && status != 403)
        return;

    ESP_LOGW(TAG, "Token rejeitado pelo servidor (HTTP %d), refazendo login", status);
    api_clear_token();
    if (sys_event_group)
        xEventGroupClearBits(sys_event_group, SYS_AUTH_OK_BIT | SYS_WS_OK_BIT);
}

void api_set_auth_header(esp_http_client_handle_t client)
{
    if (!client || !api_has_token())
        return;

    char auth[800];
    snprintf(auth, sizeof(auth), "Bearer %s", s_access_token);
    esp_http_client_set_header(client, "Authorization", auth);
}

// Contexto por request (sem globais)
typedef struct
{
    char *buf;
    int len;
    int cap;
} http_rx_ctx_t;

// Handler de eventos HTTP (apenas pra receber dados)
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    if (evt->event_id == HTTP_EVENT_ON_CONNECTED || evt->event_id == HTTP_EVENT_ON_FINISH)
        (void)esp_task_wdt_reset();

    http_rx_ctx_t *rx = (http_rx_ctx_t *)evt->user_data;
    if (!rx)
        return ESP_OK;

    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data && evt->data_len > 0)
    {
        // garante capacidade (+1 pro '\0')
        int need = rx->len + evt->data_len + 1;
        if (need > HTTP_RX_MAX_BYTES)
        {
            ESP_LOGE(TAG, "Resposta HTTP excedeu limite (%d bytes)", need);
            return ESP_ERR_NO_MEM;
        }

        if (need > rx->cap)
        {
            int new_cap = (rx->cap == 0) ? 512 : rx->cap;
            while (new_cap < need)
                new_cap *= 2;

            char *tmp = realloc(rx->buf, new_cap);
            if (!tmp)
            {
                ESP_LOGE(TAG, "Sem memória para resposta HTTP (realloc %d)", new_cap);
                return ESP_ERR_NO_MEM;
            }
            rx->buf = tmp;
            rx->cap = new_cap;
        }

        memcpy(rx->buf + rx->len, evt->data, evt->data_len);
        rx->len += evt->data_len;
        rx->buf[rx->len] = '\0';
    }

    return ESP_OK;
}

// Faz login e armazena token internamente
esp_err_t api_login(const char *usuario, const char *senha)
{
    if (!usuario || !senha)
        return ESP_ERR_INVALID_ARG;

    // Monta JSON de login
    cJSON *root = cJSON_CreateObject();
    if (!root)
        return ESP_ERR_NO_MEM;

    // sua rota nova usa "usuario" e "password"
    cJSON_AddStringToObject(root, "usuario", usuario);
    cJSON_AddStringToObject(root, "password", senha);

    char *post_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!post_data)
        return ESP_ERR_NO_MEM;

    http_rx_ctx_t rx = {0};

    esp_http_client_config_t cfg = {
        .url = API_LOGIN_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
        .event_handler = http_event_handler,
        .user_data = &rx,
        .cert_pem = rootCaCerticate,
        // enquanto estiver estabilizando, recomendo:
        .keep_alive_enable = false,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client)
    {
        cJSON_free(post_data);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");

    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    ESP_LOGI(TAG, "Login POST (usuario=%s)", usuario);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);

    cJSON_free(post_data);
    esp_http_client_cleanup(client);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "perform falhou: %s", esp_err_to_name(err));
        free(rx.buf);
        return err;
    }

    if (status < 200 || status >= 300)
    {
        ESP_LOGW(TAG, "Login falhou HTTP=%d. Body=%s", status, (rx.buf ? rx.buf : ""));
        free(rx.buf);
        return ESP_FAIL;
    }

    if (!rx.buf || rx.len == 0)
    {
        ESP_LOGW(TAG, "HTTP 200 mas body vazio (rx.len=0)");
        free(rx.buf);
        return ESP_FAIL;
    }

    cJSON *j = cJSON_Parse(rx.buf);
    if (!j)
    {
        ESP_LOGW(TAG, "Body não é JSON válido: %s", rx.buf);
        free(rx.buf);
        return ESP_FAIL;
    }

    const cJSON *access = cJSON_GetObjectItemCaseSensitive(j, "access");
    if (!cJSON_IsString(access) || !access->valuestring)
    {
        ESP_LOGW(TAG, "JSON não trouxe 'access': %s", rx.buf);
        cJSON_Delete(j);
        free(rx.buf);
        return ESP_FAIL;
    }

    strlcpy(s_access_token, access->valuestring, sizeof(s_access_token));
    cJSON_Delete(j);
    free(rx.buf);

    ESP_LOGI(TAG, "HTTP status: %d", status);

    return ESP_OK;
}

// Envia POST de alerta de tanque
esp_err_t api_post_tanque(const char *mensagem, int unidade)
{
    if (!mensagem)
        return ESP_ERR_INVALID_ARG;

    cJSON *root = cJSON_CreateObject();
    if (!root)
        return ESP_ERR_NO_MEM;

    cJSON_AddStringToObject(root, "mensagem", mensagem);
    cJSON_AddNumberToObject(root, "unidade", unidade);

    char *post_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!post_data)
        return ESP_ERR_NO_MEM;

    http_rx_ctx_t rx = {0};

    esp_http_client_config_t cfg = {
        .url = API_ALERTA_TANQUE_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,
        .event_handler = http_event_handler,
        .user_data = &rx,
        .keep_alive_enable = false,
        .buffer_size = 512,
        .buffer_size_tx = 2048, // TX (headers + json + bearer)
        .cert_pem = rootCaCerticate,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client)
    {
        cJSON_free(post_data);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");

    // Se sua API exigir Bearer, isso ajuda. Se não exigir, não atrapalha.
    api_set_auth_header(client);

    esp_http_client_set_post_field(client, post_data, strlen(post_data));

    ESP_LOGI(TAG, "Enviando ALERTA tanque: %s", mensagem);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    api_handle_auth_status(status);

    esp_http_client_cleanup(client);
    cJSON_free(post_data);
    if (rx.buf)
        free(rx.buf);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Falha HTTP alerta: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Alerta HTTP status: %d", status);

    // No Insomnia deu certo: geralmente 200/201
    if (status == 200 || status == 201)
        return ESP_OK;

    return ESP_FAIL;
}

// Envia PATCH de tanque
esp_err_t api_patch_tanque(int tanque_id, const cJSON *patch_obj)
{
    if (tanque_id <= 0 || !patch_obj)
        return ESP_ERR_INVALID_ARG;

    // monta URL final: API_DADOS_TANQUE_URL + id
    char url[256];

    snprintf(url, sizeof(url), "%s%d", API_DADOS_TANQUE_URL, tanque_id);

    // char *patch_data = cJSON_PrintUnformatted((cJSON *)patch_obj);
    char *patch_data = cJSON_PrintUnformatted(patch_obj);
    if (!patch_data)
        return ESP_ERR_NO_MEM;

    http_rx_ctx_t rx = {0};

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_PATCH,
        .timeout_ms = 15000,
        .event_handler = http_event_handler,
        .user_data = &rx,
        .cert_pem = rootCaCerticate,
        // Evita o warning de header pequeno (Bearer token é grande)
        .keep_alive_enable = false,
        .buffer_size = 1024,
        .buffer_size_tx = 2048,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client)
    {
        cJSON_free(patch_data);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");

    // Normalmente essa rota vai exigir token (se não exigir, não atrapalha)
    api_set_auth_header(client);

    esp_http_client_set_post_field(client, patch_data, strlen(patch_data));

    ESP_LOGI(TAG, "PATCH tanque id=%d url=%s body=%s", tanque_id, url, patch_data);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    api_handle_auth_status(status);

    esp_http_client_cleanup(client);
    cJSON_free(patch_data);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "PATCH falhou: %s", esp_err_to_name(err));
        free(rx.buf);
        return err;
    }

    ESP_LOGI(TAG, "PATCH status: %d", status);

    // se quiser depurar resposta:
    // ESP_LOGI(TAG, "PATCH resp: %s", (rx.buf ? rx.buf : ""));

    free(rx.buf);

    return (status >= 200 && status < 300) ? ESP_OK : ESP_FAIL;
}

esp_err_t api_patch_bomba_status(int bomba_id, bool ligada)
{
    if (bomba_id <= 0)
        return ESP_ERR_INVALID_ARG;

    // URL final: API_BOMBA_URL + id
    char url[256];
    snprintf(url, sizeof(url), "%s%d", API_BOMBA_URL, bomba_id);

    // body: {"status":1} ou {"status":0}
    cJSON *root = cJSON_CreateObject();
    if (!root)
        return ESP_ERR_NO_MEM;

    cJSON_AddNumberToObject(root, "status", ligada ? 1 : 0);

    char *patch_data = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    if (!patch_data)
        return ESP_ERR_NO_MEM;

    http_rx_ctx_t rx = {0};

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_PATCH,
        .timeout_ms = 15000,
        .event_handler = http_event_handler,
        .user_data = &rx,
        .cert_pem = rootCaCerticate,

        .keep_alive_enable = false,
        .buffer_size = 1024,
        .buffer_size_tx = 2048,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client)
    {
        cJSON_free(patch_data);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");

    // Se sua rota exigir token, isso resolve. Se não exigir, não atrapalha.
    api_set_auth_header(client);

    esp_http_client_set_post_field(client, patch_data, strlen(patch_data));

    ESP_LOGI(TAG, "PATCH bomba id=%d url=%s body=%s", bomba_id, url, patch_data);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    api_handle_auth_status(status);

    esp_http_client_cleanup(client);
    cJSON_free(patch_data);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "PATCH bomba falhou: %s", esp_err_to_name(err));
        free(rx.buf);
        return err;
    }

    ESP_LOGI(TAG, "PATCH bomba status HTTP: %d", status);

    free(rx.buf);

    return (status >= 200 && status < 300) ? ESP_OK : ESP_FAIL;
}

esp_err_t api_patch_bomba(int bomba_id, const cJSON *patch_obj)
{
    if (bomba_id <= 0 || !patch_obj)
        return ESP_ERR_INVALID_ARG;

    // monta URL final: API_BOMBA_URL + id
    char url[256];
    snprintf(url, sizeof(url), "%s%d", API_BOMBA_URL, bomba_id);

    // char *patch_data = cJSON_PrintUnformatted((cJSON *)patch_obj);
    char *patch_data = cJSON_PrintUnformatted(patch_obj);

    if (!patch_data)
        return ESP_ERR_NO_MEM;

    http_rx_ctx_t rx = {0};

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_PATCH,
        .timeout_ms = 5000,
        .event_handler = http_event_handler,
        .user_data = &rx,
        .cert_pem = rootCaCerticate,
        .keep_alive_enable = false,
        .buffer_size = 1024,
        .buffer_size_tx = 2048,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client)
    {
        cJSON_free(patch_data);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");

    api_set_auth_header(client);

    esp_http_client_set_post_field(client, patch_data, strlen(patch_data));

    ESP_LOGI(TAG, "PATCH bomba id=%d url=%s body=%s", bomba_id, url, patch_data);

    int64_t t0 = esp_timer_get_time();
    ESP_LOGI(TAG, "PATCH bomba start url=%s", url);

    esp_err_t err = esp_http_client_perform(client);

    int64_t dt_ms = (esp_timer_get_time() - t0) / 1000;
    ESP_LOGI(TAG, "PATCH bomba fim err=%s tempo=%lld ms", esp_err_to_name(err), dt_ms);

    int status = esp_http_client_get_status_code(client);
    api_handle_auth_status(status);

    esp_http_client_cleanup(client);
    cJSON_free(patch_data);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "PATCH bomba falhou: %s", esp_err_to_name(err));
        free(rx.buf);
        return err;
    }

    ESP_LOGI(TAG, "PATCH bomba status: %d", status);

    free(rx.buf);

    return (status >= 200 && status < 300) ? ESP_OK : ESP_FAIL;
}

esp_err_t api_patch_bomba_controle_false(int controle_id)
{
    if (controle_id <= 0)
        return ESP_ERR_INVALID_ARG;

    char url[256];
    snprintf(url, sizeof(url), "%s%d", API_BOMBA_CONTROLE_URL, controle_id);

    const char *patch_data = "{\"comando\":false}";
    http_rx_ctx_t rx = {0};

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_PATCH,
        .timeout_ms = 5000,
        .event_handler = http_event_handler,
        .user_data = &rx,
        .cert_pem = rootCaCerticate,
        .keep_alive_enable = false,
        .buffer_size = 1024,
        .buffer_size_tx = 512,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client)
        return ESP_FAIL;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");
    api_set_auth_header(client);
    esp_http_client_set_post_field(client, patch_data, strlen(patch_data));

    ESP_LOGI(TAG, "PATCH bomba controle id=%d url=%s body=%s", controle_id, url, patch_data);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    api_handle_auth_status(status);

    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PATCH bomba controle falhou: %s", esp_err_to_name(err));
        free(rx.buf);
        return err;
    }

    ESP_LOGI(TAG, "PATCH bomba controle status: %d", status);
    free(rx.buf);

    return (status >= 200 && status < 300) ? ESP_OK : ESP_FAIL;
}

esp_err_t api_get_tanque_leitura(int tanque_id, cJSON **out_root)
{
    if (tanque_id <= 0 || !out_root)
        return ESP_ERR_INVALID_ARG;
    *out_root = NULL;

    char url[256];
    snprintf(url, sizeof(url), "%s%d", API_TANQUE_LEITURA_URL, tanque_id);

    http_rx_ctx_t rx = {0};

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 15000,
        .event_handler = http_event_handler,
        .user_data = &rx,
        .keep_alive_enable = false,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
        .cert_pem = rootCaCerticate,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client)
        return ESP_FAIL;

    esp_http_client_set_header(client, "Accept", "application/json");
    esp_http_client_set_header(client, "Connection", "close");
    esp_http_client_set_header(client, "Accept-Encoding", "identity");

    api_set_auth_header(client);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    api_handle_auth_status(status);

    esp_http_client_cleanup(client);

    if (err != ESP_OK)
    {
        free(rx.buf);
        return err;
    }

    if (!(status >= 200 && status < 300) || !rx.buf)
    {
        free(rx.buf);
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(rx.buf);
    free(rx.buf);

    if (!root)
        return ESP_FAIL;

    *out_root = root;
    return ESP_OK;
}
