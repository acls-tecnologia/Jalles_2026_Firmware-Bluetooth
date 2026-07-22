#include "ws_client.h"
#include "config.h" // para acessar as variáveis globais

#include "esp_websocket_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

#include "cJSON.h"

static const char *TAG = "WS";

static int ws_bomba_index_from_id(int bomba_id)
{
    for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
    {
        if (g_cfg.bomba_id[i] == bomba_id)
            return i;
    }
    return -1;
}

/*
 * Handle global do WebSocket
 */
static esp_websocket_client_handle_t s_ws = NULL;

/*
 * Buffer para URI completa (base + token)
 */
static char s_ws_uri[720] = {0};

/*
 * Token salvo para uso na URI
 * (não é mais enviado em payload)
 */
static char s_token[620] = {0};

/*
 * =========================
 *  EVENT HANDLER DO WS
 * =========================
 * ONLINE  -> WEBSOCKET_EVENT_CONNECTED
 * OFFLINE -> WEBSOCKET_EVENT_DISCONNECTED
 *
 * O backend já trata o status apenas com a conexão,
 * portanto NÃO enviamos mensagens de "online".
 */

static void websocket_event_handler(void *handler_args,
                                    esp_event_base_t base,
                                    int32_t event_id,
                                    void *event_data)
{
    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket CONNECTED (ONLINE)");
        if (sys_event_group)
            xEventGroupSetBits(sys_event_group, SYS_WS_OK_BIT);
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "WebSocket DISCONNECTED (OFFLINE)");
        if (sys_event_group)
            xEventGroupClearBits(sys_event_group, SYS_WS_OK_BIT);
        break;

    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGW(TAG, "WebSocket ERROR");
        if (sys_event_group)
            xEventGroupClearBits(sys_event_group, SYS_WS_OK_BIT);
        break;

    case WEBSOCKET_EVENT_DATA:
    {
        esp_websocket_event_data_t *e = (esp_websocket_event_data_t *)event_data;
        if (e && e->data_ptr && e->data_len > 0)
        {
            ESP_LOGI(TAG, "DATA %d bytes: %.*s",
                     e->data_len,
                     e->data_len,
                     (const char *)e->data_ptr);

            cJSON *root = cJSON_ParseWithLength(e->data_ptr, e->data_len);

            if (!root)
                break;

            cJSON *ev = cJSON_GetObjectItem(root, "event");
            cJSON *data = cJSON_GetObjectItem(root, "data");

            if (cJSON_IsString(ev) && data)
            {
                const char *evento = ev->valuestring;

                if (strcmp(evento, "ControlerBomba_edit") == 0)
                {
                    // tanque sem bombas
                    if (g_cfg.qtd_bombas == 0)
                    {
                        ESP_LOGW(TAG, "WS: tanque sem bombas (qtd=0) -> ignorado");
                        cJSON_Delete(root);
                        break;
                    }

                    cJSON *comando = cJSON_GetObjectItem(data, "comando");
                    cJSON *controleId = cJSON_GetObjectItem(data, "id");
                    cJSON *bombaId = cJSON_GetObjectItem(data, "bombaId");
                    cJSON *vazao = cJSON_GetObjectItemCaseSensitive(data, "vazao");

                    // Se não vier bombaId, trata como broadcast
                    bool broadcast = !cJSON_IsNumber(bombaId);
                    int idx = 0;

                    if (!broadcast)
                    {
                        idx = ws_bomba_index_from_id(bombaId->valueint);
                        if (idx < 0)
                        {
                            ESP_LOGW(TAG,
                                     "WS: bombaId=%d nao pertence a este tanque -> ignorado",
                                     bombaId->valueint);
                            cJSON_Delete(root);
                            break;
                        }

                        bomba_registrar_controle_id(idx, cJSON_IsNumber(controleId) ? controleId->valueint : bombaId->valueint);
                    }

                    // status vindo do front
                    cJSON *st = cJSON_GetObjectItem(data, "stausBOmba");
                    if (!st)
                        st = cJSON_GetObjectItem(data, "statusBomba");

                    // ------------------------------------------------------
                    // 1) Se veio vazao, salva/aplica o setpoint
                    //    MAS NAO SAI DO HANDLER
                    // ------------------------------------------------------
                    if (vazao && cJSON_IsNumber(vazao))
                    {
                        float valor_vazao = (float)vazao->valuedouble;

                        if (!broadcast && bombaId->valueint == BOMBA_PWM_ID)
                        {
                            pwm_processar_novo_setpoint(valor_vazao);
                            pwm_agendar_sync_vazao(bombaId->valueint, valor_vazao);

                            ESP_LOGI(TAG,
                                     "WS: setpoint de vazao recebido para bomba PWM id=%d -> %.2f%%",
                                     bombaId->valueint,
                                     (double)valor_vazao);
                        }
                        else
                        {
                            ESP_LOGI(TAG,
                                     "WS: vazao ignorada para bombaId=%d (nao eh a bomba PWM)",
                                     broadcast ? -1 : bombaId->valueint);
                        }
                    }

                    // ------------------------------------------------------
                    // 2) So processa comando de bomba quando comando=true
                    // ------------------------------------------------------
                    if (cJSON_IsBool(comando) && cJSON_IsTrue(comando))
                    {
                        if (broadcast)
                        {
                            for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
                                g_controle_bomba_habilitado[i] = true;

                            ESP_LOGW(TAG, "Controle habilitado=1 (broadcast)");
                        }
                        else
                        {
                            g_controle_bomba_habilitado[idx] = true;

                            ESP_LOGW(TAG,
                                     "Controle habilitado=1 (idx=%d id=%d)",
                                     idx,
                                     g_cfg.bomba_id[idx]);
                        }

                        if (cJSON_IsNumber(st))
                        {
                            int status_atual = (st->valueint != 0) ? 1 : 0;
                            bool esta_ligada_front = (status_atual == 1);
                            bool desejado = !esta_ligada_front; // toggle

                            int start = broadcast ? 0 : idx;
                            int end = broadcast ? (int)g_cfg.qtd_bombas : (idx + 1);

                            bool emerg_global_ativa = false;
                            for (int k = 0; k < g_cfg.qtd_bombas && k < MAX_BOMBAS; k++)
                            {
                                if (entrada_emergencia_acionada(g_emergencia[k]))
                                {
                                    emerg_global_ativa = true;
                                    break;
                                }
                            }

                            for (int i = start; i < end && i < MAX_BOMBAS; i++)
                            {
                                bool ctrl = g_controle_bomba_habilitado[i];
                                bool remoto_ativo = entrada_remoto_ativo(g_local_remoto[i]);
                                bool real_ligada = g_status_bomba[i] ? true : false;

                                bool pode_processar_cmd =
                                    ctrl &&
                                    remoto_ativo &&
                                    !emerg_global_ativa;

                                if (!pode_processar_cmd)
                                {
                                    ESP_LOGW(TAG,
                                             "WS: IGNORADO (i=%d id=%d ctrl=%d lr=%d remoto=%d emerg_in=%d emerg_global=%d) front=%d (quer=%s)",
                                             i,
                                             g_cfg.bomba_id[i],
                                             ctrl ? 1 : 0,
                                             g_local_remoto[i],
                                             remoto_ativo ? 1 : 0,
                                             g_emergencia[i],
                                             emerg_global_ativa ? 1 : 0,
                                             status_atual,
                                             desejado ? "LIGAR" : "DESLIGAR");

                                    if (desejado)
                                    {
                                        const char *motivo = emerg_global_ativa ? "emergencia global ativa"
                                                            : (!remoto_ativo) ? "bomba em manual/local"
                                                            : (!ctrl)         ? "controle nao habilitado"
                                                                              : "bloqueio de seguranca";

                                        enviar_alerta_bomba_api_online(i,
                                                                       "FALHA AO LIGAR BOMBA",
                                                                       motivo);
                                    }
                                }
                                else
                                {

                                    if ((g_status_bomba_desejado[i] == desejado) && (real_ligada == desejado))
                                    {
                                        ESP_LOGI(TAG,
                                                 "WS: redundante (i=%d id=%d front=%d desejado=%d real=%d) -> sincronizando status real",
                                                 i,
                                                 g_cfg.bomba_id[i],
                                                 status_atual,
                                                 desejado ? 1 : 0,
                                                 real_ligada ? 1 : 0);

                                        bool sync_ok = sincronizar_status_bomba_real(i);
                                        ESP_LOGI(TAG,
                                                 "WS: sync status real bomba id=%d -> %s",
                                                 g_cfg.bomba_id[i],
                                                 sync_ok ? "OK" : "FAIL");
                                    }
                                    else
                                    {
                                        g_status_bomba_desejado[i] = desejado;

                                        ESP_LOGW(TAG,
                                                 "WS: bomba i=%d id=%d front=%s -> comando=%s",
                                                 i,
                                                 g_cfg.bomba_id[i],
                                                 esta_ligada_front ? "LIGADA" : "DESLIGADA",
                                                 desejado ? "LIGAR" : "DESLIGAR");

                                        ESP_LOGW(TAG,
                                                 "WS: SET i=%d id=%d ctrl=%d desejado=%d lr=%d remoto=%d emerg_global=%d real=%d",
                                                 i,
                                                 g_cfg.bomba_id[i],
                                                 g_controle_bomba_habilitado[i] ? 1 : 0,
                                                 g_status_bomba_desejado[i] ? 1 : 0,
                                                 g_local_remoto[i],
                                                 remoto_ativo ? 1 : 0,
                                                 emerg_global_ativa ? 1 : 0,
                                                 g_status_bomba[i] ? 1 : 0);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        ESP_LOGI(TAG, "WS: sem comando de bomba (comando=false ou ausente)");
                    }

                    pwm_atualizar_saida_por_estado();
                }
            }

            cJSON_Delete(root);
        }
        break;
    }

    default:
        break;
    }
}

/*
 * Retorna se o WebSocket está conectado
 */
bool ws_client_is_connected(void)
{
    return s_ws && esp_websocket_client_is_connected(s_ws);
}

/*
 * Inicia o WebSocket com token na URI
 */
esp_err_t ws_client_start(const char *token)
{
    if (!token || token[0] == '\0')
        return ESP_ERR_INVALID_ARG;

    // Já iniciado
    if (s_ws)
        return ESP_OK;

    strlcpy(s_token, token, sizeof(s_token));

    /*
     * Monta URI com token como query string
     * Ex: wss://api.exemplo/ws?token=XYZ
     */
    snprintf(s_ws_uri,
             sizeof(s_ws_uri),
             "%s?token=%s",
             API_WS_BASE_URL,
             s_token);

    esp_websocket_client_config_t cfg = {
        .uri = s_ws_uri,
        .reconnect_timeout_ms = 5000,
        .network_timeout_ms = 30000,
        .cert_pem = rootCaCerticate,
        // Keep-alive cuida do heartbeat automaticamente
        .keep_alive_enable = true,
        .keep_alive_idle = 30,     // 30s sem tráfego -> começa keepalive
        .keep_alive_interval = 10, // tenta a cada 10s
        .keep_alive_count = 3,     // 3 tentativas -> considera morto

        // Reconexão automática habilitada
        .disable_auto_reconnect = false,
    };

    s_ws = esp_websocket_client_init(&cfg);
    if (!s_ws)
    {
        ESP_LOGE(TAG, "Falha ao inicializar WebSocket");
        return ESP_FAIL;
    }

    esp_websocket_register_events(
        s_ws,
        WEBSOCKET_EVENT_ANY,
        websocket_event_handler,
        NULL);

    esp_err_t err = esp_websocket_client_start(s_ws);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG,
                 "Falha ao iniciar WebSocket: %s",
                 esp_err_to_name(err));

        esp_websocket_client_destroy(s_ws);
        s_ws = NULL;
        return err;
    }

    return ESP_OK;
}

/*
 * Para e destrói o WebSocket
 */
void ws_client_stop(void)
{
    if (!s_ws)
        return;

    ESP_LOGW(TAG, "Parando WebSocket...");

    esp_websocket_client_stop(s_ws);
    esp_websocket_client_destroy(s_ws);
    s_ws = NULL;

    if (sys_event_group)
        xEventGroupClearBits(sys_event_group, SYS_WS_OK_BIT);

    s_ws_uri[0] = '\0';

    // Token pode ser mantido para reconnect futuro
    // s_token[0] = '\0';
}
