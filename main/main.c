#include "config.h" // Inclui o arquivo de configuração
#include "wifi.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "bluetooth.h"
#include <math.h>
#include <stdarg.h>
#include <string.h>

#define TAG_Port_config ">>>> Configuração de Portas" // Tag para logs da função Port_config

static const char *TAG_WIFI = "wifi_task"; // Tag para logs da task Wi-Fi
static TaskHandle_t taskWifi = NULL;       // Handle da task Wi-Fi
static TaskHandle_t taskLogin = NULL;      // Handle da task de login


#define BLE_STARTUP_WINDOW_MS (60 * 1000U)

static bool ble_config_client_connected = false;

static tanque_limits_cache_t g_limits = {
    .min_pct = 0,
    .max_pct = 100,
    .last_ms = 0,
    .valid = false};

RTC_DATA_ATTR int g_last_restart_marker = 0;

static const char *TAG_BOMBA = "BOMBA"; // Tag para logs da task de bomba
static int g_bomba_controle_id[MAX_BOMBAS] = {0, 0, 0};

// Protótipo de Funções
esp_err_t Port_config(i2c_master_bus_handle_t *bus_handle,
                      i2c_master_dev_handle_t *mcp_handle,
                      i2c_master_dev_handle_t *ads_handle,
                      bool enable_mcp);       // Protótipo da função de configuração de portas

static void wifi_print_info(void);                                                         // Protótipo da função de impressão de informações Wi-Fi
void wifi_task(void *pv);                                                                  // Protótipo da task Wi-Fi
void login_task(void *pv);                                                                 // Protótipo da task de login
void ws_manager_task(void *pv);                                                            // Protótipo da task do gerenciador WebSocket
void ler_portas_task(void *pvParameters);                                                  // Protótipo da task de leitura das portas
static float clampf(float v, float lo, float hi);                                          // Protótipo da função clampf
static int pct_from_4_20ma(float ma);                                                      // Protótipo da função pct_from_4_20ma
static bool api_fetch_tanque_limits(int *out_min, int *out_max);                           // Protótipo da função de obtenção de limites do tanque via API
static void api_get_tanque_limits_cached(int *out_min, int *out_max);                      // Protótipo da função de obtenção de limites do tanque com cache
void nivel_task(void *pv);                                                                 // Protótipo da task de nível
static esp_err_t mcp_escrever_bomba_idx(int idx, bool ligar);                              // Protótipo da função de escrita da bomba para múltiplas bombas
void bombas_task(void *pv);                                                                // Protótipo da task de controle da bomba
static int bomba_index_from_id(uint16_t bomba_id);                                         // Protótipo da função para obter o índice da bomba a partir do ID da bomba
static float map_4_20_to_range(float ma, float out_min, float out_max);                    // Protótipo da função de mapeamento 4-20mA para faixa
static bool ler_corrente_bomba_idx(int idx, float *out_corrente_a);                        // Protótipo da função de leitura da corrente da bomba por índice
void corrente_task(void *pv);                                                              // Protótipo da task de corrente
void ConnectRest();                                                                        // Protótipo de função reset do esp
void watchdog_callback(void *arg);                                                         // Protótipo da função de callback do watchdog
void start_watchdog_timer();                                                               // Protótipo da função de início do timer do watchdog
void reset_watchdog_timer();                                                               // Protótipo da função de reset do timer do watchdog
static void lora_setup_tanque(void);                                                       // Protótipo da função de configuração do LoRa para o tanque
bool tanque_send_to_gtw_ack(uint16_t src_tank_id,
                            uint16_t gtw_id,
                            const char *msg,
                            uint8_t has_bomba,
                            uint16_t bomba_id); // Protótipo da função de envio de mensagem para a gateway com ACK

static bool lora_enqueue_json_tanque_id_int(uint16_t tanque_id, const char *campo, int valor);
void tanque_lora_tx_task(void *pv);                                         // Protótipo da task de transmissão LoRa do tanque
uint16_t lora_crc16(const uint8_t *data, size_t len);
static bool ler_nivel_atual_pct(int *out_pct);
void tanque_lora_rx_task(void *pv);                                         // Protótipo da task de recepção LoRa do tanque
bool tratar_comando_bomba(const char *cmd, int valor, uint16_t bomba_id, int controle_id);   // Protótipo da função de tratamento de comando da bomba recebido via LoRa
static void lora_start_if_needed(void);                                     // Protótipo da função de início do LoRa se necessário

void tanque_lora_start(void);
static void tanque_lora_app_task(void *pv);
static void tanque_process_cmd_from_gtw(const lora_app_frame_t *rx);
bool tanque_send_to_gtw_noack(uint16_t src_tank_id, uint16_t gtw_id, const char *msg, uint8_t has_bomba, uint16_t bomba_id);
static void tanque_send_ack(const lora_app_frame_t *rx);
static bool lora_is_duplicate(const lora_app_frame_t *rx);
static bool lora_wait_ack_notification(uint32_t timeout_ms);
static bool lora_ack_wait_match_and_signal(const lora_app_frame_t *rx);
static void lora_ack_wait_cancel(void);
static void lora_ack_wait_begin(uint8_t msg_id, uint8_t expected_src_type, uint16_t expected_src_id);
static void lora_boot_snapshot_task(void *pv);


static bool existe_emergencia_ativa_global(void);
static bool lora_enqueue_json_bomba(uint16_t bomba_id, const char *json);
static bool lora_enqueue_json_bomba_int(uint16_t bomba_id, const char *campo, int valor);
static bool lora_enqueue_alerta_codigo(uint8_t codigo, int unidade_id, int bomba_id, int pct);
static bool lora_enqueue_alerta_codigo_controle(uint8_t codigo, int unidade_id, int bomba_id, int controle_id, int pct);
static bool lora_enqueue_bomba_controle_false(int controle_id);
static void log_ble_config_missing(bool wifi_ok, bool cfg_ok);

void vazao_sync_task(void *pv);
static bool existe_bomba_ligada_ou_desejada(void);
void pwm_agendar_sync_vazao(int bomba_id, float percent);

uint16_t tanque_lora_get_device_id(void) {
    return (g_cfg.tanque_id > 0) ? (uint16_t)g_cfg.tanque_id : (uint16_t)PROVISION_TANQUE_ID;
}

// ====================================================================================

void save_status_bomba(int idx, bool state)
{
    nvs_handle_t my_handle;
    char key[16];

    snprintf(key, sizeof(key), "bomba_%d", idx); // bomba_0, bomba_1, bomba_2

    if (nvs_open("bombas", NVS_READWRITE, &my_handle) == ESP_OK)
    {
        nvs_set_i32(my_handle, key, state ? 1 : 0);
        nvs_commit(my_handle);
        nvs_close(my_handle);
    }
}

bool load_status_bomba(int idx)
{
    nvs_handle_t my_handle;
    char key[16];
    int32_t state = 0; // padrão desligada

    snprintf(key, sizeof(key), "bomba_%d", idx);

    if (nvs_open("bombas", NVS_READONLY, &my_handle) == ESP_OK)
    {
        nvs_get_i32(my_handle, key, &state);
        nvs_close(my_handle);
    }

    return state == 1;
}

// ====================================================================================

const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r)
    {
    case ESP_RST_UNKNOWN:
        return "Motivo desconhecido";

    case ESP_RST_POWERON:
        return "Ligou ao receber energia";

    case ESP_RST_EXT:
        return "Reset pelo botão (pino EN/RST)";

    case ESP_RST_SW:
        return "Reiniciado pelo sistema";

    case ESP_RST_PANIC:
        return "Erro grave no sistema";

    case ESP_RST_INT_WDT:
        return "Travou (watchdog de interrupção)";

    case ESP_RST_TASK_WDT:
        return "Travou (watchdog de tarefa)";

    case ESP_RST_WDT:
        return "Travou (watchdog geral)";

    case ESP_RST_DEEPSLEEP:
        return "Acordou do modo economia";

    case ESP_RST_BROWNOUT:
        return "Queda de energia(BROWNOUT)";

    case ESP_RST_SDIO:
        return "Reset por comunicação (SDIO)";

    default:
        return "Motivo desconhecido";
    }
}

#define RELAY_ID 25

static bool is_relay_target(uint16_t id) {
    return (id == 28);
}

static bool should_relay_frame(const lora_app_frame_t *rx, uint16_t my_id) {
    if (my_id != RELAY_ID) {
        return false;
    }

    // ida: GTW -> tanque alvo
    if (rx->dst_type == DEV_TANK && is_relay_target(rx->dst_id)) {
        return true;
    }

    // volta: tanque alvo -> GTW
    if (rx->src_type == DEV_TANK &&
        is_relay_target(rx->src_id) &&
        rx->dst_type == DEV_GTW) {
        return true;
    }

    return false;
}

static bool text_has_value(const char *value)
{
    return value && value[0] != '\0';
}

static void log_ble_config_missing(bool wifi_ok, bool cfg_ok)
{
    int missing = 0;

    ESP_LOGW("BLE_CFG", "========== CONFIGURACAO NECESSARIA VIA BLE ==========");

    if (!wifi_ok || !text_has_value(wifi_ssid))
    {
        ESP_LOGW("BLE_CFG", "FALTA: wifi_ssid");
        missing++;
    }
    else
    {
        ESP_LOGI("BLE_CFG", "OK: wifi_ssid=%s", wifi_ssid);
    }

    if (!text_has_value(wifi_password))
    {
        ESP_LOGW("BLE_CFG", "AVISO: wifi_password vazia; use assim apenas se a rede for aberta");
    }
    else
    {
        ESP_LOGI("BLE_CFG", "OK: wifi_password salva (%u caracteres)", (unsigned)strlen(wifi_password));
    }

    if (!cfg_ok)
    {
        ESP_LOGW("BLE_CFG", "CFG principal ainda nao esta completa na NVS");
    }

    if (!text_has_value(g_cfg.api_user))
    {
        ESP_LOGW("BLE_CFG", "FALTA: API_USER");
        missing++;
    }
    else
    {
        ESP_LOGI("BLE_CFG", "OK: API_USER=%s", g_cfg.api_user);
    }

    if (!text_has_value(g_cfg.api_pass))
    {
        ESP_LOGW("BLE_CFG", "FALTA: API_PASS");
        missing++;
    }
    else
    {
        ESP_LOGI("BLE_CFG", "OK: API_PASS salva (%u caracteres)", (unsigned)strlen(g_cfg.api_pass));
    }

    if (g_cfg.tanque_id <= 0)
    {
        ESP_LOGW("BLE_CFG", "FALTA: PROVISION_TANQUE_ID / tanque_id");
        missing++;
    }
    else
    {
        ESP_LOGI("BLE_CFG", "OK: tanque_id=%d", g_cfg.tanque_id);
    }

    if (g_cfg.lora_gtw_id <= 0)
    {
        ESP_LOGW("BLE_CFG", "FALTA: LORA_GTW_ID");
        missing++;
    }
    else
    {
        ESP_LOGI("BLE_CFG", "OK: LORA_GTW_ID=%d", g_cfg.lora_gtw_id);
    }

    if (text_has_value(g_cfg.nome_tanque))
    {
        ESP_LOGI("BLE_CFG", "OK: nome_tanque=%s", g_cfg.nome_tanque);
    }
    else
    {
        ESP_LOGW("BLE_CFG", "AVISO: nome_tanque vazio; nao bloqueia o boot, mas melhora os alertas");
    }

    if (missing == 0)
    {
        ESP_LOGI("BLE_CFG", "Configuracao minima completa para sair do modo BLE");
    }
    else
    {
        ESP_LOGW("BLE_CFG", "Total de campos obrigatorios faltando: %d", missing);
    }

    ESP_LOGW("BLE_CFG", "======================================================");
}

// ===============================================================================================

void app_main(void)
{
    ESP_LOGW("RESET", "Ultimo reset reason = %d", esp_reset_reason());

    ESP_ERROR_CHECK(cfg_nvs_init());

    bool tem_wifi = cfg_wifi_load();
    if (!tem_wifi)
    {
        ESP_LOGW("MAIN", "Sem WiFi salvo na NVS");
    }

    bool tem_cfg = cfg_load(&g_cfg);
    bool device_configured = tem_wifi && tem_cfg;

    if (tem_cfg)
    {
        ESP_LOGI("MAIN", ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> CFG carregada da NVS");
        ESP_LOGW("MAIN", "Boot com CFG: perfil=%s", (g_cfg.perfil == TANQUE_FULL ? "FULL" : "NIVEL"));
        cfg_log(&g_cfg);
    }
    else
    {
        ESP_LOGW(">>>>>>>>>>>>>>>>>>>>>>>>>>> MAIN", "Sem CFG valida na NVS");
        // aqui NÃO pode decidir FULL/NIVEL ainda
    }

    ESP_LOGW("MAIN",
             "CFG ATUAL: prov=%d perfil=%d unidade=%d tanque_id=%d qtd=%d b0=%d b1=%d b2=%d nome=%s",
             (int)g_cfg.provisionado, (int)g_cfg.perfil,
             g_cfg.unidade_id, g_cfg.tanque_id,
             g_cfg.qtd_bombas,
             g_cfg.bomba_id[0], g_cfg.bomba_id[1], g_cfg.bomba_id[2],
             g_cfg.nome_tanque);

    ESP_LOGI(">>>> Main", "Iniciando sistema...");
    heap_total = esp_get_free_heap_size(); // Obtém o tamanho total do heap livre
    ESP_LOGW(">>>> Main", "Tamanho total do heap livre: %zu bytes", heap_total);

    // Cria o semáforo para o barramento I2C
    // Verifica se o semáforo foi criado com sucesso
    i2c_semaphore = xSemaphoreCreateMutex();
    if (i2c_semaphore == NULL)
    {
        ESP_LOGE(">>>> Main", " Falha ao criar semáforo do barramento I2C");
    }
    else
    {
        ESP_LOGI(">>>> Main", " Semáforo do barramento I2C criado com sucesso");
    }

    // Inicializa a fila e o mutex
    // Verifica se o mutex foi criado com sucesso
    MutexHTTP = xSemaphoreCreateMutex();
    if (MutexHTTP == NULL)
    {
        ESP_LOGE("Main", "Falha ao criar mutex para requisições HTTP");
    }
    else
    {
        ESP_LOGI("Main", "Mutex para requisições HTTP criado com sucesso");
    }

    MutexLora = xSemaphoreCreateMutex();

    if (MutexLora == NULL)
    {
        ESP_LOGE("Main", "Falha ao criar mutex Lora");
    }
    else
    {
        ESP_LOGI("Main", "Mutex para o Lora criado com sucesso");
    }

    ESP_LOGI(">>>> Main", " Barramento I2C e dispositivos configurados com sucesso");

    // Cria o event group do sistema, se ainda não foi criado
    if (!sys_event_group)
    {
        sys_event_group = xEventGroupCreate();
        if (!sys_event_group)
        {
            ESP_LOGE(">>>> Main", " Falha ao criar event group do sistema");
            return;
        }
    }

    // Garante estado inicial offline no boot
    xEventGroupClearBits(sys_event_group,
                         SYS_NET_ONLINE_BIT |
                             SYS_AUTH_OK_BIT |
                             SYS_WS_OK_BIT);

    if (!device_configured)
    {
        ESP_LOGW("MAIN", "Dispositivo sem configuracao completa; abrindo BLE sem timeout");
        log_ble_config_missing(tem_wifi, tem_cfg);
        esp_err_t ble_err = bluetooth_config_start(BLUETOOTH_CONFIG_TIMEOUT_FOREVER_MS);
        if (ble_err != ESP_OK)
        {
            ESP_LOGW("MAIN", "Falha ao iniciar BLE de configuracao: %s", esp_err_to_name(ble_err));
        }

        start_watchdog_timer();
        while (1)
        {
            reset_watchdog_timer();
            rtc_wdt_feed_raw();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    ESP_LOGW("MAIN", "Boot: subindo LoRa ANTES do Wi-Fi para garantir redundancia");
    lora_start_if_needed();

    ESP_LOGI("MAIN", "Abrindo BLE por 1 minuto; Wi-Fi inicia quando a janela BLE fechar");
    esp_err_t ble_err = bluetooth_config_start(BLE_STARTUP_WINDOW_MS);
    if (ble_err != ESP_OK)
    {
        ESP_LOGW("MAIN", "Falha ao iniciar BLE de configuracao: %s", esp_err_to_name(ble_err));
    }

    ESP_LOGI(">>>> Main", "Criando task do Wi-Fi...");
    xTaskCreate(wifi_task, "wifi_task", 6096 * 2, NULL, 7, &taskWifi);
    vTaskDelay(pdMS_TO_TICKS(200));

    ESP_LOGI(">>>> Main", "Criando task de Login...");
    xTaskCreate(login_task, "login_task", 8192, NULL, 5, &taskLogin);

    ESP_LOGI(">>>> Main", "Criando task de websocket...");
    xTaskCreate(ws_manager_task, "ws_manager", 4096, NULL, 5, NULL);

    bool enable_mcp = (g_cfg.perfil == TANQUE_FULL);

    esp_err_t ret = Port_config(&bus_handle, &mcp_handle, &ads_handle, enable_mcp);
    if (ret != ESP_OK)
    {
        ESP_LOGE("MAIN", "Falha Port_config. Sistema continua com LoRa ativo.");
    }
    else
    {
        if (g_cfg.perfil == TANQUE_FULL)
        {
            xTaskCreate(ler_portas_task, "ler_portas_task", 4096, NULL, 5, NULL);
            xTaskCreate(bombas_task, "bombas_task", 4096, NULL, 5, NULL);
            xTaskCreate(corrente_task, "corrente_task", 4096, NULL, 5, NULL);
            xTaskCreate(vazao_sync_task, "vazao_sync_task", 4096, NULL, 5, NULL);
        }

        xTaskCreatePinnedToCore(nivel_task, "nivel_task", 8192, NULL, 5, NULL, 1);
        xTaskCreate(lora_boot_snapshot_task, "lora_boot_snapshot", 4096, NULL, 4, NULL);
    }

    start_watchdog_timer(); // Inicia o watchdog de software

    while (1)
    {
#if DEBUG_MODE
        ESP_LOGI("MAIN", "Executando normalmente...");
#endif
        reset_watchdog_timer(); // Reseta o timer para evitar reset
        rtc_wdt_feed_raw();     // watchdog de hardware
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Função para configurar o barramento I2C e os dispositivos MCP23017 e ADS1115
esp_err_t Port_config(i2c_master_bus_handle_t *bus_handle,
                      i2c_master_dev_handle_t *mcp_handle,
                      i2c_master_dev_handle_t *ads_handle,
                      bool enable_mcp)
{

    ESP_LOGW(TAG_Port_config, "Port_config(): perfil atual=%d (0=NIVEL,1=FULL)", (int)g_cfg.perfil);

    if (bus_handle == NULL || ads_handle == NULL)
    {
        ESP_LOGE(TAG_Port_config, "Bus ou ADS handle nulo");
        return ESP_ERR_INVALID_ARG;
    }

    if (enable_mcp && mcp_handle == NULL)
    {
        ESP_LOGE(TAG_Port_config, "MCP handle nulo (perfil FULL)");
        return ESP_ERR_INVALID_ARG;
    }

    printf("\033[1;36m\n\n========== INICIANDO CONFIGURAÇÃO DE PORTAS ==========\n\033[0m");

    bool i2c_ok = false;      // indica se o barramento I2C foi configurado com sucesso
    bool mcp_add_ok = false;  // indica se o dispositivo MCP23017 foi adicionado com sucesso
    bool mcp_init_ok = false; // indica se o dispositivo MCP23017 foi inicializado com sucesso
    bool ads_dev_ok = false;  // indica se o dispositivo ADS1115 foi adicionado com sucesso

    int retry_count = 0;      // contador de tentativas
    esp_err_t ret = ESP_FAIL; // código de retorno da função

    // ETAPA 1: Configura barramento I2C
    ESP_LOGI(TAG_Port_config, " [ETAPA 1/4] Configurando barramento I2C...");

    i2c_master_bus_config_t I2c_Config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    // Verificações básicas
    if (bus_handle == NULL)
    {
        ESP_LOGE(TAG_Port_config, " Handle do barramento I2C é nulo.");
        return ESP_ERR_INVALID_ARG;
    }
    else if (I2c_Config.i2c_port < 0 || I2c_Config.i2c_port >= I2C_NUM_MAX)
    {
        ESP_LOGE(TAG_Port_config, " Porta I2C inválida: %d", I2c_Config.i2c_port);
        return ESP_ERR_INVALID_ARG;
    }
    else if (I2c_Config.sda_io_num < 0 || I2c_Config.scl_io_num < 0)
    {
        ESP_LOGE(TAG_Port_config, " Pinos SDA ou SCL não configurados corretamente.");
        return ESP_ERR_INVALID_ARG;
    }
    else if (I2c_Config.glitch_ignore_cnt > 15)
    {
        ESP_LOGE(TAG_Port_config, " Contador de glitch inválido: %d. Deve ser entre 0 e 15.", I2c_Config.glitch_ignore_cnt);
        return ESP_ERR_INVALID_ARG;
    }
    else
    {
        ESP_LOGW(TAG_Port_config,
                 " Configuração do barramento I2C: Porta %d, SDA %d, SCL %d, Glitch Ignore Count %d",
                 I2c_Config.i2c_port, I2c_Config.sda_io_num,
                 I2c_Config.scl_io_num, I2c_Config.glitch_ignore_cnt);
        ESP_LOGW(TAG_Port_config, " Fonte de clock: %d", I2c_Config.clk_source);
    }

    retry_count = 0;
    while (retry_count < MAX_RETRIES_CONFIG)
    {
        ret = i2c_new_master_bus(&I2c_Config, bus_handle);

        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG_Port_config,
                     " >>> I2C master configurado com sucesso (tentativa %d/%d)",
                     retry_count + 1, MAX_RETRIES_CONFIG);
            i2c_ok = true;
            break;
        }

        ESP_LOGE(TAG_Port_config,
                 " >>> Falha na configuração do I2C (tentativa %d/%d): %s",
                 retry_count + 1, MAX_RETRIES_CONFIG, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(2000));
        retry_count++;
    }

    if (!i2c_ok)
    {
        ESP_LOGE(TAG_Port_config,
                 " >>> ERRO FATAL: Não foi possível configurar o barramento I2C após %d tentativas.",
                 MAX_RETRIES_CONFIG);
        goto resumo; // encerra a função com erro
    }

    if (enable_mcp)
    {

        // ETAPA 2: Adiciona dispositivo MCP23017
        ESP_LOGI(TAG_Port_config, " [ETAPA 2/4] Adicionando dispositivo MCP23017 (0x21)...");

        i2c_device_config_t mcp_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = 0x21,
            .scl_speed_hz = I2C_MASTER_FREQ_HZ,
        };

        if (mcp_handle == NULL)
        {
            ESP_LOGE(TAG_Port_config, " Handle do dispositivo MCP é nulo.");
            goto resumo;
        }
        if (mcp_cfg.device_address == I2C_DEVICE_ADDRESS_NOT_USED)
        {
            ESP_LOGE(TAG_Port_config, " Endereço do dispositivo MCP não configurado.");
            goto resumo;
        }

        retry_count = 0;
        while (retry_count < MAX_RETRIES_CONFIG)
        {
            ret = i2c_master_bus_add_device(*bus_handle, &mcp_cfg, mcp_handle);
            printf(">>>> RETORNO MCP: %d\n", ret);

            if (ret == ESP_OK)
            {
                mcp_add_ok = true;
                ESP_LOGI(TAG_Port_config,
                         " >>> MCP23017 adicionado com sucesso ao barramento (tentativa %d/%d)",
                         retry_count + 1, MAX_RETRIES_CONFIG);
                break;
            }

            ESP_LOGE(TAG_Port_config,
                     " >>> Falha ao adicionar MCP23017 (tentativa %d/%d): %s",
                     retry_count + 1, MAX_RETRIES_CONFIG, esp_err_to_name(ret));

            vTaskDelay(pdMS_TO_TICKS(2000));
            retry_count++;
        }

        if (!mcp_add_ok)
        {
            ESP_LOGE(TAG_Port_config,
                     " >>> ERRO: MCP23017 não pôde ser adicionado após %d tentativas.",
                     MAX_RETRIES_CONFIG);
            goto resumo;
        }

        // ETAPA 3: Inicializa MCP23017
        ESP_LOGI(TAG_Port_config, "[ETAPA 3/4] Inicializando MCP23017...");

        retry_count = 0;
        while (retry_count < MAX_RETRIES_CONFIG)
        {
            int mcp_status = InitMcp(*mcp_handle);

            printf(">>>> STATUS MCP: %d\n", mcp_status);

            if (mcp_status == 0)
            {
                mcp_init_ok = true;
                ESP_LOGI(TAG_Port_config,
                         " >>> MCP23017 inicializado com sucesso (tentativa %d/%d)",
                         retry_count + 1, MAX_RETRIES_CONFIG);
                break;
            }

            vTaskDelay(pdMS_TO_TICKS(2000));
            retry_count++;
        }

        if (!mcp_init_ok)
        {
            ESP_LOGE(TAG_Port_config, " ERRO: MCP23017 NÃO foi inicializado corretamente após %d tentativas.", MAX_RETRIES_CONFIG);
            goto resumo;
        }
    }
    else
    {
        ESP_LOGW(TAG_Port_config, "MCP ignorado (enable_mcp=false)");
        if (mcp_handle)
        {
            *mcp_handle = NULL;
        }
    }

    // ETAPA 4: Adiciona / Configura ADS1115
    ESP_LOGI(TAG_Port_config, "[ETAPA 4/4] Adicionando e configurando ADS1115 (0x49)...");

    i2c_device_config_t ads_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x49,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    retry_count = 0;
    while (retry_count < MAX_RETRIES_CONFIG)
    {
        ret = i2c_master_bus_add_device(*bus_handle, &ads_cfg, ads_handle);

        if (ret == ESP_OK)
        {
            ads_dev_ok = true;
            ESP_LOGI(TAG_Port_config, " ADS1115 adicionado com sucesso (tentativa %d/%d)", retry_count + 1, MAX_RETRIES_CONFIG);
            break;
        }

        ESP_LOGE(TAG_Port_config, "Falha ao adicionar ADS1115 (tentativa %d/%d): %s", retry_count + 1, MAX_RETRIES_CONFIG, esp_err_to_name(ret));

        vTaskDelay(pdMS_TO_TICKS(2000));
        retry_count++;
    }

    if (!ads_dev_ok)
    {
        ESP_LOGE(TAG_Port_config, " ERRO: ADS1115 não pôde ser adicionado após %d tentativas.", MAX_RETRIES_CONFIG);
        goto resumo;
    }

    // Configura o ADS1115 (só chega aqui se ads_dev_ok == true)
    _4a20ma = ads1115_config(*bus_handle, 0x49, *ads_handle);
    ads1115_set_max_ticks(&_4a20ma, 100);
    ads1115_set_pga(&_4a20ma, ADS1115_FSR_2_048);
    ads1115_set_sps(&_4a20ma, ADS1115_SPS_128);

    ret = ESP_OK; // se chegou aqui, tudo passou

resumo:
    printf("\n\033[1;33m---------- RESUMO CONFIGURAÇÃO I2C / PORTAS ----------\033[0m\n");

    ESP_LOGI(TAG_Port_config, " Barramento I2C      : %s", i2c_ok ? "OK" : "FALHOU");
    if (enable_mcp)
    {
        ESP_LOGI(TAG_Port_config, " MCP23017 (device)   : %s", mcp_add_ok ? "OK" : "FALHOU");
        ESP_LOGI(TAG_Port_config, " MCP23017 (init)     : %s", mcp_init_ok ? "OK" : "FALHOU");
    }
    else
    {
        ESP_LOGI(TAG_Port_config, " MCP23017 (device)   : IGNORADO (perfil NIVEL)");
        ESP_LOGI(TAG_Port_config, " MCP23017 (init)     : IGNORADO (perfil NIVEL)");
    }
    ESP_LOGI(TAG_Port_config, " ADS1115 (device)    : %s", ads_dev_ok ? "OK" : "FALHOU");

    bool sucesso_total = i2c_ok && ads_dev_ok && (!enable_mcp || (mcp_add_ok && mcp_init_ok));
    ret = sucesso_total ? ESP_OK : ESP_FAIL;

    if (sucesso_total)
    {
        printf("\n\033[1;32m========== PORTAS CONFIGURADAS COM SUCESSO ==========\033[0m\n\n");
        ret = ESP_OK;
    }
    else
    {
        printf("\n\033[1;31m========== FALHA NA CONFIGURAÇÃO DE PORTAS, VER LOG ACIMA ==========\033[0m\n\n");
        ret = ESP_FAIL;
    }

    ledc_timer_config_t ledc_timer = {.speed_mode = PWM_MODE,
                                      .timer_num = PWM_TIMER,
                                      .duty_resolution = PWM_RES,
                                      .freq_hz = PWM_FREQ_HZ,
                                      .clk_cfg = LEDC_AUTO_CLK};

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {.speed_mode = PWM_MODE,
                                          .channel = PWM_CHANNEL,
                                          .timer_sel = PWM_TIMER,
                                          .intr_type = LEDC_INTR_DISABLE,
                                          .gpio_num = PWM_GPIO,
                                          .duty = 0,
                                          .hpoint = 0};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI("PWM", "PWM 4-20mA inicializado");
    pwm_processar_novo_setpoint(0.0f);

    return ret;
}

// Função para imprimir informações da conexão Wi-Fi
static void wifi_print_info(void)
{
    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
    {
        ESP_LOGI(TAG_WIFI, "SSID: %s | RSSI: %d dBm | CH: %d", (char *)ap.ssid, ap.rssi, ap.primary);
    }

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif)
    {
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(netif, &ip) == ESP_OK)
        {
            ESP_LOGI(TAG_WIFI, "IP: " IPSTR " GW: " IPSTR, IP2STR(&ip.ip), IP2STR(&ip.gw));
        }
    }
}

void bt_client_connected_callback(void)
{
    ble_config_client_connected = true;
    xEventGroupClearBits(sys_event_group, SYS_NET_ONLINE_BIT | SYS_AUTH_OK_BIT | SYS_WS_OK_BIT);

    ESP_LOGW("TANK_BLE", "Cliente BLE conectado; parando WebSocket e Wi-Fi para modo configuracao");
    ws_client_stop();

    if (wifi_is_active())
    {
        esp_err_t err = wifi_stop_driver();
        if (err != ESP_OK)
        {
            ESP_LOGW("TANK_BLE", "Falha ao parar Wi-Fi no modo BLE: %s", esp_err_to_name(err));
        }
    }
}

void bt_client_disconnected_callback(void)
{
    ble_config_client_connected = false;
    ESP_LOGI("TANK_BLE", "Cliente BLE desconectado; Wi-Fi sera retomado pela task");
}

// Task para gerenciar a conexão Wi-Fi
void wifi_task(void *pv)
{
    esp_err_t wdt_err = esp_task_wdt_add(NULL);

    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG_WIFI, "Falha ao adicionar wifi_task no WDT: %s", esp_err_to_name(wdt_err));
    }

    ESP_LOGI(TAG_WIFI, "Task Wi-Fi iniciou");

    bool was_online = false;
    bool ws_stopped = false;
    bool first_online_done = false;

    int fails_internet = 0;

    int64_t last_wifi_try_ms = 0;
    int64_t last_net_check_ms = 0;
    int64_t last_driver_retry_ms = 0;
    int64_t wifi_disabled_until_ms = 0;

    bool wifi_driver_ok = false;
    int wifi_connect_fail_count = 0;

    while (1)
    {
        esp_task_wdt_reset();

        int64_t now_ms = esp_timer_get_time() / 1000;

        if (wifi_disabled_until_ms > now_ms)
        {
            xEventGroupClearBits(sys_event_group, SYS_NET_ONLINE_BIT | SYS_AUTH_OK_BIT | SYS_WS_OK_BIT);

            if (wifi_driver_ok && wifi_is_active())
            {
                ESP_LOGW(TAG_WIFI, "Wi-Fi em pausa; desligando driver ate proxima tentativa");
                wifi_stop_driver();
                wifi_driver_ok = false;
            }

            if (!ws_stopped)
            {
                ws_client_stop();
                ws_stopped = true;
            }

            vTaskDelay(pdMS_TO_TICKS(WIFI_TASK_TICK_MS));
            continue;
        }

        if (bluetooth_config_is_active() || ble_config_client_connected)
        {
            xEventGroupClearBits(sys_event_group, SYS_NET_ONLINE_BIT | SYS_AUTH_OK_BIT | SYS_WS_OK_BIT);

            if (wifi_driver_ok && wifi_is_active())
            {
                ESP_LOGW(TAG_WIFI, "BLE ativo; desligando Wi-Fi durante configuracao");
                wifi_stop_driver();
                wifi_driver_ok = false;
                was_online = false;
                ws_stopped = true;
            }

            vTaskDelay(pdMS_TO_TICKS(WIFI_TASK_TICK_MS));
            continue;
        }

        /*
         * Não usar ESP_ERROR_CHECK aqui.
         * Se o driver falhar, apenas tenta novamente depois.
         */
        if (!wifi_driver_ok)
        {
            if ((now_ms - last_driver_retry_ms) >= 5000)
            {
                last_driver_retry_ms = now_ms;

                esp_err_t start_err = wifi_start_driver();

                if (start_err == ESP_OK)
                {
                    wifi_driver_ok = true;
                    ESP_LOGI(TAG_WIFI, "Wi-Fi driver iniciado");
                }
                else
                {
                    ESP_LOGE(TAG_WIFI,
                             "Falha ao iniciar Wi-Fi driver: %s",
                             esp_err_to_name(start_err));
                }
            }

            xEventGroupClearBits(sys_event_group, SYS_NET_ONLINE_BIT);

            vTaskDelay(pdMS_TO_TICKS(WIFI_TASK_TICK_MS));
            continue;
        }

        bool sta_ok = wifi_sta_connected();
        bool ip_ok = wifi_has_ip();

        // =========================================================
        // 1) SEM AP ou SEM IP
        // =========================================================
        if (!sta_ok || !ip_ok)
        {
            xEventGroupClearBits(sys_event_group, SYS_NET_ONLINE_BIT);

            if (!ws_stopped)
            {
                esp_task_wdt_reset();

                ESP_LOGW(TAG_WIFI, "Parando WebSocket porque Wi-Fi está sem AP/IP");

                ws_client_stop();

                esp_task_wdt_reset();

                ws_stopped = true;
            }

            if (was_online)
            {
                was_online = false;
                ESP_LOGW(TAG_WIFI, "OFFLINE (sem AP ou sem IP) -> LoRa assume");
            }

            int retry_ms = first_online_done ? WIFI_RUNTIME_RETRY_INTERVAL_MS
                                             : WIFI_BOOT_RETRY_INTERVAL_MS;

            if ((now_ms - last_wifi_try_ms) >= retry_ms)
            {
                last_wifi_try_ms = now_ms;

                    ESP_LOGW(TAG_WIFI, "Tentando conectar Wi-Fi em '%s'...", wifi_ssid);

                    esp_task_wdt_reset();

                    esp_err_t err = wifi_connect_credentials(
                        wifi_ssid,
                        wifi_password,
                        WIFI_CONNECT_TIMEOUT_MS);

                    esp_task_wdt_reset();

                    if (err == ESP_OK)
                    {
                        wifi_connect_fail_count = 0;
                        ESP_LOGI(TAG_WIFI, "Wi-Fi conectado ao AP");
                        wifi_print_info();
                    }
                    else
                    {
                        wifi_connect_fail_count++;
                        ESP_LOGW(TAG_WIFI,
                                 "Falha ao conectar Wi-Fi (%d/%d): %s",
                                 wifi_connect_fail_count,
                                 WIFI_CONNECT_ATTEMPTS_BEFORE_SLEEP,
                                 esp_err_to_name(err));

                        if (wifi_connect_fail_count >= WIFI_CONNECT_ATTEMPTS_BEFORE_SLEEP)
                        {
                            wifi_disabled_until_ms = now_ms + (int64_t)WIFI_DISABLED_RETRY_MS;
                            wifi_connect_fail_count = 0;

                            ESP_LOGW(TAG_WIFI,
                                     "Wi-Fi pausado por %u minutos apos falhas de conexao",
                                     (unsigned)(WIFI_DISABLED_RETRY_MS / 60000ULL));

                            wifi_stop_driver();
                            wifi_driver_ok = false;
                        }
                    }
                
            }

            vTaskDelay(pdMS_TO_TICKS(WIFI_TASK_TICK_MS));
            continue;
        }

        // =========================================================
        // 2) TEM AP + IP -> checa internet/backend
        // =========================================================
        int check_interval_ms = first_online_done ? INTERNET_CHECK_INTERVAL_MS
                                                  : INTERNET_BOOT_CHECK_INTERVAL_MS;

        if ((now_ms - last_net_check_ms) >= check_interval_ms)
        {
            last_net_check_ms = now_ms;

            esp_task_wdt_reset();

            bool internet_ok = wifi_check_internet_tcp(
                INTERNET_HEALTH_URL,
                INTERNET_TIMEOUT_MS);

            esp_task_wdt_reset();

            if (internet_ok)
            {
                fails_internet = 0;
                wifi_connect_fail_count = 0;

                xEventGroupSetBits(sys_event_group, SYS_NET_ONLINE_BIT);

                if (!was_online)
                {
                    was_online = true;
                    ws_stopped = false;
                    first_online_done = true;

                    ESP_LOGI(TAG_WIFI, "ONLINE (AP + IP + reachability OK)");
                    wifi_print_info();
                }
            }
            else
            {
                fails_internet++;

                ESP_LOGW(TAG_WIFI,
                         "Sem internet/backend (%d/%d) -> LoRa assume",
                         fails_internet,
                         INTERNET_FAILS_TO_OFFLINE);

                if (fails_internet >= INTERNET_FAILS_TO_OFFLINE)
                {
                    xEventGroupClearBits(sys_event_group, SYS_NET_ONLINE_BIT);

                    if (!ws_stopped)
                    {
                        esp_task_wdt_reset();

                        ESP_LOGW(TAG_WIFI, "Parando WebSocket por falha de internet/backend");

                        ws_client_stop();

                        esp_task_wdt_reset();

                        ws_stopped = true;
                    }

                    if (was_online)
                    {
                        was_online = false;
                        ESP_LOGW(TAG_WIFI, "OFFLINE lógico (backend/internet) -> mantendo Wi-Fi associado");
                    }

                    /*
                     * Evita contador crescer infinito.
                     */
                    fails_internet = INTERNET_FAILS_TO_OFFLINE;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(WIFI_TASK_TICK_MS));
    }
}

// Task para gerenciar o login na API
void login_task(void *pv)
{
    static const char *TAG = "login_task";

    esp_task_wdt_add(NULL);

    int fail_count = 0;
    int initRST = 0;
    int64_t last_login_try_ms = 0;

    const int LOGIN_RETRY_MIN_MS = 5000;  // 5 s
    const int LOGIN_RETRY_MAX_MS = 60000; // 60 s
    const int LOOP_DELAY_MS = 1000;       // task roda a cada 1 s

    while (1)
    {
        esp_task_wdt_reset();

        EventBits_t bits = xEventGroupGetBits(sys_event_group);
        bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;

        // -------------------------------------------------
        // Sem internet -> não tenta login e limpa AUTH_OK
        // -------------------------------------------------
        if (!net_ok || !wifi_has_ip())
        {
            xEventGroupClearBits(sys_event_group, SYS_AUTH_OK_BIT);

            fail_count = 0; // zera backoff quando rede cai
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            continue;
        }

        // -------------------------------------------------
        // Já tem token -> mantém AUTH_OK
        // -------------------------------------------------
        if (api_has_token())
        {
            xEventGroupSetBits(sys_event_group, SYS_AUTH_OK_BIT);
            fail_count = 0;
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        // -------------------------------------------------
        // Sem token -> tenta login com intervalo controlado
        // -------------------------------------------------
        int retry_ms = LOGIN_RETRY_MIN_MS;

        if (fail_count > 0)
        {
            retry_ms = LOGIN_RETRY_MIN_MS << fail_count; // 5,10,20,40...
            if (retry_ms > LOGIN_RETRY_MAX_MS)
                retry_ms = LOGIN_RETRY_MAX_MS;
        }

        int64_t now_ms = esp_timer_get_time() / 1000;

        if ((now_ms - last_login_try_ms) < retry_ms)
        {
            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
            continue;
        }

        last_login_try_ms = now_ms;

        ESP_LOGI(TAG, "Tentando login na API...");

        esp_err_t err = ESP_ERR_TIMEOUT;
        if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(15000)) == pdTRUE)
        {
            err = api_login(API_USER, API_PASS);
            xSemaphoreGive(MutexHTTP);
        }
        else
        {
            ESP_LOGW(TAG, "Login aguardando HTTP livre; MutexHTTP ocupado");
        }

        if (err == ESP_OK && api_has_token())
        {
            ESP_LOGI(TAG, "Login OK");
            if (initRST == 0)
            {
                initRST = 1;
                esp_reset_reason_t reset_reason = esp_reset_reason();
                if (reset_reason == ESP_RST_PANIC)
                {
                    if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                    {
                        char msg[160];
                        snprintf(msg, sizeof(msg), "Motivo do ultimo reset tanque %s : %s", NOME_CURTO,
                                 reset_reason_str(reset_reason));
                        esp_err_t post_err = api_post_tanque(msg, g_cfg.unidade_id);

                        if (post_err != ESP_OK)
                        {
                            post_err = api_post_tanque(msg, g_cfg.unidade_id);
                        }

                        xSemaphoreGive(MutexHTTP);
                        if (post_err == ESP_OK)
                        {
                            printf("....................>>FOI\n");
                        }
                    }
                }
                else
                {
                    ESP_LOGI("RESET", "Reset nao enviado: %s", reset_reason_str(reset_reason));
                }
            }
            xEventGroupSetBits(sys_event_group, SYS_AUTH_OK_BIT);
            fail_count = 0;
            vTaskDelay(pdMS_TO_TICKS(10000));
        }
        else
        {
            ESP_LOGW(TAG, "Falha no login: %s", esp_err_to_name(err));
            xEventGroupClearBits(sys_event_group, SYS_AUTH_OK_BIT);

            if (fail_count < 10)
                fail_count++;

            vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
        }
    }
}

// Task para gerenciar a conexão WebSocket
void ws_manager_task(void *pv)
{
    bool was_connected = false;
    bool is_connecting = false;

    while (1)
    {
        EventBits_t bits = xEventGroupGetBits(sys_event_group);

        // Precisa de internet + auth
        if (!(bits & SYS_NET_ONLINE_BIT) || !(bits & SYS_AUTH_OK_BIT))
        {
            // ✅ OFFLINE (ou sem token): garante WS parado e não tenta reconnect
            ws_client_stop();

            was_connected = false;
            is_connecting = false;

            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Não está conectado
        if (!ws_client_is_connected())
        {
            was_connected = false;

            if (!is_connecting)
            {
                is_connecting = true;
                ESP_LOGI("WS_MGR", "Iniciando conexão WebSocket...");
                ws_client_start(api_get_token());
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Está conectado
        is_connecting = false;

        if (!was_connected)
        {
            was_connected = true;
            ESP_LOGI("WS_MGR", "WS conectado -> enviando ONLINE...");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static bool existe_emergencia_ativa_global(void)
{
    if (g_cfg.qtd_bombas <= 0)
        return false;

    return entrada_emergencia_acionada(g_emergencia[0]);
}

// Task para ler as portas do MCP23017
void ler_portas_task(void *pvParameters)
{
    static const char *TAG = "LER_PORTAS";

    // Últimos estados lidos (persistem na task)
    static int last_modo_global = -1;       // 0=LOCAL (alguma bomba em local), 1=REMOTO (todas remoto)
    static int last_emergencia_global = -1; // -1=boot, 0=sem emergencia, 1=com emergencia

    static int last_modo_bomba_lora_enviado[MAX_BOMBAS] = {-1, -1, -1};
    static int last_modo_bomba[MAX_BOMBAS] = {-1, -1, -1}; // 0=manual/local, 1=remoto
    static bool alerta_manual_pendente[MAX_BOMBAS] = {false, false, false};

    // Status por bomba (para PATCH individual)
    static int last_st_lido[MAX_BOMBAS] = {-1, -1, -1};

    static int last_st_http_enviado[MAX_BOMBAS] = {-1, -1, -1};
    static int last_st_lora_enviado[MAX_BOMBAS] = {-1, -1, -1};
    static bool sync_modo_bomba_pendente[MAX_BOMBAS] = {false, false, false};

    static int last_modo_bomba_enviado[MAX_BOMBAS] = {-1, -1, -1};

    static bool patch_pendente[MAX_BOMBAS] = {false, false, false};
    static bool patch_forcado_online[MAX_BOMBAS] = {false, false, false};
    static bool api_ready_prev = false;
    static int64_t status_periodic_offset_ms = -1;
    static int64_t last_status_periodic_ms = 0;
    static bool boot_status_pendente[MAX_BOMBAS] = {false, false, false};
    static int64_t boot_status_due_ms[MAX_BOMBAS] = {0, 0, 0};

    static int patch_valor[MAX_BOMBAS] = {-1, -1, -1};

    // Controle
    static bool alerta_emerg_pendente = false;

    ESP_LOGI(TAG, "Task de leitura de portas iniciada");

    while (1)
    {
        // ----------- LÊ MCP (tudo dentro do semáforo I2C) -----------
        int lr_bombas[MAX_BOMBAS] = {-1, -1, -1};
        int st_bombas[MAX_BOMBAS] = {-1, -1, -1};
        int emg_tanque = -1;

        int n = g_cfg.qtd_bombas;
        if (n < 0)
            n = 0;
        if (n > MAX_BOMBAS)
            n = MAX_BOMBAS;

        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            // lê LR/EM + STATUS por bomba (somente se tiver bomba)
            for (int i = 0; i < n; i++)
            {
                lr_bombas[i] = ReadPinMcp(mcp_handle, bomba_lr_ref[i].port, bomba_lr_ref[i].pin);
                st_bombas[i] = ReadPinMcp(mcp_handle, GPB, bomba_status_pin[i]);
            }
            // lê uma única emergência do tanque
            emg_tanque = ReadPinMcp(mcp_handle, GPB, Emergencia_GPB);

            xSemaphoreGive(i2c_semaphore);
        }
        else
        {
            TaskHandle_t holder = xSemaphoreGetMutexHolder(i2c_semaphore);
            ESP_LOGE(TAG, "Timeout no semáforo do I2C (holder=%s)",
                     holder ? pcTaskGetName(holder) : "NULL");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // valida leitura (local/remoto e emergência são obrigatórios)
        if (n > 0)
        {

            for (int i = 0; i < n; i++)
            {
                if (lr_bombas[i] < 0)
                {
                    ESP_LOGE(TAG, "Erro MCP (B%d lr=%d)", i + 1, lr_bombas[i]);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    goto next_loop;
                }
            }

            if (emg_tanque < 0)
            {
                ESP_LOGE(TAG, "Erro MCP (emergencia_tanque=%d)", emg_tanque);
                vTaskDelay(pdMS_TO_TICKS(1000));
                goto next_loop;
            }
        }

        // ----------- ATUALIZA ESTADO GLOBAL -----------
        for (int i = 0; i < MAX_BOMBAS; i++)
        {
            if (i < n)
            {
                g_local_remoto[i] = lr_bombas[i];
                g_emergencia[i] = emg_tanque;

                if (st_bombas[i] >= 0)
                    g_status_bomba[i] = (st_bombas[i] == 1);
            }
            else
            {
                // posições de bombas inexistentes
                g_local_remoto[i] = 0;        // local / não considerado remoto
                g_emergencia[i] = emg_tanque; // mesma emergência única do tanque
                g_status_bomba[i] = false;
                g_status_bomba_desejado[i] = false;
                g_controle_bomba_habilitado[i] = false;
            }
        }
        // ----------- EVENTO: MODO INDIVIDUAL POR BOMBA -----------
        for (int i = 0; i < n; i++)
        {
            int modo_bomba = entrada_remoto_ativo(lr_bombas[i]) ? 1 : 0; // 1=remoto, 0=manual/local

            if (last_modo_bomba[i] == -1)
            {
                last_modo_bomba[i] = modo_bomba;
                sync_modo_bomba_pendente[i] = true; // sync do estado atual no boot
            }
            else if (modo_bomba != last_modo_bomba[i])
            {
                ESP_LOGW(TAG, "Modo bomba %d mudou: %d -> %d",
                         i + 1, last_modo_bomba[i], modo_bomba);

                if (modo_bomba == 0) // entrou em manual/local
                {
                    bool bomba_estava_ativa = g_status_bomba[i] || g_status_bomba_desejado[i];

                    g_status_bomba_desejado[i] = false;
                    g_controle_bomba_habilitado[i] = false;
                    alerta_manual_pendente[i] = true;
                    sync_modo_bomba_pendente[i] = true;

                    ESP_LOGW(TAG, "Bomba %d entrou em MANUAL/LOCAL: comando interno bloqueado",
                             i + 1);

                    if (bomba_estava_ativa)
                    {
                        enviar_alerta_bomba_api_online(i,
                                                       "BOMBA PARADA",
                                                       "modo manual/local acionado");
                    }
                }
                else // voltou para remoto
                {
                    ESP_LOGI(TAG, "Bomba %d voltou para REMOTO", i + 1);

                    // não liga sozinha ao voltar para remoto
                    g_status_bomba_desejado[i] = false;
                    g_controle_bomba_habilitado[i] = false;
                    sync_modo_bomba_pendente[i] = true;
                }

                last_modo_bomba[i] = modo_bomba;
            }
        }

        // ----------- EVENTO: EMERGÊNCIA GLOBAL (OR de todas as bombas) -----------
        bool emergencia_global_ativa = entrada_emergencia_acionada(emg_tanque);
        int emergencia_global_atual = emergencia_global_ativa ? 1 : 0;

        if (last_emergencia_global == -1)
        {
            last_emergencia_global = emergencia_global_atual;
            ESP_LOGI(TAG, "Estado inicial da emergência global: %d", emergencia_global_atual);
        }
        else if (emergencia_global_atual != last_emergencia_global)
        {
            ESP_LOGW(TAG, "Emergência GLOBAL mudou: %d -> %d",
                     last_emergencia_global, emergencia_global_atual);

            if (emergencia_global_ativa)
            {
                alerta_emerg_pendente = true;

                ESP_LOGW(TAG, "Emergência GLOBAL acionada: desligando e bloqueando todas as bombas");

                EventBits_t bits = xEventGroupGetBits(sys_event_group);
                bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
                bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;

                for (int j = 0; j < n; j++)
                {
                    bool bomba_estava_ativa = g_status_bomba[j] || g_status_bomba_desejado[j];

                    g_status_bomba_desejado[j] = false;
                    g_controle_bomba_habilitado[j] = false;

                    if (g_cfg.bomba_id[j] <= 0)
                        continue;

                    if (bomba_estava_ativa)
                    {
                        enviar_alerta_bomba_api_online(j,
                                                       "BOMBA PARADA",
                                                       "emergencia global acionada");
                    }

                    if (net_ok && auth_ok)
                    {
                        if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                        {
                            (void)api_patch_bomba_status(g_cfg.bomba_id[j], false);
                            ESP_LOGW(TAG,
                                     "EMERGÊNCIA GLOBAL: PATCH HTTP bomba_id=%d status=0",
                                     g_cfg.bomba_id[j]);
                            xSemaphoreGive(MutexHTTP);
                        }
                        else
                        {
                            ESP_LOGW(TAG, "EMERGÊNCIA GLOBAL: falha ao obter MutexHTTP");
                        }
                    }
                    else
                    {
                        bool ok = lora_enqueue_json_bomba_int((uint16_t)g_cfg.bomba_id[j],
                                                              "status",
                                                              0);

                        ESP_LOGW(TAG,
                                 "EMERGÊNCIA GLOBAL: LoRa enqueue bomba_id=%d status=0 -> %s",
                                 g_cfg.bomba_id[j],
                                 ok == pdTRUE ? "OK" : "FAIL");
                    }
                }
            }
            else
            {
                ESP_LOGI(TAG, "Emergência GLOBAL liberada: novos comandos voltam a ser aceitos");
            }

            last_emergencia_global = emergencia_global_atual;
        }

        // ----------- EVENTO: MODO GLOBAL NOVO (manual só quando TODAS estiverem em manual) -----------
        int bombas_em_manual = 0;

        for (int i = 0; i < n; i++)
        {
            if (!entrada_remoto_ativo(lr_bombas[i]))
                bombas_em_manual++;
        }

        // Regra nova:
        // 0 = MANUAL somente se TODAS as bombas estiverem em manual
        // 1 = REMOTO em qualquer outro caso
        int modo_global = -1;
        if (n > 0)
        {
            modo_global = (bombas_em_manual == n) ? 0 : 1;
        }

        // Sync no boot
        if (last_modo_global == -1)
        {
            if (modo_global != -1)
            {
                last_modo_global = modo_global;
                ESP_LOGI(TAG, "Modo no boot (global novo): %d", modo_global);
            }
        }
        else if (modo_global != -1 && modo_global != last_modo_global)
        {
            ESP_LOGI(TAG, "Modo GLOBAL novo mudou: %d -> %d", last_modo_global, modo_global);

            last_modo_global = modo_global;
        }
        // ----------- EVENTO: STATUS POR BOMBA (agenda PATCH individual) -----------

        for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
        {
            int st = st_bombas[i];
            if (st < 0)
                continue; // não invalida a task inteira, só ignora essa bomba

            // Primeiro status válido após boot: agenda sync inicial
            if (last_st_lido[i] == -1)
            {
                int64_t now_boot_ms = esp_timer_get_time() / 1000;
                ESP_LOGI(TAG, "Status inicial bomba %d = %d (aguardando estabilizar)", i + 1, st);

                last_st_lido[i] = st;
                boot_status_pendente[i] = true;
                boot_status_due_ms[i] = now_boot_ms + 8000;
                continue;
            }

            // Mudança real do status
            if (st != last_st_lido[i])
            {
                ESP_LOGI(TAG, "Status bomba %d mudou: %d -> %d", i + 1, last_st_lido[i], st);

                if (boot_status_pendente[i])
                {
                    int64_t now_boot_ms = esp_timer_get_time() / 1000;
                    boot_status_due_ms[i] = now_boot_ms + 2000;
                    ESP_LOGI(TAG, "Status bomba %d mudou no boot; adiando sync final", i + 1);
                }
                else
                {
                    patch_valor[i] = st;
                    patch_pendente[i] = true;
                }
            }

            last_st_lido[i] = st;
        }

        int64_t now_boot_status_ms = esp_timer_get_time() / 1000;
        for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
        {
            if (boot_status_pendente[i] &&
                boot_status_due_ms[i] > 0 &&
                now_boot_status_ms >= boot_status_due_ms[i])
            {
                patch_valor[i] = g_status_bomba[i] ? 1 : 0;
                patch_pendente[i] = true;
                boot_status_pendente[i] = false;

                ESP_LOGI(TAG, "Sync final boot status bomba %d id=%d valor=%d",
                         i + 1,
                         g_cfg.bomba_id[i],
                         patch_valor[i]);
            }
        }

        // ----------- ENVIO PATCH STATUS POR BOMBA (somente a bomba que mudou) -----------
        {
            EventBits_t bits = xEventGroupGetBits(sys_event_group);
            bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
            bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;
            bool api_ready = net_ok && auth_ok;
            int64_t now_status_ms = esp_timer_get_time() / 1000;

            if (status_periodic_offset_ms < 0)
            {
                uint32_t slot_id = (g_cfg.tanque_id > 0) ? (uint32_t)g_cfg.tanque_id : 1U;
                status_periodic_offset_ms =
                    (int64_t)((slot_id % 20U) * BOMBA_STATUS_PERIODIC_SLOT_MS) +
                    (int64_t)(esp_random() % BOMBA_STATUS_PERIODIC_JITTER_MS);

                ESP_LOGI(TAG,
                         "Envio periodico status bomba: base=%u min offset=%lld ms",
                         (unsigned)(BOMBA_STATUS_PERIODIC_SEND_MS / 60000ULL),
                         (long long)status_periodic_offset_ms);
            }

            if (api_ready && !api_ready_prev)
            {
                for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
                {
                    if (g_cfg.bomba_id[i] <= 0)
                        continue;

                    if (boot_status_pendente[i])
                        continue;

                    patch_valor[i] = g_status_bomba[i] ? 1 : 0;
                    patch_pendente[i] = true;
                    patch_forcado_online[i] = true;

                    ESP_LOGI(TAG,
                             "API online/auth OK: agendando sync status atual bomba %d id=%d valor=%d",
                             i + 1,
                             g_cfg.bomba_id[i],
                             patch_valor[i]);
                }

                last_status_periodic_ms = now_status_ms;
            }
            else if (api_ready &&
                     last_status_periodic_ms > 0 &&
                     (uint64_t)(now_status_ms - last_status_periodic_ms) >=
                         (BOMBA_STATUS_PERIODIC_SEND_MS + (uint64_t)status_periodic_offset_ms))
            {
                for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
                {
                    if (g_cfg.bomba_id[i] <= 0)
                        continue;

                    patch_valor[i] = g_status_bomba[i] ? 1 : 0;
                    patch_pendente[i] = true;
                    patch_forcado_online[i] = true;

                    ESP_LOGI(TAG,
                             "Sync periodico API status bomba %d id=%d valor=%d",
                             i + 1,
                             g_cfg.bomba_id[i],
                             patch_valor[i]);
                }

                last_status_periodic_ms = now_status_ms;
            }
            api_ready_prev = api_ready;

            // 1) Descobre se existe algo pendente
            bool tem_pendente = false;
            for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
            {
                if (patch_pendente[i])
                {
                    tem_pendente = true;
                    break;
                }
            }

            if (tem_pendente)
            {
                if (net_ok && auth_ok)
                {
                    // ======= ONLINE: envia via HTTP (PATCH) =======
                    if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                    {
                        for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
                        {
                            if (!patch_pendente[i])
                                continue;

                            if (g_cfg.bomba_id[i] <= 0)
                            {
                                patch_pendente[i] = false;
                                patch_forcado_online[i] = false;
                                continue;
                            }

                            // evita repetir PATCH do mesmo valor já enviado
                            if (!patch_forcado_online[i] && patch_valor[i] == last_st_http_enviado[i])
                            {
                                patch_pendente[i] = false;
                                continue;
                            }

                            bool ligada = (patch_valor[i] == 1);

                            esp_err_t e = api_patch_bomba_status(g_cfg.bomba_id[i], ligada);
                            if (e == ESP_OK)
                            {
                                ESP_LOGI(TAG, "PATCH status bomba %d OK (valor=%d)", i + 1, patch_valor[i]);

                                last_st_http_enviado[i] = patch_valor[i];

                                // IMPORTANTE:
                                // atualiza também o último estado já entregue por qualquer canal
                                last_st_lora_enviado[i] = patch_valor[i];

                                patch_pendente[i] = false;
                                patch_forcado_online[i] = false;
                            }
                            else
                            {
                                ESP_LOGE(TAG, "PATCH status bomba %d falhou (%s) - vai tentar de novo",
                                         i + 1, esp_err_to_name(e));

                                // Se falhou por problema típico de conectividade,
                                // envia imediatamente via LoRa para não perder a transição.
                                if (e == ESP_ERR_HTTP_CONNECT ||
                                    e == ESP_ERR_HTTP_EAGAIN ||
                                    e == ESP_ERR_TIMEOUT)
                                {
                                    bool ok = lora_enqueue_json_bomba_int((uint16_t)g_cfg.bomba_id[i],
                                                                          "status",
                                                                          patch_valor[i] ? 1 : 0);

                                    if (ok)
                                    {
                                        ESP_LOGW(TAG, "HTTP falhou -> status bomba %d enviado via LoRa: S%d",
                                                 i + 1, patch_valor[i] ? 1 : 0);

                                        last_st_lora_enviado[i] = patch_valor[i];

                                    }
                                    else
                                    {
                                        ESP_LOGW(TAG, "HTTP falhou e fila LoRa cheia para bomba %d", i + 1);
                                    }
                                }

                                // mantém pendente para tentar HTTP novamente depois
                            }
                        }

                        xSemaphoreGive(MutexHTTP);
                    }
                }
                else
                {
                    // ===== OFFLINE: envia LoRa só quando houver mudança real =====
                    for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
                    {
                        if (!patch_pendente[i])
                            continue;

                        if (g_cfg.bomba_id[i] <= 0)
                        {
                            patch_pendente[i] = false;
                            patch_forcado_online[i] = false;
                            continue;
                        }

                        // Se já mandei esse mesmo estado por LoRa, não manda de novo
                        if (patch_valor[i] == last_st_lora_enviado[i])
                        {
                            // mantém apenas a pendência para HTTP, se necessário
                            // patch_pendente[i] = false;
                            continue;
                        }

                        bool ok = lora_enqueue_json_bomba_int((uint16_t)g_cfg.bomba_id[i],
                                                              "status",
                                                              patch_valor[i] ? 1 : 0);

                        if (ok)
                        {
                            ESP_LOGI(TAG, "LoRa status bomba %d enviado: {\"status\":%d}",
                                     i + 1, patch_valor[i] ? 1 : 0);

                            last_st_lora_enviado[i] = patch_valor[i];

                            // Não precisa reenviar LoRa enquanto estiver offline,
                            // mas precisa sincronizar HTTP quando a rede voltar.
                            patch_pendente[i] = false;
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Fila LoRa cheia ao enviar status bomba %d", i + 1);
                        }
                    }
                }
            }
        }
        // ----------- ENVIO ALERTA EMERGÊNCIA (se pendente) -----------
        if (alerta_emerg_pendente)
        {
            EventBits_t bits = xEventGroupGetBits(sys_event_group);

            if ((bits & SYS_NET_ONLINE_BIT) && (bits & SYS_AUTH_OK_BIT))
            {
                if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                {
                    char msg[160];
                    snprintf(msg, sizeof(msg),
                             "Botão de emergência acionado no tanque %s",
                             g_cfg.nome_tanque);

                    esp_err_t err = api_post_tanque(msg, g_cfg.unidade_id);

                    xSemaphoreGive(MutexHTTP);

                    if (err == ESP_OK)
                    {
                        ESP_LOGW(TAG, "ALERTA enviado com sucesso!");
                        alerta_emerg_pendente = false;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Falha ao enviar ALERTA (%s). Vai tentar de novo.",
                                 esp_err_to_name(err));
                        bool ok = lora_enqueue_alerta_codigo(1, g_cfg.unidade_id, 0, -1);
                        ESP_LOGW("LORA", "fallback ALERTA EMG codigo=1 -> %s", ok ? "OK" : "FAIL");
                        if (ok)
                            alerta_emerg_pendente = false;
                    }
                }
            }
            else
            {
                bool ok = lora_enqueue_alerta_codigo(1, g_cfg.unidade_id, 0, -1);
                ESP_LOGW("LORA", "enqueue ALERTA EMG codigo=1 -> %s", ok ? "OK" : "FAIL");

                if (ok)
                {
                    ESP_LOGW(TAG, "ALERTA emergência enviado via LoRa!");
                    alerta_emerg_pendente = false;
                }
                else
                {
                    ESP_LOGW(TAG, "Falha ao enfileirar alerta emergência via LoRa");
                }
            }
        }

        // ----------- ENVIO ALERTA MANUAL POR BOMBA -----------
        for (int i = 0; i < n; i++)
        {
            if (!alerta_manual_pendente[i])
                continue;

            EventBits_t bits = xEventGroupGetBits(sys_event_group);
            bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
            bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;

            if (net_ok && auth_ok)
            {
                if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                {
                    char msg[160];
                    snprintf(msg, sizeof(msg),
                             "ALERTA: bomba %d do tanque %s está em MANUAL",
                             i + 1, g_cfg.nome_tanque);

                    esp_err_t err = api_post_tanque(msg, g_cfg.unidade_id);

                    xSemaphoreGive(MutexHTTP);

                    if (err == ESP_OK)
                    {
                        ESP_LOGW(TAG, "Alerta MANUAL enviado para bomba %d", i + 1);
                        alerta_manual_pendente[i] = false;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Falha ao enviar alerta MANUAL da bomba %d (%s)",
                                 i + 1, esp_err_to_name(err));
                    }
                }
            }
            else
            {
                bool ok = lora_enqueue_json_bomba_int((uint16_t)g_cfg.bomba_id[i],
                                                      "automatico",
                                                      0);

                if (ok)
                {
                    ESP_LOGW(TAG, "Modo MANUAL via LoRa enviado para bomba %d", i + 1);
                    alerta_manual_pendente[i] = false;
                }
            }
        }

        // ----------- ENVIO PATCH MODO INDIVIDUAL POR BOMBA (automatico) -----------
        for (int i = 0; i < n; i++)
        {
            if (!sync_modo_bomba_pendente[i])
                continue;

            if (g_cfg.bomba_id[i] <= 0)
            {
                sync_modo_bomba_pendente[i] = false;
                continue;
            }

            EventBits_t bits = xEventGroupGetBits(sys_event_group);
            bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
            bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;

            // =========================
            // ONLINE -> envia via HTTP
            // =========================

            if (net_ok && auth_ok)
            {
                if (last_modo_bomba[i] == last_modo_bomba_enviado[i])
                {
                    sync_modo_bomba_pendente[i] = false;
                    continue;
                }

                if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                {
                    esp_err_t err = ESP_FAIL;
                    cJSON *patch = cJSON_CreateObject();

                    if (patch)
                    {
                        cJSON_AddNumberToObject(patch, "automatico", last_modo_bomba[i]);
                        err = api_patch_bomba(g_cfg.bomba_id[i], patch);
                        cJSON_Delete(patch);
                    }

                    xSemaphoreGive(MutexHTTP);

                    if (err == ESP_OK)
                    {
                        last_modo_bomba_enviado[i] = last_modo_bomba[i];
                        // last_modo_bomba_lora_enviado[i] = last_modo_bomba[i];
                        sync_modo_bomba_pendente[i] = false;

                        ESP_LOGI(TAG,
                                 "PATCH modo bomba %d OK (bomba_id=%d, automatico=%d)",
                                 i + 1,
                                 g_cfg.bomba_id[i],
                                 last_modo_bomba[i]);
                    }
                    else
                    {
                        ESP_LOGE(TAG,
                                 "PATCH modo bomba %d falhou (%s)",
                                 i + 1,
                                 esp_err_to_name(err));

                        // se falhou por conectividade, cai para LoRa também
                        if (err == ESP_ERR_HTTP_CONNECT ||
                            err == ESP_ERR_HTTP_EAGAIN ||
                            err == ESP_ERR_TIMEOUT)
                        {
                            bool ok = lora_enqueue_json_bomba_int((uint16_t)g_cfg.bomba_id[i],
                                                                  "automatico",
                                                                  last_modo_bomba[i] ? 1 : 0);

                            if (ok)
                            {
                                last_modo_bomba_lora_enviado[i] = last_modo_bomba[i];

                                ESP_LOGW(TAG,
                                         "HTTP falhou -> modo bomba %d enviado via LoRa: A%d",
                                         i + 1,
                                         last_modo_bomba[i] ? 1 : 0);
                            }
                            else
                            {
                                ESP_LOGW(TAG,
                                         "HTTP falhou e fila LoRa cheia ao enviar modo bomba %d",
                                         i + 1);
                            }
                        }
                    }
                }
            }
            // ==========================
            // OFFLINE -> envia via LoRa
            // ==========================
            else
            {
                if (last_modo_bomba[i] == last_modo_bomba_lora_enviado[i])
                {
                    continue;
                }

                bool ok = lora_enqueue_json_bomba_int((uint16_t)g_cfg.bomba_id[i],
                                                      "automatico",
                                                      last_modo_bomba[i] ? 1 : 0);
                if (ok)
                {
                    last_modo_bomba_lora_enviado[i] = last_modo_bomba[i];

                    ESP_LOGI(TAG,
                             "LoRa modo bomba %d enviado: A%d",
                             i + 1,
                             last_modo_bomba[i] ? 1 : 0);

                    // mantém pendência para sincronizar HTTP quando voltar a internet
                }
                else
                {
                    ESP_LOGW(TAG, "Fila LoRa cheia ao enviar modo bomba %d", i + 1);
                }
            }
        }

        // ----------- ATUALIZA ÚLTIMOS -----------

        last_modo_global = modo_global;
        last_emergencia_global = entrada_emergencia_acionada(emg_tanque) ? 1 : 0;

    next_loop:
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

// Função clamp para floats
static float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

// Converte (mA) em porcentagem 0..100 com base em 4..20mA

static int pct_from_4_20ma(float ma)
{
    // 4mA -> 0%, 20mA -> 100%
    float pct = (ma - 4.0f) * (100.0f / 16.0f);
    pct = clampf(pct, 0.0f, 100.0f);
    return (int)(pct + 0.5f); // arredonda
}

static bool ler_nivel_atual_pct(int *out_pct)
{
    if (!out_pct)
        return false;

    float v = -1.0f;
    if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        ads1115_set_mode(&_4a20ma, ADS1115_MODE_SINGLE);
        ads1115_set_mux(&_4a20ma, lerNivel);

        (void)ads1115_get_voltage(&_4a20ma);
        vTaskDelay(pdMS_TO_TICKS(20));

        float acc = 0.0f;
        for (int i = 0; i < 3; i++)
        {
            acc += ads1115_get_voltage(&_4a20ma);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        v = acc / 3.0f;

        xSemaphoreGive(i2c_semaphore);
    }

    if (v < 0.0f)
        return false;

    float ma = (v / SHUNT_RES_OHMS) * 1000.0f;
    *out_pct = pct_from_4_20ma(ma);
    return true;
}

// Função para buscar os limites do tanque na API (sem cache)
static bool api_fetch_tanque_limits(int *out_min, int *out_max)
{
    if (!out_min || !out_max || g_cfg.tanque_id <= 0)
        return false;

    cJSON *root = NULL;
    if (api_get_tanque_leitura(g_cfg.tanque_id, &root) != ESP_OK || !root)
        return false;

    bool ok = false;

    cJSON *nmin = cJSON_GetObjectItem(root, "nivel_minimo");
    cJSON *nmax = cJSON_GetObjectItem(root, "nivel_maximo");

    if (cJSON_IsNumber(nmin) && cJSON_IsNumber(nmax))
    {
        *out_min = nmin->valueint;
        *out_max = nmax->valueint;
        ok = true;
    }

    cJSON_Delete(root);
    return ok;
}

//  Função para obter os limites do tanque usando cache (5 min de validade)
static void api_get_tanque_limits_cached(int *out_min, int *out_max)
{
    EventBits_t bits = xEventGroupGetBits(sys_event_group);
    bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
    bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;

    if (!net_ok || !auth_ok)
    {
        *out_min = g_limits.min_pct;
        *out_max = g_limits.max_pct;
        return;
    }

    const TickType_t now = xTaskGetTickCount();
    const TickType_t ttl = pdMS_TO_TICKS(5 * 60 * 1000);

    if (g_limits.valid && (now - g_limits.last_ms) < ttl)
    {
        *out_min = g_limits.min_pct;
        *out_max = g_limits.max_pct;
        return;
    }

    int mn = g_limits.min_pct;
    int mx = g_limits.max_pct;

    if (api_fetch_tanque_limits(&mn, &mx))
    {
        g_limits.min_pct = mn;
        g_limits.max_pct = mx;
        g_limits.valid = true;
        g_limits.last_ms = now;
    }

    *out_min = g_limits.min_pct;
    *out_max = g_limits.max_pct;
}

// Task para ler o nível do tanque usando o ADS1115
void nivel_task(void *pv)
{
    esp_task_wdt_add(NULL); // registra task

    static const char *TAG = "NIVEL";

    int last_sent_pct = -1;
    bool alert_min_sent = false;
    bool alert_max_sent = false;

    int last_sent_pct_n2 = -1;
    int64_t last_send_ms_n2 = 0;
    int64_t nivel_periodic_offset_ms = -1;
    int64_t nivel2_periodic_offset_ms = -1;

    bool alert_min_sent_n2 = false;
    bool alert_max_sent_n2 = false;

    while (1)
    {
        esp_task_wdt_reset();
        float v = -1.0f;

        if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(800)) == pdTRUE)
        {
            // força SEMPRE o canal correto antes de ler
            ads1115_set_mode(&_4a20ma, ADS1115_MODE_SINGLE);
            ads1115_set_mux(&_4a20ma, lerNivel);

            // descarta 1 leitura “suja”
            (void)ads1115_get_voltage(&_4a20ma);
            vTaskDelay(pdMS_TO_TICKS(20));

            // média de 3 leituras
            float acc = 0.0f;
            for (int i = 0; i < 3; i++)
            {
                acc += ads1115_get_voltage(&_4a20ma);
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            v = acc / 3.0f;

            xSemaphoreGive(i2c_semaphore);
        }
        else
        {
            ESP_LOGW(TAG, "Timeout semáforo I2C (ADS)");
        }

        if (v < 0.0f)
        {
            ESP_LOGW(TAG, "Falha leitura ADS1115 (v=%.2f)", v);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Converte tensão -> mA -> %
        float ma = (v / SHUNT_RES_OHMS) * 1000.0f;
        int pct = pct_from_4_20ma(ma);

        // Estado da rede
        EventBits_t bits = xEventGroupGetBits(sys_event_group);
        bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
        bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;
        bool pode_http = net_ok && auth_ok;

        // Guarda últimos limites conhecidos
        static int lim_min_cache = 0;
        static int lim_max_cache = 99;

        if (pode_http)
        {
            int lim_min_tmp = lim_min_cache;
            int lim_max_tmp = lim_max_cache;
            api_get_tanque_limits_cached(&lim_min_tmp, &lim_max_tmp);
            lim_min_cache = lim_min_tmp;
            lim_max_cache = lim_max_tmp;
        }

        int lim_min = lim_min_cache;
        int lim_max = lim_max_cache;

        // Throttle único
        static int64_t last_send_ms = 0;
        int64_t now_ms = esp_timer_get_time() / 1000;

        if (nivel_periodic_offset_ms < 0)
        {
            uint16_t slot_id = tanque_lora_get_device_id();
            nivel_periodic_offset_ms = (int64_t)((slot_id % 20) * NIVEL_PERIODIC_SLOT_MS) +
                                       (int64_t)(esp_random() % NIVEL_PERIODIC_JITTER_MS);
            nivel2_periodic_offset_ms = nivel_periodic_offset_ms + 1500;
            ESP_LOGI(TAG, "Envio periodico nivel: base=%u min offset=%lld ms",
                     (unsigned)(NIVEL_PERIODIC_SEND_MS / 60000ULL),
                     (long long)nivel_periodic_offset_ms);
        }

        bool min_interval_ok = (now_ms - last_send_ms) >= 5000;
        bool mudou_bastante = (last_sent_pct < 0 || abs(pct - last_sent_pct) >= NIVEL_DEADBAND_PCT);
        uint64_t periodic_delay_ms = NIVEL_PERIODIC_SEND_MS + (uint64_t)nivel_periodic_offset_ms;
        bool envio_periodico = (last_send_ms > 0 && (uint64_t)(now_ms - last_send_ms) >= periodic_delay_ms);
        bool deve_enviar_nivel = mudou_bastante || envio_periodico;

        if (pode_http)
        {
            // ===== ONLINE: manda por HTTP =====
            if (min_interval_ok && deve_enviar_nivel)
            {
                if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                {
                    cJSON *patch = cJSON_CreateObject();
                    if (patch)
                    {
                        cJSON_AddNumberToObject(patch, "nivel_atual", pct);
                        esp_err_t err = api_patch_tanque(g_cfg.tanque_id, patch);
                        cJSON_Delete(patch);

                        if (err == ESP_OK)
                        {
                            last_sent_pct = pct;
                            last_send_ms = now_ms;
                            ESP_LOGI(TAG, "Nivel atualizado na API: %d%% (v=%.3fV, i=%.2fmA, motivo=%s)",
                                     pct, v, ma, envio_periodico ? "periodico" : "mudanca");
                        }
                        else
                        {
                            ESP_LOGW(TAG, "Falha PATCH nivel (%s)", esp_err_to_name(err));
                        }
                    }
                    xSemaphoreGive(MutexHTTP);
                }
            }
        }
        else
        {
            // ===== OFFLINE: manda por LoRa com payload curto =====
            // Exemplo: N87
            if (min_interval_ok && deve_enviar_nivel)
            {
                bool ok = lora_enqueue_json_tanque_id_int((uint16_t)g_cfg.tanque_id, "nivel_atual", pct);
                ESP_LOGI("LORA", "enqueue NIVEL pct=%d motivo=%s -> %s",
                         pct, envio_periodico ? "periodico" : "mudanca", ok ? "OK" : "FAIL");

                if (ok == pdTRUE)
                {
                    last_sent_pct = pct;
                    last_send_ms = now_ms;
                }
            }
        }

        // ================= ALERTAS =================

        // Reseta travas quando volta para dentro com histerese
        if (pct > lim_min + 1)
            alert_min_sent = false;

        if (pct < lim_max - 1)
            alert_max_sent = false;

        if (pode_http)
        {
            // ===== ALERTAS ONLINE: HTTP =====
            if (!alert_min_sent && pct <= lim_min)
            {
                if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                {
                    char msg[160];
                    snprintf(msg, sizeof(msg),
                             "ALERTA: NÍVEL MÍNIMO ATINGIDO %s",
                             NOME_CURTO);

                    esp_err_t err = api_post_tanque(msg, g_cfg.unidade_id);
                    xSemaphoreGive(MutexHTTP);

                    if (err == ESP_OK)
                    {
                        alert_min_sent = true;
                        ESP_LOGW(TAG, "Alerta MIN enviado (pct=%d lim_min=%d)", pct, lim_min);
                    }
                }
            }

            if (!alert_max_sent && pct >= lim_max)
            {
                if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                {
                    char msg[160];
                    snprintf(msg, sizeof(msg),
                             "ALERTA: NÍVEL MÁXIMO ATINGIDO %s",
                             NOME_CURTO);

                    esp_err_t err = api_post_tanque(msg, g_cfg.unidade_id);
                    xSemaphoreGive(MutexHTTP);

                    if (err == ESP_OK)
                    {
                        alert_max_sent = true;
                        ESP_LOGW(TAG, "Alerta MAX enviado (pct=%d lim_max=%d)", pct, lim_max);
                    }
                }
            }
        }
        else
        {
            // ===== ALERTAS OFFLINE: LoRa no MESMO JSON da API =====
            // {"mensagem":"...","unidade":1}

            if (!alert_min_sent && pct <= lim_min)
            {
                bool ok = lora_enqueue_alerta_codigo(6, g_cfg.unidade_id, 0, pct);

                ESP_LOGW("LORA", "enqueue ALERTA MIN codigo=6 pct=%d -> %s",
                         pct, ok ? "OK" : "FAIL");

                if (ok)
                    alert_min_sent = true;
            }

            if (!alert_max_sent && pct >= lim_max)
            {
                bool ok = lora_enqueue_alerta_codigo(7, g_cfg.unidade_id, 0, pct);

                ESP_LOGW("LORA", "enqueue ALERTA MAX codigo=7 pct=%d -> %s",
                         pct, ok ? "OK" : "FAIL");

                if (ok)
                    alert_max_sent = true;
            }
        }

        if (tanque_especial_2niveis_ativo())
        {
            float v2 = -1.0f;

            if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(800)) == pdTRUE)
            {
                ads1115_set_mode(&_4a20ma, ADS1115_MODE_SINGLE);
                ads1115_set_mux(&_4a20ma, lerNivel2);

                (void)ads1115_get_voltage(&_4a20ma);
                vTaskDelay(pdMS_TO_TICKS(20));

                float acc2 = 0.0f;
                for (int i = 0; i < 3; i++)
                {
                    acc2 += ads1115_get_voltage(&_4a20ma);
                    vTaskDelay(pdMS_TO_TICKS(20));
                }
                v2 = acc2 / 3.0f;

                xSemaphoreGive(i2c_semaphore);
            }

            if (v2 >= 0.0f)
            {
                float ma2 = (v2 / SHUNT_RES_OHMS) * 1000.0f;
                int pct2 = pct_from_4_20ma(ma2);

                // Reseta travas quando volta para dentro com histerese
                if (pct2 > lim_min + 1)
                    alert_min_sent_n2 = false;

                if (pct2 < lim_max - 1)
                    alert_max_sent_n2 = false;

                bool min_interval_ok_n2 = (now_ms - last_send_ms_n2) >= 5000;
                bool mudou_bastante_n2 = (last_sent_pct_n2 < 0 || abs(pct2 - last_sent_pct_n2) >= NIVEL_DEADBAND_PCT);
                uint64_t periodic_delay_n2_ms = NIVEL_PERIODIC_SEND_MS + (uint64_t)nivel2_periodic_offset_ms;
                bool envio_periodico_n2 = (last_send_ms_n2 > 0 &&
                                           (uint64_t)(now_ms - last_send_ms_n2) >= periodic_delay_n2_ms);
                bool deve_enviar_nivel2 = mudou_bastante_n2 || envio_periodico_n2;

                if (min_interval_ok_n2 && deve_enviar_nivel2)
                {
                    if (pode_http)
                    {
                        if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                        {
                            cJSON *patch2 = cJSON_CreateObject();
                            if (patch2)
                            {
                                cJSON_AddNumberToObject(patch2, "nivel_atual", pct2);
                                esp_err_t err2 = api_patch_tanque(TANQUE_ESPECIAL_NIVEL2_ID, patch2);
                                cJSON_Delete(patch2);

                                if (err2 == ESP_OK)
                                {
                                    last_sent_pct_n2 = pct2;
                                    last_send_ms_n2 = now_ms;
                                    ESP_LOGI(TAG, "Nivel 2 atualizado na API: %d%% (v=%.3fV, i=%.2fmA, motivo=%s)",
                                             pct2, v2, ma2, envio_periodico_n2 ? "periodico" : "mudanca");
                                }
                                else
                                {
                                    ESP_LOGW(TAG, "Falha PATCH nivel 2 (%s)", esp_err_to_name(err2));
                                }
                            }
                            xSemaphoreGive(MutexHTTP);
                        }
                    }
                    else
                    {
                        bool ok2 = lora_enqueue_json_tanque_id_int((uint16_t)TANQUE_ESPECIAL_NIVEL2_ID,
                                                                   "nivel_atual",
                                                                   pct2);

                        ESP_LOGI("LORA", "enqueue NIVEL2 pct=%d motivo=%s -> %s",
                                 pct2, envio_periodico_n2 ? "periodico" : "mudanca", ok2 ? "OK" : "FAIL");

                        if (ok2 == pdTRUE)
                        {
                            last_sent_pct_n2 = pct2;
                            last_send_ms_n2 = now_ms;
                        }
                    }
                }

                // ===== ALERTAS NIVEL 2 =====
                if (pode_http)
                {
                    if (!alert_min_sent_n2 && pct2 <= lim_min)
                    {
                        if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                        {
                            char msg[160];
                            snprintf(msg, sizeof(msg),
                                     "ALERTA: NÍVEL MÍNIMO ATINGIDO %s NIVEL 2",
                                     NOME_CURTO);

                            esp_err_t err = api_post_tanque(msg, g_cfg.unidade_id);
                            xSemaphoreGive(MutexHTTP);

                            if (err == ESP_OK)
                            {
                                alert_min_sent_n2 = true;
                                ESP_LOGW(TAG, "Alerta MIN NIVEL 2 enviado (pct2=%d lim_min=%d)", pct2, lim_min);
                            }
                        }
                    }

                    if (!alert_max_sent_n2 && pct2 >= lim_max - 1)
                    {
                        if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                        {
                            char msg[160];
                            snprintf(msg, sizeof(msg),
                                     "ALERTA: NÍVEL MÁXIMO ATINGIDO %s NIVEL 2",
                                     NOME_CURTO);

                            esp_err_t err = api_post_tanque(msg, g_cfg.unidade_id);
                            xSemaphoreGive(MutexHTTP);

                            if (err == ESP_OK)
                            {
                                alert_max_sent_n2 = true;
                                ESP_LOGW(TAG, "Alerta MAX NIVEL 2 enviado (pct2=%d lim_max=%d)", pct2, lim_max);
                            }
                        }
                    }
                }
                else
                {
                    if (!alert_min_sent_n2 && pct2 <= lim_min)
                    {
                        bool ok = lora_enqueue_alerta_codigo(8, g_cfg.unidade_id, 0, pct2);

                        ESP_LOGW("LORA", "enqueue ALERTA MIN NIVEL2 codigo=8 pct=%d -> %s",
                                 pct2, ok ? "OK" : "FAIL");

                        if (ok)
                            alert_min_sent_n2 = true;
                    }

                    if (!alert_max_sent_n2 && pct2 >= lim_max - 1)
                    {
                        bool ok = lora_enqueue_alerta_codigo(9, g_cfg.unidade_id, 0, pct2);

                        ESP_LOGW("LORA", "enqueue ALERTA MAX NIVEL2 codigo=9 pct=%d -> %s",
                                 pct2, ok ? "OK" : "FAIL");

                        if (ok)
                            alert_max_sent_n2 = true;
                    }
                }
            }
            else
            {
                ESP_LOGW(TAG, "Falha leitura ADS1115 nivel 2 (v2=%.2f)", v2);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(NIVEL_TASK_PERIOD_MS));
    }
}

// função para múltiplas bombas (ex: BOMBA1_GPA, BOMBA2_GPA, etc.)
static esp_err_t mcp_escrever_bomba_idx(int idx, bool ligar)
{
    if (idx < 0 || idx >= g_cfg.qtd_bombas)
        return ESP_ERR_INVALID_ARG;
    if (!mcp_handle)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(500)) != pdTRUE)
        return ESP_ERR_TIMEOUT;

    esp_err_t err = WritePinMcp(mcp_handle, GPA, bomba_out_pin[idx], ligar ? 1 : 0);
    xSemaphoreGive(i2c_semaphore);
    return err;
}

//========================================================================================================

// Task para gerenciar o controle das bombas (liga/desliga conforme desejado)
void bombas_task(void *pv)
{
    bool ultimo_desejado[MAX_BOMBAS] = {0};
    bool aguardando[MAX_BOMBAS] = {0};
    int64_t t0_ms[MAX_BOMBAS] = {0};



for (int i = 0; i < g_cfg.qtd_bombas; i++)
{
    bool restaurar = load_status_bomba(i);

    g_status_bomba_desejado[i] = restaurar;
    ultimo_desejado[i] = !restaurar; // força aplicar no primeiro loop

    printf("Bomba %d restaurada: %d\n", i + 1, restaurar ? 1 : 0);
}

    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1)
    {
        for (int i = 0; i < g_cfg.qtd_bombas; i++)
        {
            int modo_bomba = entrada_remoto_ativo(g_local_remoto[i]) ? 1 : 0;

            g_controle_bomba_habilitado[i] = (modo_bomba == 1);

            if (modo_bomba == 0)
            {
                g_status_bomba_desejado[i] = false;
            }

            // Segurança: só age se controle habilitado (por bomba)
            if (!g_controle_bomba_habilitado[i])
            {
                mcp_escrever_bomba_idx(i, false);
                aguardando[i] = false;
                ultimo_desejado[i] = false;
                save_status_bomba(i, false);
                continue;
            }

            if (!entrada_remoto_ativo(g_local_remoto[i]) || existe_emergencia_ativa_global())
            {
                mcp_escrever_bomba_idx(i, false);
                aguardando[i] = false;
                ultimo_desejado[i] = false;
                continue;
            }

            bool desejado = g_status_bomba_desejado[i];

            // Mudou desejado?

            if (desejado != ultimo_desejado[i])
            {

                printf("  status_desejado: %s\n", g_status_bomba_desejado[i] ? "LIGADA" : "DESLIGADA");

                printf("_______________\n");

                printf("Ultimo  status_desejado: %s\n", ultimo_desejado[i] ? "LIGADA" : "DESLIGADA");

                ultimo_desejado[i] = desejado;

                ESP_LOGW(TAG_BOMBA, "[B%d id=%d] Desejado -> %s",
                         i + 1, g_cfg.bomba_id[i], desejado ? "LIGAR" : "DESLIGAR");

                if (mcp_escrever_bomba_idx(i, desejado) == ESP_OK)
                {
                    vTaskDelay(pdMS_TO_TICKS(500));

                    aguardando[i] = true;
                    t0_ms[i] = esp_timer_get_time() / 1000;         

                    save_status_bomba(i, desejado);
                }


            }

            // Confirmação real
            if (aguardando[i])
            {
                bool real_atual = g_status_bomba[i];

                if (real_atual == desejado)
                {
                    ESP_LOGI(TAG_BOMBA, "[B%d id=%d] Confirmado pelo retorno real: %s",
                             i + 1, g_cfg.bomba_id[i], desejado ? "LIGADA" : "DESLIGADA");

                    aguardando[i] = false;
                }
                else
                {
                    int64_t now = esp_timer_get_time() / 1000;
                    if (now - t0_ms[i] > 30000)
                    {
                        ESP_LOGE(TAG_BOMBA, "[B%d id=%d] Timeout confirmacao 30s (real=%d desejado=%d)",
                                 i + 1, g_cfg.bomba_id[i],
                                 real_atual ? 1 : 0,
                                 desejado ? 1 : 0);
                        enviar_alerta_bomba_api_online(i,
                                                       desejado ? "FALHA AO LIGAR BOMBA" : "FALHA AO DESLIGAR BOMBA",
                                                       "retorno real nao confirmou em 30s");
                        aguardando[i] = false;
                    }
                }
            }
        }

        pwm_atualizar_saida_por_estado();
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Função para obter o índice da bomba no array de configuração a partir do ID da bomba
static int bomba_index_from_id(uint16_t bomba_id)
{
    for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
        if (g_cfg.bomba_id[i] == (int)bomba_id)
            return i;
    return -1;
}

// Converte mA (4-20) para um valor dentro de um range (ex: 0..500V)
static float map_4_20_to_range(float ma, float out_min, float out_max)
{
    // 4mA -> out_min, 20mA -> out_max
    float v = (ma - 4.0f) * ((out_max - out_min) / 16.0f) + out_min;
    return clampf(v, out_min, out_max);
}

static bool ler_corrente_bomba_idx(int idx, float *out_corrente_a)
{
    static const char *TAGC = "CORRENTE";

    if (!out_corrente_a)
        return false;

    if (idx < 0 || idx >= g_cfg.qtd_bombas || idx >= MAX_BOMBAS)
        return false;

    if (xSemaphoreTake(i2c_semaphore, pdMS_TO_TICKS(1200)) != pdTRUE)
    {
        TaskHandle_t holder = xSemaphoreGetMutexHolder(i2c_semaphore);
        ESP_LOGE(TAGC, "Falha ao obter I2C para corrente bomba %d (holder=%s)",
                 idx + 1,
                 holder ? pcTaskGetName(holder) : "NULL");
        return false;
    }

    ads1115_set_mode(&_4a20ma, ADS1115_MODE_SINGLE);
    ads1115_set_mux(&_4a20ma, bomba_corrente_mux[idx]);

    float v0 = ads1115_get_voltage(&_4a20ma);
    vTaskDelay(pdMS_TO_TICKS(20));

    if (v0 < 0.0f)
    {
        ESP_LOGW(TAGC, "Leitura ADS inválida na bomba %d (descarta)", idx + 1);
        xSemaphoreGive(i2c_semaphore);
        return false;
    }

    float acc = 0.0f;
    const int N = 3;

    for (int i = 0; i < N; i++)
    {
        float v = ads1115_get_voltage(&_4a20ma);

        if (v < 0.0f)
        {
            ESP_LOGW(TAGC, "Leitura ADS inválida na bomba %d (amostra %d/%d)",
                     idx + 1, i + 1, N);
            xSemaphoreGive(i2c_semaphore);
            return false;
        }

        acc += v;
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    xSemaphoreGive(i2c_semaphore);

    float v_avg = acc / (float)N;
    float ma = (v_avg / SHUNT_RES_OHMS) * 1000.0f;
    float corrente_a = map_4_20_to_range(ma, 0.0f, bomba_tc_max_a[idx]);

    *out_corrente_a = corrente_a;
    return true;
}

void corrente_task(void *pv)
{
    static const char *TAGC = "CORRENTE";

    float last_corrente[MAX_BOMBAS] = {-1000.0f, -1000.0f, -1000.0f};

    // Guarda o último estado que foi sincronizado com sucesso na API
    bool last_bomba_ligada_sync[MAX_BOMBAS] = {false, false, false};

    int64_t last_send_ms[MAX_BOMBAS] = {0, 0, 0};

    while (1)
    {
        EventBits_t bits = xEventGroupGetBits(sys_event_group);
        bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
        bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;
        bool pode_http = net_ok && auth_ok;

        int64_t now_ms = esp_timer_get_time() / 1000;

        for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
        {
            if (g_cfg.bomba_id[i] <= 0)
                continue;

            bool bomba_ligada = g_status_bomba[i];

            // =========================================================
            // BOMBA DESLIGADA:
            // - não mede corrente
            // - envia 0 no boot ou quando houver transição para desligada
            // =========================================================
            if (!bomba_ligada)
            {
                bool precisa_zerar =
                    (last_bomba_ligada_sync[i] != false) ||
                    (last_corrente[i] != 0.0f);

                if (!precisa_zerar)
                {
                    continue;
                }

                if (!pode_http)
                {
                    continue;
                }

                if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
                {
                    esp_err_t err = ESP_FAIL;
                    cJSON *patch = cJSON_CreateObject();

                    if (patch)
                    {
                        cJSON_AddNumberToObject(patch, "corrente", 0);
                        err = api_patch_bomba(g_cfg.bomba_id[i], patch);
                        cJSON_Delete(patch);
                    }
                    else
                    {
                        ESP_LOGE(TAGC, "Sem memória p/ JSON PATCH corrente zero");
                    }

                    xSemaphoreGive(MutexHTTP);

                    if (err == ESP_OK)
                    {
                        last_bomba_ligada_sync[i] = false;
                        last_corrente[i] = 0.0f;
                        last_send_ms[i] = now_ms;

                        ESP_LOGI(TAGC, "Bomba %d desligada -> corrente zerada na API", i + 1);
                    }
                    else
                    {
                        ESP_LOGW(TAGC, "Falha ao zerar corrente da bomba %d (%s)",
                                 i + 1, esp_err_to_name(err));
                    }
                }

                continue;
            }

            // =========================================================
            // BOMBA LIGADA:
            // - mede corrente somente aqui
            // =========================================================
            float corrente_a = 0.0f;
            bool ok = ler_corrente_bomba_idx(i, &corrente_a);

            if (!ok)
            {
                ESP_LOGW(TAGC, "Falha leitura corrente bomba %d", i + 1);
                continue;
            }

            int corrente_i = (int)(corrente_a + 0.5f);

            bool mudou_estado = (last_bomba_ligada_sync[i] != true);
            bool mudou_corrente =
                (last_corrente[i] < 0.0f) ||
                (fabsf(corrente_a - last_corrente[i]) >= CORRENTE_DEADBAND_A);

            bool intervalo_ok = (now_ms - last_send_ms[i]) >= CORRENTE_SEND_MIN_MS;

            // ESP_LOGI(TAGC, "Bomba %d ligada -> corrente: %.2f A", i + 1, corrente_a);

            // Envia imediatamente:
            // - no boot
            // - quando a bomba mudou para ligada
            // - ou quando mudou a corrente e respeitou intervalo
            bool deve_enviar =
                mudou_estado ||
                (mudou_corrente && intervalo_ok);

            if (!deve_enviar)
            {
                continue;
            }

            if (!pode_http)
            {
                continue;
            }

            if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
            {
                esp_err_t err = ESP_FAIL;
                cJSON *patch = cJSON_CreateObject();

                if (patch)
                {
                    cJSON_AddNumberToObject(patch, "corrente", corrente_i);
                    err = api_patch_bomba(g_cfg.bomba_id[i], patch);
                    cJSON_Delete(patch);
                }
                else
                {
                    ESP_LOGE(TAGC, "Sem memória p/ JSON PATCH corrente");
                }

                xSemaphoreGive(MutexHTTP);

                if (err == ESP_OK)
                {
                    last_bomba_ligada_sync[i] = true;
                    last_corrente[i] = corrente_a;
                    last_send_ms[i] = now_ms;

                    ESP_LOGI(TAGC, "PATCH corrente bomba %d OK -> %d A",
                             i + 1, corrente_i);
                }
                else
                {
                    ESP_LOGW(TAGC, "Falha PATCH corrente bomba %d (%s)",
                             i + 1, esp_err_to_name(err));
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CORRENTE_TASK_PERIOD_MS));
    }
}

// Reinicia o sistema
void ConnectRest()
{
    printf("\033[1;36m\n\n========== SISTEMA REINICIANDO: Connect ==========\n\n\033[0m");

    // Resetar GPIOs (já presente e recomendado)
    gpio_reset_pin(I2C_SDA);
    gpio_reset_pin(I2C_SCL);

    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart(); // Reinicia o ESP
}

/*#########################################  Funçoes Watchdog Timer   ######################################### */

// Callback do timer: se chegar aqui, é porque o timer expirou (timeout), ou seja, nenhuma atividade detectada
void watchdog_callback(void *arg)
{
    g_last_restart_marker = 1;
    ESP_EARLY_LOGE("SOFT_WDT", "Watchdog de software expirou -> reiniciando");
    ConnectRest();
}

// Função para iniciar o timer watchdog (10 segundos)
void start_watchdog_timer()
{
    const esp_timer_create_args_t timer_args = {
        .callback = &watchdog_callback,
        .name = "watchdog_timer"};
    esp_timer_create(&timer_args, &watchdog_timer);
    esp_timer_start_once(watchdog_timer, 10000000); // 10 segundos
}

// Função para resetar o timer watchdog (chame sempre que detectar atividade)
void reset_watchdog_timer()
{
    esp_timer_stop(watchdog_timer);
    esp_timer_start_once(watchdog_timer, 10000000); // Reinicia para 10s
}

/*############################################## Lora  ################################################*/
// Configura o módulo LoRa E32 com os pinos e parâmetros corretos para o tanque
static void lora_setup_tanque(void)
{
    lora_e32_config_t cfg = {
        .uart_num = UART_NUM_1,
        .tx_pin = 16,
        .rx_pin = 18,
        .m0_pin = 12,
        .m1_pin = 14,
        .rst_pin = -1,
        .uart_baudrate = 9600,

        .head = 0xC0,
        .addh = 0x00,
        .addl = 0x01,
        .speed = 0x18, // 9600 UART + menor air rate para alcance longo
        .channel = 0x17,
        .option = 0x64,
    };

    ESP_ERROR_CHECK(lora_e32_init(&cfg));
    ESP_ERROR_CHECK(lora_e32_apply_cfg());
}

typedef struct {
    uint32_t timeout;
    uint32_t frame_ok;
    uint32_t incomplete;
    uint32_t bad_preamble;
    uint32_t bad_crc;
    uint32_t duplicate;
    uint32_t wrong_dst;
    uint32_t ack_rx;
    uint32_t ack_match;
    uint32_t ack_unexpected;
    uint32_t ack_sent_for_payload;
    uint32_t app_queue_ok;
    uint32_t app_queue_drop;
} tanque_lora_rx_stats_t;

static void tanque_lora_log_rx_stats(const tanque_lora_rx_stats_t *s, const char *motivo)
{
    if (!s)
        return;

    ESP_LOGW("LORA_RX_STATS",
             "motivo=%s timeout=%lu frame_ok=%lu incomplete=%lu preamble=%lu crc=%lu dup=%lu wrong_dst=%lu "
             "ack_rx=%lu ack_match=%lu ack_unexp=%lu ack_sent=%lu app_ok=%lu app_drop=%lu",
             motivo,
             (unsigned long)s->timeout,
             (unsigned long)s->frame_ok,
             (unsigned long)s->incomplete,
             (unsigned long)s->bad_preamble,
             (unsigned long)s->bad_crc,
             (unsigned long)s->duplicate,
             (unsigned long)s->wrong_dst,
             (unsigned long)s->ack_rx,
             (unsigned long)s->ack_match,
             (unsigned long)s->ack_unexpected,
             (unsigned long)s->ack_sent_for_payload,
             (unsigned long)s->app_queue_ok,
             (unsigned long)s->app_queue_drop);
}

uint16_t lora_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static size_t lora_frame_air_len(const lora_app_frame_t *frame) {
    if (!frame || frame->len > sizeof(frame->payload))
        return 0;

    return offsetof(lora_app_frame_t, payload) + frame->len + sizeof(frame->crc);
}

static int lora_send_frame_air(lora_app_frame_t *frame) {
    if (!frame)
        return -1;

    size_t header_len = offsetof(lora_app_frame_t, payload);
    size_t payload_len = frame->len;
    size_t air_len = lora_frame_air_len(frame);

    if (air_len == 0 || air_len > sizeof(lora_app_frame_t))
        return -1;

    uint8_t raw[sizeof(lora_app_frame_t)] = {0};
    memcpy(raw, frame, header_len + payload_len);

    frame->crc = lora_crc16(raw, header_len + payload_len);
    memcpy(raw + header_len + payload_len, &frame->crc, sizeof(frame->crc));

    return lora_e32_send_raw(raw, (int)air_len);
}

static bool lora_received_frame_valid(const lora_app_frame_t *rx, int air_len, uint16_t *out_crc_calc, uint16_t *out_crc_rx) {
    if (!rx || air_len < (int)(offsetof(lora_app_frame_t, payload) + sizeof(rx->crc)))
        return false;

    size_t expected_len = lora_frame_air_len(rx);
    if (expected_len == 0 || air_len != (int)expected_len)
        return false;

    const uint8_t *raw = (const uint8_t *)rx;
    uint16_t crc_rx = 0;
    memcpy(&crc_rx, raw + expected_len - sizeof(crc_rx), sizeof(crc_rx));
    uint16_t crc_calc = lora_crc16(raw, expected_len - sizeof(crc_rx));

    if (out_crc_calc)
        *out_crc_calc = crc_calc;
    if (out_crc_rx)
        *out_crc_rx = crc_rx;

    return crc_calc == crc_rx;
}

static void lora_ack_wait_begin(uint8_t msg_id, uint8_t expected_src_type, uint16_t expected_src_id) {
    xSemaphoreTake(g_lora_ack_mutex, portMAX_DELAY);
    g_lora_ack_wait.active = true;
    g_lora_ack_wait.msg_id = msg_id;
    g_lora_ack_wait.expected_src_type = expected_src_type;
    g_lora_ack_wait.expected_src_id = expected_src_id;
    g_lora_ack_wait.waiter = xTaskGetCurrentTaskHandle();
    xSemaphoreGive(g_lora_ack_mutex);
}

static void lora_ack_wait_cancel(void) {
    xSemaphoreTake(g_lora_ack_mutex, portMAX_DELAY);
    g_lora_ack_wait.active = false;
    g_lora_ack_wait.msg_id = 0;
    g_lora_ack_wait.expected_src_type = 0;
    g_lora_ack_wait.expected_src_id = 0;
    g_lora_ack_wait.waiter = NULL;
    xSemaphoreGive(g_lora_ack_mutex);
}

static bool lora_ack_wait_match_and_signal(const lora_app_frame_t *rx) {
    bool matched = false;
    TaskHandle_t waiter = NULL;

    xSemaphoreTake(g_lora_ack_mutex, portMAX_DELAY);

    if (g_lora_ack_wait.active && rx->msg_type == MSG_ACK && rx->src_type == g_lora_ack_wait.expected_src_type &&
        rx->src_id == g_lora_ack_wait.expected_src_id && rx->msg_id == g_lora_ack_wait.msg_id) {
        matched = true;
        waiter = g_lora_ack_wait.waiter;
        g_lora_ack_wait.active = false;
        g_lora_ack_wait.msg_id = 0;
        g_lora_ack_wait.expected_src_type = 0;
        g_lora_ack_wait.expected_src_id = 0;
        g_lora_ack_wait.waiter = NULL;
    }

    xSemaphoreGive(g_lora_ack_mutex);

    if (matched && waiter) {
        xTaskNotifyGive(waiter);
    }

    return matched;
}

static bool lora_wait_ack_notification(uint32_t timeout_ms) {
    return (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms)) > 0);
}

static bool lora_is_duplicate(const lora_app_frame_t *rx) {
    TickType_t now = xTaskGetTickCount();
    TickType_t window = pdMS_TO_TICKS(LORA_DUP_WINDOW_MS);

    for (int i = 0; i < LORA_DUP_CACHE_SIZE; i++) {
        if (g_lora_dup_cache[i].used && g_lora_dup_cache[i].src_type == rx->src_type &&
            g_lora_dup_cache[i].src_id == rx->src_id && g_lora_dup_cache[i].msg_type == rx->msg_type &&
            g_lora_dup_cache[i].msg_id == rx->msg_id) {
            if ((now - g_lora_dup_cache[i].seen_at) <= window) {
                return true;
            }
        }
    }

    int slot = rx->msg_id % LORA_DUP_CACHE_SIZE;
    g_lora_dup_cache[slot].used = true;
    g_lora_dup_cache[slot].src_type = rx->src_type;
    g_lora_dup_cache[slot].src_id = rx->src_id;
    g_lora_dup_cache[slot].msg_type = rx->msg_type;
    g_lora_dup_cache[slot].msg_id = rx->msg_id;
    g_lora_dup_cache[slot].seen_at = now;

    return false;
}

static void tanque_send_ack(const lora_app_frame_t *rx) {
    lora_app_frame_t ack = {0};

    ack.preamble = LORA_PREAMBLE;
    ack.src_type = DEV_TANK;
    ack.src_id = tanque_lora_get_device_id();
    ack.dst_type = rx->src_type;
    ack.dst_id = rx->src_id;
    ack.msg_type = MSG_ACK;
    ack.msg_id = rx->msg_id;
    ack.len = 0;
    ack.has_bomba_id = 0;
    ack.bomba_id = 0;
    if (xSemaphoreTake(MutexLora, pdMS_TO_TICKS(LORA_ACK_SEND_TIMEOUT_MS)) == pdTRUE) {
        lora_send_frame_air(&ack);
        xSemaphoreGive(MutexLora);
        ESP_LOGI("LORA", "ACK enviado -> dst=%u msg_id=%u", rx->src_id, rx->msg_id);
        vTaskDelay(pdMS_TO_TICKS(LORA_POST_TX_GUARD_MS));
    } else {
        ESP_LOGW("LORA", "Falha ao pegar MutexLora para ACK msg_id=%u", rx->msg_id);
    }
}

// Envia uma mensagem para a GTW via LoRa, esperando ACK se configurado. Retorna true se ACK recebido ou se não requer ACK.
bool tanque_send_to_gtw_ack(uint16_t src_tank_id, uint16_t gtw_id, const char *msg, uint8_t has_bomba,
                            uint16_t bomba_id){
    lora_app_frame_t frame = {0};

    frame.preamble = LORA_PREAMBLE;
    frame.src_type = DEV_TANK;
    frame.src_id = (src_tank_id != 0) ? src_tank_id : tanque_lora_get_device_id();
    frame.dst_type = DEV_GTW;
    frame.dst_id = (gtw_id != 0) ? gtw_id : LORA_GTW_ID;
    frame.msg_type = MSG_DATA;
    frame.msg_id = msg_counter++;
    frame.has_bomba_id = has_bomba;
    frame.bomba_id = bomba_id;

    frame.len = (uint8_t)strnlen(msg ? msg : "", sizeof(frame.payload));
    if (frame.len > 0) {
        memcpy(frame.payload, msg, frame.len);
    }

    for (int attempt = 1; attempt <= LORA_MAX_RETRIES; attempt++) {
        ESP_LOGI("LORA", "TX DATA -> dst=%u msg_id=%u tentativa=%d", frame.dst_id, frame.msg_id, attempt);

#if (LORA_REQUIRE_ACK != 0)
        lora_ack_wait_begin(frame.msg_id, DEV_GTW, frame.dst_id);
#endif

        if (xSemaphoreTake(MutexLora, pdMS_TO_TICKS(2500)) == pdTRUE) {
            lora_send_frame_air(&frame);
            xSemaphoreGive(MutexLora);

            vTaskDelay(pdMS_TO_TICKS(LORA_POST_TX_GUARD_MS));
        } else {
#if (LORA_REQUIRE_ACK != 0)
            lora_ack_wait_cancel();
#endif
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

#if (LORA_REQUIRE_ACK == 0)
        return true;
#else
        if (lora_wait_ack_notification(LORA_ACK_TIMEOUT_MS)) {
            ESP_LOGI("LORA", "ACK recebido id=%u", frame.msg_id);
            return true;
        }

        lora_ack_wait_cancel();
        ESP_LOGW("LORA", "ACK timeout id=%u tentativa=%d", frame.msg_id, attempt);
        uint32_t backoff_ms = (LORA_RETRY_BACKOFF_MIN_MS * attempt) +
                              (esp_random() % LORA_RETRY_BACKOFF_JITTER_MS);
        ESP_LOGW("LORA", "Nova tentativa LoRa em %u ms", (unsigned)backoff_ms);
        vTaskDelay(pdMS_TO_TICKS(backoff_ms));
#endif
    }

    ESP_LOGE("LORA >>>", "Falha envio id=%d", frame.msg_id);
    return false;
}

bool tanque_send_to_gtw_noack(uint16_t src_tank_id, uint16_t gtw_id, const char *msg, uint8_t has_bomba, uint16_t bomba_id) {
    lora_app_frame_t frame = {0};

    frame.preamble = LORA_PREAMBLE;
    frame.src_type = DEV_TANK;
    frame.src_id = (src_tank_id != 0) ? src_tank_id : tanque_lora_get_device_id();
    frame.dst_type = DEV_GTW;
    frame.dst_id = (gtw_id != 0) ? gtw_id : LORA_GTW_ID;
    frame.msg_type = MSG_DATA;
    frame.msg_id = msg_counter++;
    frame.has_bomba_id = has_bomba;
    frame.bomba_id = bomba_id;
    frame.len = (uint8_t)strnlen(msg ? msg : "", sizeof(frame.payload));

    if (frame.len > 0) {
        memcpy(frame.payload, msg, frame.len);
    }

    if (xSemaphoreTake(MutexLora, pdMS_TO_TICKS(1200)) != pdTRUE) {
        return false;
    }

    lora_send_frame_air(&frame);
    xSemaphoreGive(MutexLora);
    vTaskDelay(pdMS_TO_TICKS(LORA_POST_TX_GUARD_MS));
    return true;
}

// Task para enviar mensagens do tanque para a GTW via LoRa, lendo de uma fila. Cada item da fila contém a mensagem e metadados.
void tanque_lora_tx_task(void *pv) {
    tx_item_t item;

    ESP_LOGI("LORA", "TX TANQUE iniciado");

    while (1) {
        if (xQueueReceive(lora_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            uint16_t src_tank_id = item.src_tank_id;
            if (src_tank_id == 0) {
                src_tank_id = tanque_lora_get_device_id();
            }

#if (LORA_REQUIRE_ACK != 0)
            (void)tanque_send_to_gtw_ack(src_tank_id, item.gtw_id, item.msg, item.has_bomba, item.bomba_id);
#else
            (void)tanque_send_to_gtw_noack(src_tank_id, item.gtw_id, item.msg, item.has_bomba, item.bomba_id);
#endif
        }
    }
}

static void tanque_process_cmd_from_gtw(const lora_app_frame_t *rx) {
    char json[65];
    size_t copy_len = rx->len;

    if (copy_len > sizeof(rx->payload))
        copy_len = sizeof(rx->payload);
    if (copy_len > 64)
        copy_len = 64;

    memcpy(json, rx->payload, copy_len);
    json[copy_len] = '\0';

    ESP_LOGI("LORA", "CMD recebido -> src=%u msg_id=%u bomba_id=%u has_bomba=%u payload=%s", rx->src_id, rx->msg_id,
             rx->bomba_id, rx->has_bomba_id, json);

    cJSON *root = cJSON_Parse(json);
    if (!root) {
        ESP_LOGE("LORA", "Erro ao parsear JSON recebido do GTW");
        return;
    }

    cJSON *cmd = cJSON_GetObjectItemCaseSensitive(root, "cmd");
    cJSON *status = cJSON_GetObjectItemCaseSensitive(root, "status");
    if (!status)
        status = cJSON_GetObjectItemCaseSensitive(root, "s");
    cJSON *vazao = cJSON_GetObjectItemCaseSensitive(root, "vazao");
    if (!vazao)
        vazao = cJSON_GetObjectItemCaseSensitive(root, "v");
    cJSON *controle = cJSON_GetObjectItemCaseSensitive(root, "c");

    bool tem_vazao = cJSON_IsNumber(vazao);
    bool cmd_bomba_ok = !cmd || (cJSON_IsString(cmd) && strcmp(cmd->valuestring, "bomba") == 0);
    bool tem_cmd_bomba = cmd_bomba_ok && cJSON_IsNumber(status) && rx->has_bomba_id;

    bool eh_bomba_pwm = rx->has_bomba_id && (rx->bomba_id == BOMBA_PWM_ID);

    float valor_vazao = 0.0f;
    if (tem_vazao)
        valor_vazao = (float)vazao->valuedouble;

    bool vazao_mudou = false;
    if (tem_vazao && eh_bomba_pwm) {
        vazao_mudou = (fabsf(valor_vazao - g_vazao_pwm_percent) > 0.01f);
    }

    if (tem_vazao && eh_bomba_pwm) {
        pwm_processar_novo_setpoint(valor_vazao);
        pwm_agendar_sync_vazao(rx->bomba_id, valor_vazao);

        ESP_LOGI("LORA", "Setpoint PWM recebido via LoRa -> bomba_id=%u valor=%.2f%%", rx->bomba_id,
                 (double)valor_vazao);
    } else if (tem_vazao && rx->has_bomba_id && !eh_bomba_pwm) {
        ESP_LOGI("LORA", "Vazao ignorada -> bomba_id=%u nao eh a bomba PWM (%u)", rx->bomba_id, BOMBA_PWM_ID);
    }

    bool processa_cmd = tem_cmd_bomba;

    if (processa_cmd && eh_bomba_pwm && tem_vazao) {
        int st = status->valueint;
        if (st == 1 && vazao_mudou) {
            processa_cmd = false;
            ESP_LOGI("LORA", "Pacote interpretado como AJUSTE DE VAZAO -> comando nao processado");
        }
    }

    if (processa_cmd) {
        ESP_LOGI("LORA", "CMD=bomba STATUS=%d BombaID=%u", status->valueint, rx->bomba_id);

        int controle_id = cJSON_IsNumber(controle) ? controle->valueint : (int)rx->bomba_id;
        (void)tratar_comando_bomba("bomba", status->valueint, rx->bomba_id, controle_id);
    } else if (!tem_vazao && !tem_cmd_bomba) {
        ESP_LOGW("LORA", "JSON sem cmd/status validos e sem vazao valida");
    }

    pwm_atualizar_saida_por_estado();
    cJSON_Delete(root);
}

// Função para tratar o comando de controle da bomba recebido via LoRa. Verifica validade e aplica na GPIO.
bool tratar_comando_bomba(const char *cmd, int valor, uint16_t bomba_id, int controle_id){
    // 1) Verifica comando
    if (!cmd || strcmp(cmd, "bomba") != 0) {
        ESP_LOGW("LORA >>>", "Comando ignorado. CMD nao eh 'bomba'");
        return false;
    }

    // 2) Verifica ID válido
    if (bomba_id == 0xFFFF || bomba_id == 0) {
        ESP_LOGE("LORA >>>", "Bomba ID invalido (%d)", bomba_id);
        return false;
    }

    // 3) Verifica valor válido
    if (valor != 0 && valor != 1) {
        ESP_LOGE("LORA >>>", "Valor invalido recebido: %d (esperado 0 ou 1)", valor);
        return false;
    }

    // 4) Inverte valor
    int valor_invertido = (valor == 1) ? 0 : 1;

    ESP_LOGI("LORA >>>", "Comando Bomba recebido -> bomba_id=%d, valor=%d, invertido=%d", bomba_id, valor,
             valor_invertido);

    int idx = bomba_index_from_id(bomba_id);
    if (idx < 0) {
        ESP_LOGE("LORA >>>", "bomba_id=%d nao pertence a este tanque", bomba_id);
        return false;
    }

    if (controle_id <= 0) {
        controle_id = (int)bomba_id;
    }
    g_bomba_controle_id[idx] = controle_id;

    if (existe_emergencia_ativa_global()) { 
        ESP_LOGW("LORA >>>", "Comando ignorado para bomba_id=%d: emergencia global ativa", bomba_id);

        if (valor_invertido) {
            enviar_alerta_bomba_api_online(idx,
                                           "FALHA AO LIGAR BOMBA",
                                           "emergencia global ativa");
        }

        g_controle_bomba_habilitado[idx] = false;
        g_status_bomba_desejado[idx] = false;
        return false;
    }

    g_controle_bomba_habilitado[idx] = true;
    g_status_bomba_desejado[idx] = (valor_invertido ? true : false);

    if (g_status_bomba[idx] == g_status_bomba_desejado[idx]) {
        lora_enqueue_json_bomba_int((uint16_t)bomba_id, "status", g_status_bomba[idx]);
    }

    ESP_LOGI("LORA >>>", "Bomba idx=%d setada para %d", idx, valor_invertido);
    pwm_atualizar_saida_por_estado();
    return true;
}

static void tanque_lora_app_task(void *pv) {
    lora_rx_app_item_t item;

    ESP_LOGI("LORA", "APP TANQUE iniciado");

    while (1) {
        if (xQueueReceive(lora_rx_app_queue, &item, portMAX_DELAY) == pdTRUE) {
            lora_app_frame_t *rx = &item.frame;

            switch (rx->msg_type) {
            case MSG_CMD:
                tanque_process_cmd_from_gtw(rx);
                break;

            case MSG_PING:
                ESP_LOGI("LORA", "PING recebido do GTW");
                break;

            case MSG_DATA: {
                char body[65] = {0};
                size_t n = rx->len;
                if (n > sizeof(rx->payload))
                    n = sizeof(rx->payload);
                if (n > 64)
                    n = 64;
                memcpy(body, rx->payload, n);
                body[n] = '\0';
                ESP_LOGI("LORA", "DATA recebido do GTW: %s", body);
                break;
            }

            default:
                ESP_LOGW("LORA", "Tipo nao tratado na APP: %u", rx->msg_type);
                break;
            }
        }
    }
}

void tanque_lora_rx_task(void *pv) {
    lora_app_frame_t rx;
    uint16_t my_id = tanque_lora_get_device_id();
    tanque_lora_rx_stats_t stats = {0};
    int64_t last_stats_log_ms = esp_timer_get_time() / 1000;

    ESP_LOGI("LORA", "RX TANQUE iniciado (ID=%u)", my_id);

    while (1) {
        memset(&rx, 0, sizeof(rx));
        my_id = tanque_lora_get_device_id();
        int r = 0;
        int64_t now_ms = esp_timer_get_time() / 1000;

        if ((now_ms - last_stats_log_ms) >= 60000) {
            tanque_lora_log_rx_stats(&stats, "periodico_60s");
            last_stats_log_ms = now_ms;
        }

        if (xSemaphoreTake(MutexLora, portMAX_DELAY) == pdTRUE) {
            r = lora_e32_receive_raw((uint8_t *)&rx, sizeof(rx), LORA_RX_READ_TIMEOUT_MS);
            xSemaphoreGive(MutexLora);
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (r <= 0) {
            stats.timeout++;
            if ((stats.timeout % 500U) == 0U) {
                tanque_lora_log_rx_stats(&stats, "timeout_500");
            }
            vTaskDelay(pdMS_TO_TICKS(LORA_RX_IDLE_DELAY_MS));
            continue;
        }

        stats.frame_ok++;
        ESP_LOGI("LORA", "RX frame bruto ok #%lu bytes=%d src_type=%u src_id=%u dst_type=%u dst_id=%u type=%u msg_id=%u len=%u",
                 (unsigned long)stats.frame_ok, r, rx.src_type, rx.src_id, rx.dst_type, rx.dst_id,
                 rx.msg_type, rx.msg_id, rx.len);

        size_t expected_air_len = lora_frame_air_len(&rx);
        if (expected_air_len == 0 || r != (int)expected_air_len) {
            stats.incomplete++;
            ESP_LOGW("LORA", "Frame incompleto recebido: %d esperado=%u",
                     r, (unsigned)expected_air_len);
            tanque_lora_log_rx_stats(&stats, "incomplete");
            continue;
        }

        if (rx.preamble != LORA_PREAMBLE) {
            stats.bad_preamble++;
            ESP_LOGW("LORA", "Preamble invalido");
            tanque_lora_log_rx_stats(&stats, "bad_preamble");
            continue;
        }

        uint16_t crc_calc = 0;
        uint16_t crc_rx = 0;
        if (!lora_received_frame_valid(&rx, r, &crc_calc, &crc_rx)) {
            stats.bad_crc++;
            ESP_LOGW("LORA", "CRC invalido msg_id=%u recebido=0x%04X calculado=0x%04X",
                     rx.msg_id, crc_rx, crc_calc);
            tanque_lora_log_rx_stats(&stats, "bad_crc");
            continue;
        }

        if (lora_is_duplicate(&rx)) {
            stats.duplicate++;
            ESP_LOGW("LORA", "Frame duplicado ignorado -> src=%u type=%u msg_id=%u",
                     rx.src_id, rx.msg_type, rx.msg_id);

            if (rx.dst_type == DEV_TANK && rx.dst_id == my_id &&
                (rx.msg_type == MSG_CMD || rx.msg_type == MSG_DATA || rx.msg_type == MSG_PING)) {
                ESP_LOGW("LORA", "Reenviando ACK para frame duplicado msg_id=%u", rx.msg_id);
                tanque_send_ack(&rx);
                stats.ack_sent_for_payload++;
            }

            tanque_lora_log_rx_stats(&stats, "duplicate");
            continue;
        }

        if (rx.dst_type != DEV_TANK || rx.dst_id != my_id) {
            stats.wrong_dst++;
            ESP_LOGW("LORA", "Destino invalido -> dst_type=%u dst_id=%u meu_id=%u",
                     rx.dst_type, rx.dst_id, my_id);

            if (should_relay_frame(&rx, my_id)) {
                ESP_LOGI("LORA", "RELAY -> src_type=%u src_id=%u dst_type=%u dst_id=%u msg_type=%u msg_id=%u",
                         rx.src_type, rx.src_id, rx.dst_type, rx.dst_id, rx.msg_type, rx.msg_id);

                if (xSemaphoreTake(MutexLora, pdMS_TO_TICKS(1000)) == pdTRUE) {
                    lora_send_frame_air(&rx);
                    xSemaphoreGive(MutexLora);
                } else {
                    ESP_LOGW("LORA", "Falha ao pegar MutexLora para relay");
                }
            }

            tanque_lora_log_rx_stats(&stats, "wrong_dst");
            continue;
        }

        if (rx.msg_type == MSG_ACK) {
            stats.ack_rx++;
            ESP_LOGI("LORA", "ACK bruto recebido -> src=%u msg_id=%u", rx.src_id, rx.msg_id);

            if (lora_ack_wait_match_and_signal(&rx)) {
                stats.ack_match++;
                ESP_LOGI("LORA", "ACK casado -> src=%u msg_id=%u", rx.src_id, rx.msg_id);
            } else {
                stats.ack_unexpected++;
                ESP_LOGW("LORA", "ACK inesperado -> src=%u msg_id=%u", rx.src_id, rx.msg_id);
            }
            continue;
        }

        lora_rx_app_item_t app_item = {0};
        memcpy(&app_item.frame, &rx, sizeof(rx));

        bool should_ack_payload = (rx.msg_type == MSG_CMD || rx.msg_type == MSG_DATA || rx.msg_type == MSG_PING);

        if (xQueueSend(lora_rx_app_queue, &app_item, 0) != pdTRUE) {
            stats.app_queue_drop++;
            ESP_LOGW("LORA", "Fila RX APP cheia, frame sem ACK para retry msg_id=%u", rx.msg_id);
            tanque_lora_log_rx_stats(&stats, "app_queue_drop");
        } else {
            stats.app_queue_ok++;

            if (should_ack_payload) {
                tanque_send_ack(&rx);
                stats.ack_sent_for_payload++;
            }
        }
    }
}

void tanque_lora_start(void) {
    static bool started = false;
    if (started)
        return;

    if (!g_lora_ack_mutex) {
        g_lora_ack_mutex = xSemaphoreCreateMutex();
        if (!g_lora_ack_mutex) {
            ESP_LOGE("LORA", "Falha ao criar g_lora_ack_mutex");
            return;
        }
    }

    if (!lora_tx_queue) {
        lora_tx_queue = xQueueCreate(LORA_TX_QUEUE_LEN, sizeof(tx_item_t));
        if (!lora_tx_queue) {
            ESP_LOGE("LORA", "Falha ao criar lora_tx_queue");
            return;
        }
    }

    if (!lora_rx_app_queue) {
        lora_rx_app_queue = xQueueCreate(LORA_RX_APP_QUEUE_LEN, sizeof(lora_rx_app_item_t));
        if (!lora_rx_app_queue) {
            ESP_LOGE("LORA", "Falha ao criar lora_rx_app_queue");
            return;
        }
    }

    memset(g_lora_dup_cache, 0, sizeof(g_lora_dup_cache));
    lora_ack_wait_cancel();

    ESP_LOGW("LORA", "Iniciando rádio E32.");
    lora_setup_tanque();
    ESP_LOGW("LORA", "Rádio OK, iniciando tasks TX/RX/APP.");

    xTaskCreate(tanque_lora_tx_task, "lora_tx", 4096, NULL, 5, NULL);
    xTaskCreate(tanque_lora_rx_task, "lora_rx", 8192, NULL, 5, NULL);
    xTaskCreate(tanque_lora_app_task, "lora_app", 6144, NULL, 5, &g_lora_app_task_handle);

    started = true;
}

static void lora_start_if_needed(void) { tanque_lora_start(); }

static void lora_boot_snapshot_task(void *pv)
{
    (void)pv;

    vTaskDelay(pdMS_TO_TICKS(3500));

    ESP_LOGW("LORA_BOOT", "Enviando snapshot inicial via LoRa");

    if (g_cfg.tanque_id > 0)
    {
        int nivel_pct = -1;
        if (ler_nivel_atual_pct(&nivel_pct))
        {
            bool ok = lora_enqueue_json_tanque_id_int((uint16_t)g_cfg.tanque_id,
                                                      "nivel_atual",
                                                      nivel_pct);
            ESP_LOGI("LORA_BOOT", "Snapshot nivel_atual=%d -> %s", nivel_pct, ok ? "OK" : "FAIL");
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        else
        {
            ESP_LOGW("LORA_BOOT", "Snapshot nivel_atual falhou: leitura indisponivel");
        }
    }

    if (g_cfg.perfil == TANQUE_FULL)
    {
        bool emergencia_ativa = existe_emergencia_ativa_global();
        if (emergencia_ativa)
        {
            bool ok = lora_enqueue_alerta_codigo(1, g_cfg.unidade_id, 0, -1);
            ESP_LOGW("LORA_BOOT", "Snapshot emergencia ativa codigo=1 -> %s", ok ? "OK" : "FAIL");
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    }

    ESP_LOGW("LORA_BOOT", "Snapshot inicial LoRa finalizado");
    vTaskDelete(NULL);
}

static bool lora_enqueue_json_bomba(uint16_t bomba_id, const char *json) {
    if (!lora_tx_queue || bomba_id == 0 || !json)
        return false;

    tx_item_t item = {.gtw_id = LORA_GTW_ID, .has_bomba = 1, .bomba_id = bomba_id};

    snprintf(item.msg, sizeof(item.msg), "%s", json);

    return (xQueueSend(lora_tx_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE);
}

static bool lora_enqueue_json_bomba_int(uint16_t bomba_id, const char *campo, int valor) {
    if (!campo || bomba_id == 0)
        return false;

    if (strcmp(campo, "status") == 0)
        campo = "s";
    else if (strcmp(campo, "automatico") == 0)
        campo = "m";
    else if (strcmp(campo, "vazao") == 0)
        campo = "v";

    char json[48];
    snprintf(json, sizeof(json), "{\"%s\":%d}", campo, valor);

    return lora_enqueue_json_bomba(bomba_id, json);
}

bool sincronizar_status_bomba_real(int idx) {
    if (idx < 0 || idx >= g_cfg.qtd_bombas || idx >= MAX_BOMBAS || g_cfg.bomba_id[idx] <= 0) {
        ESP_LOGW(TAG_BOMBA, "Sync status real ignorado: idx=%d qtd=%d bomba_id=%d",
                 idx, g_cfg.qtd_bombas, (idx >= 0 && idx < MAX_BOMBAS) ? g_cfg.bomba_id[idx] : -1);
        return false;
    }

    bool real_ligada = g_status_bomba[idx] ? true : false;
    EventBits_t bits = xEventGroupGetBits(sys_event_group);
    bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
    bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;

    if (net_ok && auth_ok) {
        if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE) {
            esp_err_t err = api_patch_bomba_status(g_cfg.bomba_id[idx], real_ligada);
            xSemaphoreGive(MutexHTTP);

            if (err == ESP_OK) {
                ESP_LOGI(TAG_BOMBA, "Sync status real via HTTP OK bomba_id=%d status=%d",
                         g_cfg.bomba_id[idx], real_ligada ? 1 : 0);
                return true;
            }

            ESP_LOGW(TAG_BOMBA, "Sync status real HTTP falhou (%s); tentando LoRa", esp_err_to_name(err));
        } else {
            ESP_LOGW(TAG_BOMBA, "Sync status real HTTP pulado: MutexHTTP ocupado; tentando LoRa");
        }
    } else {
        ESP_LOGW(TAG_BOMBA, "Sync status real sem internet/auth (net=%d auth=%d); enviando via LoRa",
                 net_ok ? 1 : 0, auth_ok ? 1 : 0);
    }

    bool ok = lora_enqueue_json_bomba_int((uint16_t)g_cfg.bomba_id[idx], "status", real_ligada ? 1 : 0);
    ESP_LOGW(TAG_BOMBA, "Sync status real via LoRa bomba_id=%d status=%d -> %s",
             g_cfg.bomba_id[idx], real_ligada ? 1 : 0, ok ? "OK" : "FAIL");
    return ok;
}

static uint8_t alerta_codigo_bomba(const char *evento, const char *motivo) {
    if (evento && strcmp(evento, "BOMBA PARADA") == 0) {
        return 2;
    }

    if (evento && strcmp(evento, "FALHA AO LIGAR BOMBA") == 0) {
        if (motivo && strstr(motivo, "emergencia") != NULL) {
            return 3;
        }
        return 4;
    }

    if (evento && strcmp(evento, "FALHA AO DESLIGAR BOMBA") == 0) {
        return 5;
    }

    return 0;
}

void bomba_registrar_controle_id(int idx, int controle_id) {
    if (idx < 0 || idx >= MAX_BOMBAS)
        return;

    if (controle_id > 0)
        g_bomba_controle_id[idx] = controle_id;
}

static bool alerta_bomba_reseta_controle(uint8_t codigo) {
    return codigo == 3 || codigo == 4 || codigo == 5;
}

bool enviar_alerta_bomba_api_online(int idx, const char *evento, const char *motivo) {
    if (idx < 0 || idx >= g_cfg.qtd_bombas || idx >= MAX_BOMBAS || g_cfg.bomba_id[idx] <= 0) {
        ESP_LOGW(TAG_BOMBA, "Alerta bomba API ignorado: idx=%d qtd=%d bomba_id=%d",
                 idx, g_cfg.qtd_bombas, (idx >= 0 && idx < MAX_BOMBAS) ? g_cfg.bomba_id[idx] : -1);
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(sys_event_group);
    bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
    bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;
    uint8_t codigo = alerta_codigo_bomba(evento, motivo);
    int controle_id = g_bomba_controle_id[idx] > 0 ? g_bomba_controle_id[idx] : g_cfg.bomba_id[idx];

    if (!net_ok || !auth_ok) {
        bool ok = (codigo != 0) &&
                  lora_enqueue_alerta_codigo_controle(codigo, g_cfg.unidade_id, g_cfg.bomba_id[idx], controle_id, -1);
        bool ctrl_ok = true;
        if (alerta_bomba_reseta_controle(codigo)) {
            ctrl_ok = lora_enqueue_bomba_controle_false(controle_id);
        }
        ESP_LOGW(TAG_BOMBA, "Alerta bomba via LoRa codigo=%u net=%d auth=%d bomba_id=%d controle_id=%d -> alerta=%s ctrl=%s",
                 codigo, net_ok ? 1 : 0, auth_ok ? 1 : 0, g_cfg.bomba_id[idx], controle_id,
                 ok ? "OK" : "FAIL", ctrl_ok ? "OK" : "FAIL");
        return ok && ctrl_ok;
    }

    const char *tanque_nome = g_cfg.nome_tanque[0] ? g_cfg.nome_tanque : NOME_CURTO;
    const char *motivo_msg = motivo ? motivo : "motivo nao informado";

    char msg[160];
    if (evento && strcmp(evento, "BOMBA PARADA") == 0) {
        snprintf(msg, sizeof(msg),
                 "ALERTA BOMBA PARADA: Tanque %s, Bomba %d, %s",
                 tanque_nome,
                 g_cfg.bomba_id[idx],
                 motivo_msg);
    } else {
        snprintf(msg, sizeof(msg),
                 "ALERTA BOMBA: Tanque %s, Bomba %d, %s, %s",
                 tanque_nome,
                 g_cfg.bomba_id[idx],
                 evento ? evento : "evento",
                 motivo_msg);
    }

    esp_err_t err = ESP_ERR_TIMEOUT;
    esp_err_t ctrl_err = ESP_OK;
    if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE) {
        err = api_post_tanque(msg, g_cfg.unidade_id);
        if (err == ESP_OK && alerta_bomba_reseta_controle(codigo)) {
            ctrl_err = api_patch_bomba_controle_false(controle_id);
        }
        xSemaphoreGive(MutexHTTP);
    } else {
        if (codigo != 0 && lora_enqueue_alerta_codigo_controle(codigo, g_cfg.unidade_id, g_cfg.bomba_id[idx], controle_id, -1)) {
            bool ctrl_ok = true;
            if (alerta_bomba_reseta_controle(codigo)) {
                ctrl_ok = lora_enqueue_bomba_controle_false(controle_id);
            }
            ESP_LOGW(TAG_BOMBA, "MutexHTTP ocupado; alerta bomba enviado via LoRa codigo=%u bomba_id=%d controle_id=%d ctrl=%s",
                     codigo, g_cfg.bomba_id[idx], controle_id, ctrl_ok ? "OK" : "FAIL");
            return ctrl_ok;
        }
        ESP_LOGW(TAG_BOMBA, "Alerta bomba API nao enviado: MutexHTTP ocupado");
        return false;
    }

    if (err == ESP_OK && ctrl_err == ESP_OK) {
        ESP_LOGW(TAG_BOMBA, "Alerta bomba API enviado: %s", msg);
        return true;
    }

    if (err == ESP_OK && ctrl_err != ESP_OK) {
        bool ctrl_ok = lora_enqueue_bomba_controle_false(controle_id);
        ESP_LOGW(TAG_BOMBA, "Alerta API OK, mas controle=false falhou (%s); enviado via LoRa controle_id=%d -> %s",
                 esp_err_to_name(ctrl_err), controle_id, ctrl_ok ? "OK" : "FAIL");
        return ctrl_ok;
    }

    if (codigo != 0 && lora_enqueue_alerta_codigo_controle(codigo, g_cfg.unidade_id, g_cfg.bomba_id[idx], controle_id, -1)) {
        bool ctrl_ok = true;
        if (alerta_bomba_reseta_controle(codigo)) {
            ctrl_ok = lora_enqueue_bomba_controle_false(controle_id);
        }
        ESP_LOGW(TAG_BOMBA, "Falha API; alerta bomba enviado via LoRa codigo=%u bomba_id=%d controle_id=%d ctrl=%s",
                 codigo, g_cfg.bomba_id[idx], controle_id, ctrl_ok ? "OK" : "FAIL");
        return ctrl_ok;
    }

    ESP_LOGW(TAG_BOMBA, "Falha alerta bomba API (%s) ctrl=%s: %s",
             esp_err_to_name(err), esp_err_to_name(ctrl_err), msg);
    return false;
}

static bool lora_enqueue_json_tanque_id_int(uint16_t tanque_id, const char *campo, int valor) {
    if (!lora_tx_queue || !campo || tanque_id == 0)
        return false;

    tx_item_t item = {.gtw_id = LORA_GTW_ID, .src_tank_id = tanque_id, .has_bomba = 0, .bomba_id = 0};

    if (strcmp(campo, "nivel_atual") == 0)
        campo = "n";

    snprintf(item.msg, sizeof(item.msg), "{\"%s\":%d}", campo, valor);

    return (xQueueSend(lora_tx_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE);
}

static __attribute__((unused)) bool lora_enqueue_alerta_api(const char *mensagem, int unidade_id) {
    if (!lora_tx_queue || !mensagem)
        return false;

    tx_item_t item = {.gtw_id = LORA_GTW_ID, .has_bomba = 2, .bomba_id = 0};

    int n = snprintf(item.msg, sizeof(item.msg), "{\"mensagem\":\"%s\",\"unidade\":%d}", mensagem, unidade_id);

    if (n <= 0 || n >= (int)sizeof(item.msg)) {
        // fallback ultra-curto (caso alguém coloque texto grande sem querer)
        n = snprintf(item.msg, sizeof(item.msg), "{\"mensagem\":\"AL\",\"unidade\":%d}", unidade_id);
        if (n <= 0 || n >= (int)sizeof(item.msg))
            return false;
    }

    return (xQueueSend(lora_tx_queue, &item, pdMS_TO_TICKS(200)) == pdTRUE);
}

static bool lora_enqueue_alerta_codigo(uint8_t codigo, int unidade_id, int bomba_id, int pct) {
    return lora_enqueue_alerta_codigo_controle(codigo, unidade_id, bomba_id, 0, pct);
}

static bool lora_enqueue_alerta_codigo_controle(uint8_t codigo, int unidade_id, int bomba_id, int controle_id, int pct) {
    if (!lora_tx_queue || codigo == 0)
        return false;

    tx_item_t item = {.gtw_id = LORA_GTW_ID, .has_bomba = 2, .bomba_id = 0};
    int n;

    if (pct >= 0 && bomba_id > 0 && controle_id > 0) {
        n = snprintf(item.msg, sizeof(item.msg), "{\"a\":%u,\"u\":%d,\"b\":%d,\"c\":%d,\"p\":%d}",
                     (unsigned)codigo, unidade_id, bomba_id, controle_id, pct);
    } else if (pct >= 0 && bomba_id > 0) {
        n = snprintf(item.msg, sizeof(item.msg), "{\"a\":%u,\"u\":%d,\"b\":%d,\"p\":%d}",
                     (unsigned)codigo, unidade_id, bomba_id, pct);
    } else if (bomba_id > 0 && controle_id > 0) {
        n = snprintf(item.msg, sizeof(item.msg), "{\"a\":%u,\"u\":%d,\"b\":%d,\"c\":%d}",
                     (unsigned)codigo, unidade_id, bomba_id, controle_id);
    } else if (pct >= 0) {
        n = snprintf(item.msg, sizeof(item.msg), "{\"a\":%u,\"u\":%d,\"p\":%d}",
                     (unsigned)codigo, unidade_id, pct);
    } else if (bomba_id > 0) {
        n = snprintf(item.msg, sizeof(item.msg), "{\"a\":%u,\"u\":%d,\"b\":%d}",
                     (unsigned)codigo, unidade_id, bomba_id);
    } else {
        n = snprintf(item.msg, sizeof(item.msg), "{\"a\":%u,\"u\":%d}",
                     (unsigned)codigo, unidade_id);
    }

    if (n <= 0 || n >= (int)sizeof(item.msg))
        return false;

    return (xQueueSend(lora_tx_queue, &item, pdMS_TO_TICKS(200)) == pdTRUE);
}

static bool lora_enqueue_bomba_controle_false(int controle_id) {
    if (!lora_tx_queue || controle_id <= 0)
        return false;

    tx_item_t item = {.gtw_id = LORA_GTW_ID, .has_bomba = 3, .bomba_id = (uint16_t)controle_id};
    int n = snprintf(item.msg, sizeof(item.msg), "{\"c\":%d}", controle_id);

    if (n <= 0 || n >= (int)sizeof(item.msg))
        return false;

    return (xQueueSend(lora_tx_queue, &item, pdMS_TO_TICKS(200)) == pdTRUE);
}

//=========================================================================================================================

static bool existe_bomba_ligada_ou_desejada(void)
{
    for (int i = 0; i < g_cfg.qtd_bombas && i < MAX_BOMBAS; i++)
    {
        if (g_status_bomba[i] || g_status_bomba_desejado[i])
            return true;
    }
    return false;
}

void vazao_sync_task(void *pv)
{
    static const char *TAGV = "VAZAO_SYNC";
    static int last_sent_http = -1;
    static int last_sent_lora = -1;

    while (1)
    {
        if (!g_vazao_pwm_sync_pendente)
        {
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        int pct = g_vazao_pwm_sync_pct;

        EventBits_t bits = xEventGroupGetBits(sys_event_group);
        bool net_ok = (bits & SYS_NET_ONLINE_BIT) != 0;
        bool auth_ok = (bits & SYS_AUTH_OK_BIT) != 0;
        bool pode_http = net_ok && auth_ok;

        if (!pode_http)
        {
            if (pct != last_sent_lora)
            {
                bool ok = lora_enqueue_json_bomba_int((uint16_t)BOMBA_PWM_ID,
                                                      "vazao",
                                                      pct);
                if (ok)
                {
                    last_sent_lora = pct;
                    g_vazao_pwm_sync_pendente = false;

                    ESP_LOGI(TAGV, "Vazao enviada via LoRa -> %d%%", pct);
                }
                else
                {
                    ESP_LOGW(TAGV, "Falha ao enfileirar vazao via LoRa");
                }
            }
            else
            {
                g_vazao_pwm_sync_pendente = false;
            }

            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (pct == last_sent_http)
        {
            g_vazao_pwm_sync_pendente = false;
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        if (xSemaphoreTake(MutexHTTP, pdMS_TO_TICKS(3000)) == pdTRUE)
        {
            esp_err_t err = ESP_FAIL;
            cJSON *patch = cJSON_CreateObject();

            if (patch)
            {
                cJSON_AddNumberToObject(patch, "vazao", pct);
                err = api_patch_bomba(BOMBA_PWM_ID, patch);
                cJSON_Delete(patch);
            }

            xSemaphoreGive(MutexHTTP);

            if (err == ESP_OK)
            {
                last_sent_http = pct;
                g_vazao_pwm_sync_pendente = false;

                ESP_LOGI(TAGV, "PATCH vazao OK -> %d%%", pct);
            }
            else
            {
                ESP_LOGW(TAGV, "Falha PATCH vazao (%s)", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void pwm_salvar_setpoint_percent(float percent)
{
    g_vazao_pwm_percent = clampf(percent, 0.0f, 100.0f);

    ESP_LOGI("PWM_4A20",
             "Setpoint salvo = %.2f%%",
             (double)g_vazao_pwm_percent);
}

float pwm_obter_setpoint_percent(void)
{
    return g_vazao_pwm_percent;
}

void pwm_atualizar_saida_por_estado(void)
{
    static float ultimo_pct_aplicado = -1.0f;
    static bool ultima_saida_ativa = false;

    bool saida_ativa = false;

    if (g_cfg.qtd_bombas > 0)
        saida_ativa = existe_bomba_ligada_ou_desejada();

    float alvo_pct = saida_ativa ? g_vazao_pwm_percent : 0.0f;

    if ((ultima_saida_ativa == saida_ativa) &&
        (fabsf(alvo_pct - ultimo_pct_aplicado) < 0.01f))
    {
        return;
    }

    set_4a20_from_percent(alvo_pct);

    ultimo_pct_aplicado = alvo_pct;
    ultima_saida_ativa = saida_ativa;

    ESP_LOGI("PWM_4A20",
             "Saida %s | Setpoint salvo=%.2f%% | Aplicado=%.2f%%",
             saida_ativa ? "ATIVA" : "EM REPOUSO",
             (double)g_vazao_pwm_percent,
             (double)alvo_pct);
}

void pwm_processar_novo_setpoint(float percent)
{
    pwm_salvar_setpoint_percent(percent);
    pwm_atualizar_saida_por_estado();
}

void pwm_agendar_sync_vazao(int bomba_id, float percent)
{
    if (bomba_id != BOMBA_PWM_ID)
        return;

    int pct = (int)(clampf(percent, 0.0f, 100.0f) + 0.5f);

    g_vazao_pwm_sync_pct = pct;
    g_vazao_pwm_sync_pendente = true;

    ESP_LOGI("PWM_4A20", "Vazao agendada para sync: bomba=%d valor=%d%%",
             bomba_id, pct);
}

void set_4a20_from_percent(float percent)
{
    if (percent < 0.0f)
        percent = 0.0f;
    if (percent > 100.0f)
        percent = 100.0f;

    // Corrente ideal
    float current_ma = 4.0f + (percent * 16.0f / 100.0f);

    // Compensação
    current_ma = (current_ma * CAL_GAIN) + CAL_OFFSET;

    // Limites físicos
    if (current_ma < 4.0f)
        current_ma = 4.0f;
    if (current_ma > 20.0f)
        current_ma = 20.0f;

    // Converte de volta para percentual efetivo
    float percent_corrigido = ((current_ma - 4.0f) / 16.0f) * 100.0f;

    uint32_t max_duty = (1 << 10) - 1; // 1023
    uint32_t duty = (uint32_t)((percent_corrigido / 100.0f) * max_duty);

    ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty);
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);

    ESP_LOGI("PWM_4A20", "Req=%.2f%% | Corr=%.2f mA | Duty=%lu", percent, current_ma, duty);
}
