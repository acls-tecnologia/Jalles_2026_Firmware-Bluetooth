#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cJSON.h"
#include "esp_netif.h"
#include "esp_websocket_client.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <math.h>

#include "driver/ledc.h"

#include "ads1115.h"
#include "lora.h"
#include "mcp23017.h"
#include "wifi.h"

#include "driver/uart.h"

#include "api_client.h"
#include "watchdog_rtc_raw.h"
#include "ws_client.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>

#include "esp_task_wdt.h"
#include "esp_timer.h"

//============== CONFIGURAÇÕES GERAIS DO PROJETO ==============

#define DEBUG_MODE 0 // ou 1 para ativar logs

//============== Definições da API ==============

// Tanques e Ids
/*
=== Tanques Jalles ===
-> R-0 (Dois Niveis)Nivel 1  ID = 1
-> R-0 (Dois Niveis)Nivel 2  ID = 21
-> R-01 (uma bomba) ID = 22
-> R-02A (uma bomba) ID = 23
-> R-04 (uma bomba) ID = 24
-> R-11 (uma bomba) ID = 29
-> Capitação Revolta (Duas bombas) ID = 41
-> RA-09 (só Nivel) ID = 61
-> RA-10 (Uma Bomba) ID = 62
-> RD-0 (Tres Bombas, Bomba 1 modular ID = 42 ) ID = 63
-> RD-02 (Só Nível ) ID = 81
-> RD-06 (Só Nível ) ID = 82
-> RD-09 (Uma Bomba ) ID = 83
-> Booster (Uma Bomba sem Nivel Bomba 1 modular ID = 62 ) ID = 84


==== Tanques Uol
-> R-02 (Só Nível) ID = 25
-> R-03 (Só Nível) ID = 26
-> R-05 (Uma Bomba) ID = 27
-> Etar (duas Bomba) ID = 28

*/

//=========================================================================================

//====================== CONFIGURAÇÕES PRINCIPAIS DO SISTEMA ==============================

//=========================================================================================

#define API_USER_MAX_LEN 32
#define API_PASS_MAX_LEN 48

extern char g_api_user[API_USER_MAX_LEN];
extern char g_api_pass[API_PASS_MAX_LEN];
extern int g_provision_tanque_id;
extern int g_lora_gtw_id;

#define API_USER g_api_user
#define API_PASS g_api_pass
#define DEVICE_ID g_api_user

// static const char API_PASS[] = "hER49}W2:ql~£P94Y9WN"; // Senha do dispositivo na API
#define NOME_CURTO ((g_cfg.nome_tanque[0] != '\0') ? g_cfg.nome_tanque : g_api_user)

#define PROVISION_TANQUE_ID g_provision_tanque_id // ID do Tanque

#define BOMBA_PWM_ID 42 // ID da bomba para ser modulada

#define LORA_GTW_ID g_lora_gtw_id // ID Gateway configuravel
// #define LORA_GTW_ID 2 // ID Gateway UOL
#define LORA_REQUIRE_ACK 1 // se true, o tanque espera ACK da GTW e reenvia se não receber

#define CORRENTE_TC_B1 200.0f // <<<<<<<<<<<< escala do TC bomba 1
#define CORRENTE_TC_B2 200.0f // <<<<<<<<<<<< escala do TC bomba 2
#define CORRENTE_TC_B3 200.0f // <<<<<<<<<<<< escala do TC bomba 3

// Tanque Especial com 2 níveis
#define TANQUE_ESPECIAL_2NIVEIS_ENABLE 1
#define TANQUE_ESPECIAL_ID 1
#define TANQUE_ESPECIAL_NIVEL2_ID 21

//=========================================================================================

//=========================================================================================

//=========================================================================================

// reaproveita o canal que hoje seria a corrente da bomba 3
#define lerNivel2 lerCorrente3

//==============================================

extern const char *rootCaCerticate;

//============== Estruturas de configuração ==============
#define MAX_BOMBAS 3 // <<<<<<<<<<<<<<<<< Limite de Bombas por Tanque

typedef enum {
    TANQUE_NIVEL = 0, // Apenas leitura de nível
    TANQUE_FULL = 1   // Nível + portas + comandos
} perfil_tanque_t;    // tipo de perfil do tanque

typedef struct {
    bool provisionado;      // true quando já salvou NVS
    perfil_tanque_t perfil; // TANQUE_NIVEL ou TANQUE_FULL

    int unidade_id; // ID da unidade na API
    int tanque_id;  // ID do tanque na API

    uint8_t qtd_bombas;       // quantidade de bombas (se perfil FULL), 0 se NIVEL
    int bomba_id[MAX_BOMBAS]; // IDs das bombas (se perfil FULL), 0 se não tiver ou não for FULL

    bool ativo;
    int automatico;
    int nivel_maximo;
    int nivel_minimo;

    char nome_tanque[16];             // "R-04", etc (opcional, mas ajuda)
    char device_id[API_USER_MAX_LEN]; // mesmo valor usado como usuario da API
    char api_user[API_USER_MAX_LEN];  // usuario para login da API
    char api_pass[API_PASS_MAX_LEN];  // senha para login da API
    int lora_gtw_id;                  // gateway LoRa destino
} device_cfg_t;

typedef struct {
    bool active;
    uint8_t msg_id;
    uint8_t expected_src_type;
    uint16_t expected_src_id;
    TaskHandle_t waiter;
} lora_ack_wait_t;

typedef struct {
    bool used;
    uint8_t src_type;
    uint16_t src_id;
    uint8_t msg_type;
    uint8_t msg_id;
    TickType_t seen_at;
} lora_dup_entry_t;

#define CFG_PROVISION_INTERVAL_MS (24ULL * 60ULL * 60ULL * 1000ULL) // 24 horas

extern volatile int g_vazao_pwm_sync_pct;
extern volatile bool g_vazao_pwm_sync_pendente;

void pwm_agendar_sync_vazao(int bomba_id, float percent);

extern device_cfg_t g_cfg; // configuração global do dispositivo

extern volatile int g_local_remoto[MAX_BOMBAS];           // 0=Local, 1=Remoto
extern volatile int g_emergencia[MAX_BOMBAS];             // espelha a emergência única do tanque em todas as posições
extern volatile bool g_status_bomba[MAX_BOMBAS];          // true = Ligada, false = Desligada
extern volatile bool g_status_bomba_desejado[MAX_BOMBAS]; // true = Ligada, false = Desligada
extern volatile bool g_controle_bomba_habilitado[MAX_BOMBAS]; // servidor liberou receber comandos?

extern volatile bool g_comando_bomba_desejado; // último comando recebido (liga/desliga)

extern volatile float g_vazao_pwm_percent; // último setpoint salvo (0..100%)

void set_4a20_from_percent(float percent);

void pwm_salvar_setpoint_percent(float percent);
float pwm_obter_setpoint_percent(void);
void pwm_atualizar_saida_por_estado(void);
void pwm_processar_novo_setpoint(float percent);
bool sincronizar_status_bomba_real(int idx);
bool enviar_alerta_bomba_api_online(int idx, const char *evento, const char *motivo);
void bomba_registrar_controle_id(int idx, int controle_id);

//============================================================

//============== Funções de configuração (config.c) ==============

esp_err_t cfg_nvs_init(void);                // inicializa NVS para configuração
bool cfg_load(device_cfg_t *out);            // carrega configuração da NVS
esp_err_t cfg_save(const device_cfg_t *cfg); /// salva configuração na NVS
void cfg_apply_runtime(const device_cfg_t *cfg);
bool cfg_wifi_load(void);
esp_err_t cfg_wifi_save(const char *ssid, const char *password);
void cfg_guess_nome_tanque_from_user(char *out, size_t out_len); // deriva nome do tanque a partir do usuário API_USER
void cfg_log(const device_cfg_t *cfg);                           // loga configuração atual
esp_err_t cfg_erase(void);                                       // apaga configuração da NVS
bool cfg_important_equal(const device_cfg_t *a, const device_cfg_t *b,
                         bool *need_restart); // compara cfgs e diz se são "importantemente iguais" (ou seja, não
                                              // precisa reiniciar) e se precisa reiniciar
//==============================================================

//============== Definições gerais do sistema ==============
#define API_DEBUG_PRINT_TOKEN 1   // 1 para printar token no login (só pra debug)
#define MAX_RETRIES_CONFIG 3      // número máximo de tentativas para configurar dispositivos I2C
#define I2C_SDA 4                 // GPIO para SDA do I2C
#define I2C_SCL 15                // GPIO para SCL do I2C
#define I2C_MASTER_FREQ_HZ 200000 // Frequência do barramento I2C

#define WIFI_CONNECT_TIMEOUT_MS 8000     // Tempo limite para conexão Wi-Fi
#define WIFI_CHECK_PERIOD_MS 15000       // Período entre checagens de internet
#define WIFI_MAX_CONNECT_FAILS 10        // falhas seguidas pra reiniciar Wi-Fi
#define WIFI_MAX_INTERNET_FAILS 5        // falhas seguidas de internet pra reiniciar Wi-Fi
#define WIFI_MAX_CYCLES_BEFORE_REBOOT 10 // quantas “rodadas de restart Wi-Fi” antes de reboot
#define WIFI_BACKOFF_BASE_MS 1000        // tempo base para backoff exponencial
#define WIFI_BACKOFF_MAX_MS 30000        // tempo máximo de backoff

// BOOT: tentativa rápida até entrar online pela primeira vez
#define WIFI_BOOT_RETRY_INTERVAL_MS 5000     // tenta a cada 5s no boot
#define INTERNET_BOOT_CHECK_INTERVAL_MS 3000 // check de internet mais rápido no boot

// RUNTIME: depois de já estar operando
#define WIFI_RUNTIME_RETRY_INTERVAL_MS 10000 // tenta recuperar por aproximadamente 1 minuto
#define INTERNET_CHECK_INTERVAL_MS 60000     // check normal de internet
#define INTERNET_FAILS_TO_OFFLINE 3          // 3 falhas seguidas => offline lógico

// Wi-Fi é opcional no tanque: após 5 falhas, desliga por 15 minutos.
#define WIFI_CONNECT_ATTEMPTS_BEFORE_SLEEP 5
#define WIFI_DISABLED_RETRY_MS (30ULL * 60ULL * 1000ULL)

// Health check TCP
#define INTERNET_HEALTH_URL "Google.com:80" // URL para checagem de internet (TCP)
#define INTERNET_TIMEOUT_MS 3000

#define WIFI_TASK_TICK_MS 5000
#define WIFI_RECONNECT_INTERVAL_MS 120000 // 2 minutos

//============== Definições do sistema de nível ==============
#define LEVEL_TAG "NIVEL"                                // tag de log do sistema de nível
#define lerNivel ADS1115_MUX_3_GND                       // Pino 4.1 - Nível do tanque
#define NIVEL_TASK_PERIOD_MS 2000                        // lê a cada 1s
#define NIVEL_DEADBAND_PCT 3                             // envia se variar >= 3%
#define NIVEL_PERIODIC_SEND_MS (15ULL * 60ULL * 1000ULL) // reenvio periodico mesmo sem variacao
#define NIVEL_PERIODIC_SLOT_MS 10000ULL                  // espalha tanques diferentes no tempo em campo longo
#define NIVEL_PERIODIC_JITTER_MS 120000ULL               // evita rajada se todos reiniciarem juntos

#define BOMBA_STATUS_PERIODIC_SEND_MS (15ULL * 60ULL * 1000ULL) // reenvio periodico do status real
#define BOMBA_STATUS_PERIODIC_SLOT_MS 10000ULL                  // espalha tanques diferentes no tempo
#define BOMBA_STATUS_PERIODIC_JITTER_MS 120000ULL               // evita rajada se todos reiniciarem juntos

//===========================================================

#define SHUNT_RES_OHMS 100.0f // resistor shunt de 100 ohms para medir corrente

//============== Definições do sistema de corrente ==============
#define lerCorrente1 ADS1115_MUX_2_GND // bomba 1
#define lerCorrente2 ADS1115_MUX_1_GND // bomba 2
#define lerCorrente3 ADS1115_MUX_0_GND // bomba 3

#define CORRENTE_TASK_PERIOD_MS 2000 // lê a cada 2s
#define CORRENTE_DEADBAND_A 1.0f     // só envia se variar >= 1A
#define CORRENTE_SEND_MIN_MS 5000    // intervalo mínimo entre envios

// Escala do TC / transmissor de corrente em ampères
// Ex.: se o transmissor foi configurado para 0..100A, deixe 100.0f
static const float bomba_tc_max_a[MAX_BOMBAS] = {
    CORRENTE_TC_B1, // bomba 1
    CORRENTE_TC_B2, // bomba 2
    CORRENTE_TC_B3  // bomba 3
};

// Canal ADS1115 usado para corrente de cada bomba
static const ads1115_mux_t bomba_corrente_mux[MAX_BOMBAS] = {lerCorrente1, lerCorrente2, lerCorrente3};

static inline bool tanque_especial_2niveis_ativo(void) {
    return (TANQUE_ESPECIAL_2NIVEIS_ENABLE == 1) && (g_cfg.tanque_id == TANQUE_ESPECIAL_ID) && (g_cfg.qtd_bombas == 0);
}

// ======== Definições do sistema de eventos ==============
extern EventGroupHandle_t sys_event_group; // event group global do sistema
#define SYS_NET_ONLINE_BIT BIT0            // Wi-Fi + internet OK
#define SYS_AUTH_OK_BIT BIT1               // logado (token válido)
#define SYS_WS_OK_BIT BIT2                 // websocket conectado
#define EVT_PORTAS_LIDAS_BIT BIT3          // portas lidas com sucesso
#define EVT_EMERGENCIA_BIT BIT4            // emergência acionada
#define EVT_LOCAL_REMOTO_BIT BIT5          // modo local/remoto alterado
#define EVT_STATUS_BOMBA_BIT BIT6          // status da bomba alterado

/* ======================= PWM ======================= */
#define PWM_GPIO 26
#define PWM_FREQ_HZ 1000
#define PWM_RES LEDC_TIMER_10_BIT // 0–1023
#define PWM_CHANNEL LEDC_CHANNEL_0
#define PWM_TIMER LEDC_TIMER_0
#define PWM_MODE LEDC_HIGH_SPEED_MODE

#define CAL_GAIN 1.0165f
#define CAL_OFFSET 0.0f // em mA, ajuste se necessário depois

// ===== Definições dos pinos das entradas digitais =====
// Bomba 1
#define LocalRemoto1_GPB GPB_PIN_4 // Pino 5.1 - Entrada Local/Remoto Bomba 1
// Bomba 2
#define LocalRemoto2_GPB GPB_PIN_5 // Pino 5.2 - Entrada Local/Remoto Bomba 2
// Bomba 3
#define LocalRemoto3_GPB GPB_PIN_6 // Pino 5.3 - Entrada Local/Remoto Bomba 3

#define Emergencia_GPB GPB_PIN_0 // Pino 5.4 - Entrada Emergencia Bombas

#define StatusBomba_GPB GPB_PIN_1  // Pino 6.1 - Entrada Status Bomba 1
#define StatusBomba2_GPB GPB_PIN_2 // Pino 6.3 - Entrada Status Bomba 2 (se tiver)
#define StatusBomba3_GPB GPB_PIN_3 // Pino 6.4 - Entrada Status Bomba 3 (se tiver)

// para facilitar leitura por índice (0..qtd_bombas-1)
static const uint8_t bomba_status_pin[MAX_BOMBAS] = {StatusBomba_GPB, StatusBomba2_GPB, StatusBomba3_GPB};

//======================================================

// ---- tabela dinâmica por índice (0..qtd_bombas-1) ----
typedef struct {
    int port;    // GPA ou GPB
    uint8_t pin; // GPA_PIN_x / GPB_PIN_x
} mcp_pin_ref_t;

static const mcp_pin_ref_t bomba_lr_ref[MAX_BOMBAS] = {
    {GPB, LocalRemoto1_GPB},
    {GPB, LocalRemoto2_GPB},
    {GPB, LocalRemoto3_GPB},

};

static const mcp_pin_ref_t bomba_em_ref[MAX_BOMBAS] = {
    {GPB, Emergencia_GPB},

};

// ===== Definições dos pinos das saídas digitais =====
#define BOMBA1_GPA GPA_PIN_5 // Pino 7.1  - Saída para controle da bomba 1 (liga/desliga)
#define BOMBA2_GPA GPA_PIN_4 // Pino 7.2  - Saída para controle da bomba 2 (liga/desliga) - se tiver
#define BOMBA3_GPA GPA_PIN_2 // Pino 7.3  - Saída para controle da bomba 3 (liga/desliga) - se tiver

static const uint8_t bomba_out_pin[MAX_BOMBAS] = {
    BOMBA1_GPA,
    BOMBA2_GPA,
    BOMBA3_GPA,
};

//=====================================================

// ===== Globais (somente DECLARAÇÃO) =====
extern size_t heap_total;

extern i2c_master_bus_handle_t bus_handle;
extern i2c_master_dev_handle_t mcp_handle;
extern i2c_master_dev_handle_t ads_handle;

extern ads1115_t _4a20ma;

extern SemaphoreHandle_t i2c_semaphore;
extern SemaphoreHandle_t MutexHTTP;
extern SemaphoreHandle_t MutexLora;

// Wi-Fi (para testes agora)
extern char wifi_ssid[33];
extern char wifi_password[65];

// Cache de limites dos tanques (min/max %) para evitar leituras frequentes
typedef struct {
    int min_pct; // 0..100
    int max_pct; // 0..100
    TickType_t last_ms;
    bool valid;
} tanque_limits_cache_t;

// API Client

#define API_URL "https://83cd1aa837661fab941b2c5a2a65424b.jm.net.br:2087/67ZlfPVt"
// #define API_URL "https://jalles.aclsconnect.com/67ZlfPVt"

#define API_LOGIN_URL API_URL "/users/login/"             // URL de login
#define API_ALERTA_TANQUE_URL API_URL "/alertas/tanque/"  // URL de alertas do tanque
#define API_DADOS_TANQUE_URL API_URL "/tanque/"           // URL de dados do tanque
#define API_TANQUE_LEITURA_URL API_URL "/tanque/Leitura/" // URL de leitura do tanque
#define API_BOMBA_URL API_URL "/bomba/"                   // URL de controle da bomba
#define API_BOMBA_CONTROLE_URL API_URL "/bomba/controle/" // URL do comando/controle da bomba

#define ENTRADA_ATIVA_EM_0                                                                                             \
    0 // se o botão em condição normal fica em 0 e ao apertar sobe para 1, então use ENTRADA_ATIVA_EM_1
#define ENTRADA_ATIVA_EM_1                                                                                             \
    1 // se o botão em condição normal fica energizado e ao apertar cai para 0, então use ENTRADA_ATIVA_EM_0

// Emergência:
// se o botão em condição normal fica energizado e ao apertar cai para 0,
// então use ENTRADA_ATIVA_EM_0
#define EMERGENCIA_ACIONADA_LEVEL ENTRADA_ATIVA_EM_1

// (local/remoto)
#define REMOTO_ATIVO_LEVEL ENTRADA_ATIVA_EM_1 // <<<<<< ajustar conforme o tipo de entrada de local/remoto (0 ou 1)
#define LOCAL_ATIVO_LEVEL ENTRADA_ATIVA_EM_0  // <<<<<<< ajustar conforme o tipo de entrada de local/remoto (0 ou 1)

static inline bool entrada_emergencia_acionada(int leitura) {
    return (leitura >= 0 && leitura == EMERGENCIA_ACIONADA_LEVEL);
}

static inline bool entrada_remoto_ativo(int leitura) { return (leitura >= 0 && leitura == REMOTO_ATIVO_LEVEL); }

// WebSocket base URL
// #define API_WS_BASE_URL "wss://jalles.aclsconnect.com/ws-native/native-ws" // URL base do WebSocket
#define API_WS_BASE_URL "wss://83cd1aa837661fab941b2c5a2a65424b.jm.net.br:2087/ws-native/native-ws" // URL base do
// WebSocket

// Watchdog
extern esp_timer_handle_t watchdog_timer;

// ========================== extruturas do lora

#define LORA_DUP_CACHE_SIZE 32
#define LORA_TX_QUEUE_LEN 32

extern lora_ack_wait_t g_lora_ack_wait;
extern lora_dup_entry_t g_lora_dup_cache[LORA_DUP_CACHE_SIZE];
extern TaskHandle_t g_lora_app_task_handle;

extern volatile int msg_counter;
extern QueueHandle_t lora_tx_queue;
extern QueueHandle_t lora_rx_app_queue;

extern SemaphoreHandle_t g_lora_ack_mutex;

#define LORA_RX_APP_QUEUE_LEN 24

#define LORA_DUP_WINDOW_MS 15000
#define LORA_RX_READ_TIMEOUT_MS 800
#define LORA_RX_IDLE_DELAY_MS 50
#define LORA_ACK_SEND_TIMEOUT_MS 2500
#define LORA_POST_TX_GUARD_MS 120

#define GPIO_OUTPUT_IO_22 22
#define LORA_ACK_TIMEOUT_MS 15000
#define LORA_MAX_RETRIES 5
#define LORA_RETRY_BACKOFF_MIN_MS 2500
#define LORA_RETRY_BACKOFF_JITTER_MS 7000

#define DEVICE_TYPE DEV_TANK

#define CMD_DATA 0x02
#define CMD_ACK 0x03

#define MY_ID g_cfg.tanque_id

#define LORA_PREAMBLE 0xAA

typedef enum { DEV_GTW = 0x01, DEV_TANK = 0x02, DEV_REP = 0x03 } device_type_t;

typedef enum { MSG_DATA = 0x01, MSG_CMD = 0x02, MSG_ACK = 0x03, MSG_PING = 0x04 } msg_type_t;

typedef struct __attribute__((packed)) {
    uint8_t preamble; // 0xAA
    uint8_t src_type; // tanque, gtw, repetidor
    uint16_t src_id;  // id de quem envia
    uint8_t dst_type; // destino
    uint16_t dst_id;  // id destino
    uint8_t msg_type; // DATA, CMD, ACK
    uint8_t msg_id;   // contador
    uint8_t len;      // tamanho payload

    uint8_t has_bomba_id; // 0 = não tem | 1 = tem
    uint16_t bomba_id;    // válido somente se has_bomba_id == 1

    uint8_t payload[64]; // dados
    uint16_t crc;        // CRC-16/CCITT-FALSE
} lora_app_frame_t;

_Static_assert(sizeof(lora_app_frame_t) == 79, "Formato LoRa tanque incompatível");

typedef struct {
    uint16_t gtw_id;
    uint16_t src_tank_id; // 0 = usa o tanque atual
    char msg[64];
    uint8_t msg_type; // 0/MSG_DATA = telemetria, MSG_PING = heartbeat
    uint8_t has_bomba;
    uint16_t bomba_id;
} tx_item_t;

typedef struct {
    lora_app_frame_t frame;
} lora_rx_app_item_t;

#endif
