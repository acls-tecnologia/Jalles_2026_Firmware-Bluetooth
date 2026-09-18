#include "config.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

char wifi_ssid[33] = "";
char wifi_password[65] = "";

char g_api_user[API_USER_MAX_LEN] = "";
char g_api_pass[API_PASS_MAX_LEN] = "";
int g_provision_tanque_id = 0;
int g_lora_gtw_id = 0;

static const char *TAG_CFG = "CFG";
static const char *NVS_NS = "NVS - CFG";
static const char *NVS_WIFI_NS = "wifi_cfg";

// Global
device_cfg_t g_cfg = {0};

int tanque_bomba_pwm_id(void) {
    int tanque_id = (g_cfg.tanque_id > 0) ? g_cfg.tanque_id : PROVISION_TANQUE_ID;

    switch (tanque_id) {
    case TANQUE_RD0_ID:
        return BOMBA_PWM_RD0_ID;
    case TANQUE_BOOSTER_ID:
        return BOMBA_PWM_BOOSTER_ID;
    default:
        return 0;
    }
}

bool tanque_bomba_eh_pwm(int bomba_id) {
    int bomba_pwm_id = tanque_bomba_pwm_id();
    return bomba_pwm_id > 0 && bomba_id == bomba_pwm_id;
}

size_t heap_total = 0;

EventGroupHandle_t sys_event_group = NULL;

i2c_master_bus_handle_t bus_handle = NULL;
i2c_master_dev_handle_t mcp_handle = NULL;
i2c_master_dev_handle_t ads_handle = NULL;

ads1115_t _4a20ma = {0};

SemaphoreHandle_t i2c_semaphore = NULL;
SemaphoreHandle_t MutexHTTP = NULL;
SemaphoreHandle_t MutexLora = NULL;
SemaphoreHandle_t g_lora_ack_mutex = NULL;

volatile int g_local_remoto[MAX_BOMBAS] = {0};
volatile int g_emergencia[MAX_BOMBAS] = {0};
volatile bool g_status_bomba[MAX_BOMBAS] = {0}; // true = Ligada, false = Desligada
volatile bool g_status_bomba_desejado[MAX_BOMBAS] = {
    0}; // true = Ligada, false = Desligada (último status desejado recebido do servidor, para cada bomba)
volatile bool g_controle_bomba_habilitado[MAX_BOMBAS] = {
    0}; // servidor liberou receber comandos? (true/false por bomba)

volatile int msg_counter = 0;

QueueHandle_t lora_tx_queue = NULL;
QueueHandle_t lora_rx_app_queue = NULL;
lora_ack_wait_t g_lora_ack_wait = {0};
lora_dup_entry_t g_lora_dup_cache[LORA_DUP_CACHE_SIZE] = {0};
TaskHandle_t g_lora_app_task_handle = NULL;

volatile bool g_comando_bomba_desejado = false; // último comando recebido (liga/desliga)
volatile float g_vazao_pwm_percent = 0.0f;      // último setpoint salvo

const char *rootCaCerticate = "-----BEGIN CERTIFICATE-----\n"
                              "MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw\n"
                              "TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh\n"
                              "cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4\n"
                              "WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu\n"
                              "ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY\n"
                              "MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc\n"
                              "h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+\n"
                              "0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U\n"
                              "A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW\n"
                              "T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH\n"
                              "B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC\n"
                              "B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv\n"
                              "KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn\n"
                              "OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn\n"
                              "jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw\n"
                              "qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI\n"
                              "rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV\n"
                              "HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq\n"
                              "hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL\n"
                              "ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ\n"
                              "3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK\n"
                              "NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5\n"
                              "ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur\n"
                              "TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC\n"
                              "jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc\n"
                              "oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq\n"
                              "4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA\n"
                              "mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d\n"
                              "emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=\n"
                              "-----END CERTIFICATE-----\n";

esp_err_t cfg_nvs_init(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG_CFG, "NVS truncada/versão nova -> apagando...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_timer_handle_t watchdog_timer = NULL;

void cfg_apply_runtime(const device_cfg_t *cfg) {
    if (!cfg)
        return;

    if (cfg->api_user[0] != '\0')
        strlcpy(g_api_user, cfg->api_user, sizeof(g_api_user));
    else if (cfg->device_id[0] != '\0')
        strlcpy(g_api_user, cfg->device_id, sizeof(g_api_user));

    if (cfg->api_pass[0] != '\0')
        strlcpy(g_api_pass, cfg->api_pass, sizeof(g_api_pass));

    if (cfg->tanque_id > 0)
        g_provision_tanque_id = cfg->tanque_id;

    if (cfg->lora_gtw_id > 0)
        g_lora_gtw_id = cfg->lora_gtw_id;
}

static void cfg_set_defaults(device_cfg_t *cfg) {
    // valores default (usados se NVS não tiver ou for inválida)
    memset(cfg, 0, sizeof(*cfg));
    cfg->unidade_id = 0;

    // tanque_id 0 é inválido, força provisionamento
    cfg->tanque_id = 0; // tanque_id 0 é inválido, força provisionamento

    // zera bombas
    memset(cfg->bomba_id, 0, sizeof(cfg->bomba_id));
    cfg->qtd_bombas = 0;

    // perfil default é NIVEL (apenas leitura de nível, sem bombas)
    cfg->ativo = true;
    cfg->automatico = 0;
    cfg->nivel_maximo = 100;
    cfg->nivel_minimo = 0;

    cfg->api_user[0] = '\0';
    cfg->api_pass[0] = '\0';
    cfg->device_id[0] = '\0';
    cfg->lora_gtw_id = 0;
    cfg->perfil = TANQUE_NIVEL;
    cfg->provisionado = false;

    // nome_tanque derivado do usuário (ex: GTW_R04 -> R-04)
    cfg->nome_tanque[0] = '\0';
}

bool cfg_load(device_cfg_t *out) {
    if (!out)
        return false;

    cfg_set_defaults(out);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK)
        return false;

    uint8_t prov = 0;
    if (nvs_get_u8(h, "prov", &prov) != ESP_OK || prov != 1) {
        nvs_close(h);
        return false;
    }

    // Lê device_id salvo
    size_t len = sizeof(out->device_id);
    if (nvs_get_str(h, "device", out->device_id, &len) != ESP_OK) {
        nvs_close(h);
        return false;
    }

    len = sizeof(out->api_user);
    if (nvs_get_str(h, "api_user", out->api_user, &len) != ESP_OK) {
        strlcpy(out->api_user, out->device_id, sizeof(out->api_user));
    }

    len = sizeof(out->api_pass);
    if (nvs_get_str(h, "api_pass", out->api_pass, &len) != ESP_OK) {
        out->api_pass[0] = '\0';
    }

    strlcpy(out->device_id, out->api_user, sizeof(out->device_id));

    int32_t v = 0;
    nvs_get_i32(h, "unidade", &v);
    out->unidade_id = (int)v;
    nvs_get_i32(h, "tanque", &v);
    out->tanque_id = (int)v;

    if (nvs_get_i32(h, "lora_gtw", &v) == ESP_OK)
        out->lora_gtw_id = (int)v;

    uint8_t qtdb = 0;
    nvs_get_u8(h, "qtdb", &qtdb);
    if (qtdb > MAX_BOMBAS)
        qtdb = MAX_BOMBAS;
    out->qtd_bombas = qtdb;

    // zera + lê b0..b2
    for (int i = 0; i < MAX_BOMBAS; i++)
        out->bomba_id[i] = 0;

    int32_t b = 0;
    if (nvs_get_i32(h, "b0", &b) == ESP_OK)
        out->bomba_id[0] = (int)b;
    if (nvs_get_i32(h, "b1", &b) == ESP_OK)
        out->bomba_id[1] = (int)b;
    if (nvs_get_i32(h, "b2", &b) == ESP_OK)
        out->bomba_id[2] = (int)b;

    uint8_t p = 0;
    if (nvs_get_u8(h, "perfil", &p) == ESP_OK)
        out->perfil = (p == 1) ? TANQUE_FULL : TANQUE_NIVEL;

    // nome
    size_t nlen = sizeof(out->nome_tanque);
    nvs_get_str(h, "nome", out->nome_tanque, &nlen);

    uint8_t ativo_u8 = 1;
    if (nvs_get_u8(h, "ativo", &ativo_u8) == ESP_OK)
        out->ativo = (ativo_u8 != 0);

    int32_t auto_i = 0;
    if (nvs_get_i32(h, "auto", &auto_i) == ESP_OK)
        out->automatico = (int)auto_i;

    int32_t nmax_i = 100;
    if (nvs_get_i32(h, "nmax", &nmax_i) == ESP_OK)
        out->nivel_maximo = (int)nmax_i;

    int32_t nmin_i = 0;
    if (nvs_get_i32(h, "nmin", &nmin_i) == ESP_OK)
        out->nivel_minimo = (int)nmin_i;

    int32_t legacy = 0;
    if (nvs_get_i32(h, "bomba", &legacy) == ESP_OK) {
        if (out->bomba_id[0] == 0)
            out->bomba_id[0] = (int)legacy;
        if (out->qtd_bombas == 0 && legacy > 0)
            out->qtd_bombas = 1;
    }

    // coerência de perfil
    if (out->qtd_bombas > MAX_BOMBAS)
        out->qtd_bombas = MAX_BOMBAS;
    out->perfil = (out->qtd_bombas >= 1) ? TANQUE_FULL : TANQUE_NIVEL;

    out->provisionado = true;
    nvs_close(h);

    if (out->api_user[0] == '\0' || out->api_pass[0] == '\0' || out->tanque_id <= 0 || out->lora_gtw_id <= 0) {
        ESP_LOGW(TAG_CFG, "CFG incompleta na NVS; mantendo modo de configuracao BLE");
        return false;
    }

    cfg_apply_runtime(out);

    return true;
}

bool cfg_wifi_load(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_WIFI_NS, NVS_READONLY, &h);
    if (err != ESP_OK)
        return false;

    char saved_ssid[sizeof(wifi_ssid)] = {0};
    char saved_password[sizeof(wifi_password)] = {0};
    size_t ssid_len = sizeof(saved_ssid);
    size_t password_len = sizeof(saved_password);

    err = nvs_get_str(h, "ssid", saved_ssid, &ssid_len);
    if (err == ESP_OK)
        err = nvs_get_str(h, "password", saved_password, &password_len);

    nvs_close(h);

    if (err != ESP_OK || saved_ssid[0] == '\0')
        return false;

    strlcpy(wifi_ssid, saved_ssid, sizeof(wifi_ssid));
    strlcpy(wifi_password, saved_password, sizeof(wifi_password));
    ESP_LOGI(TAG_CFG, "WiFi carregado da NVS: %s", wifi_ssid);
    return true;
}

esp_err_t cfg_wifi_save(const char *ssid, const char *password) {
    if (!ssid || !password || ssid[0] == '\0')
        return ESP_ERR_INVALID_ARG;

    if (strlen(ssid) >= sizeof(wifi_ssid) || strlen(password) >= sizeof(wifi_password))
        return ESP_ERR_INVALID_SIZE;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_WIFI_NS, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    err = nvs_set_str(h, "ssid", ssid);
    if (err == ESP_OK)
        err = nvs_set_str(h, "password", password);
    if (err == ESP_OK)
        err = nvs_commit(h);

    nvs_close(h);

    if (err == ESP_OK) {
        strlcpy(wifi_ssid, ssid, sizeof(wifi_ssid));
        strlcpy(wifi_password, password, sizeof(wifi_password));
        ESP_LOGI(TAG_CFG, "WiFi salvo na NVS: %s", wifi_ssid);
    }

    return err;
}

esp_err_t cfg_save(const device_cfg_t *cfg) {
    if (!cfg)
        return ESP_ERR_INVALID_ARG;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    const char *cfg_user = (cfg->api_user[0] != '\0') ? cfg->api_user : cfg->device_id;
    const char *cfg_pass = (cfg->api_pass[0] != '\0') ? cfg->api_pass : g_api_pass;
    int cfg_lora_gtw_id = (cfg->lora_gtw_id > 0) ? cfg->lora_gtw_id : g_lora_gtw_id;

    if (cfg_user[0] == '\0' || cfg_pass[0] == '\0' || cfg->tanque_id <= 0 || cfg_lora_gtw_id <= 0) {
        ESP_LOGW(TAG_CFG, "Recusando salvar CFG incompleta");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(nvs_set_u8(h, "prov", 1));             // marcado como provisionado
    ESP_ERROR_CHECK(nvs_set_str(h, "device", cfg_user));   // device_id
    ESP_ERROR_CHECK(nvs_set_str(h, "api_user", cfg_user)); // usuario API
    ESP_ERROR_CHECK(nvs_set_str(h, "api_pass", cfg_pass)); // senha API
    ESP_ERROR_CHECK(nvs_set_str(h, "nome", cfg->nome_tanque)); // nome do tanque

    ESP_ERROR_CHECK(nvs_set_i32(h, "unidade", cfg->unidade_id)); // ID da unidade
    ESP_ERROR_CHECK(nvs_set_i32(h, "tanque", cfg->tanque_id));   // ID do tanque
    ESP_ERROR_CHECK(nvs_set_i32(h, "lora_gtw", cfg_lora_gtw_id)); // ID GTW LoRa

    ESP_ERROR_CHECK(nvs_set_u8(h, "qtdb", cfg->qtd_bombas)); // quantidade de bombas
    ESP_ERROR_CHECK(nvs_set_i32(h, "b0", cfg->bomba_id[0])); // bomba 0
    ESP_ERROR_CHECK(nvs_set_i32(h, "b1", cfg->bomba_id[1])); // bomba 1
    ESP_ERROR_CHECK(nvs_set_i32(h, "b2", cfg->bomba_id[2])); // bomba 2

    ESP_ERROR_CHECK(nvs_set_u8(h, "ativo", cfg->ativo ? 1 : 0));
    ESP_ERROR_CHECK(nvs_set_i32(h, "auto", cfg->automatico));
    ESP_ERROR_CHECK(nvs_set_i32(h, "nmax", cfg->nivel_maximo));
    ESP_ERROR_CHECK(nvs_set_i32(h, "nmin", cfg->nivel_minimo));

    ESP_ERROR_CHECK(nvs_set_u8(h, "perfil", (cfg->perfil == TANQUE_FULL) ? 1 : 0)); // perfil

    err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK)
        cfg_apply_runtime(cfg);
    return err;
}

esp_err_t cfg_save_level_limits(int nivel_minimo, int nivel_maximo) {
    if (nivel_minimo < 0 || nivel_minimo > 100 || nivel_maximo < 0 || nivel_maximo > 100 ||
        nivel_minimo >= nivel_maximo) {
        return ESP_ERR_INVALID_ARG;
    }

    if (g_cfg.nivel_minimo == nivel_minimo && g_cfg.nivel_maximo == nivel_maximo)
        return ESP_OK;

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    err = nvs_set_i32(h, "nmin", nivel_minimo);
    if (err == ESP_OK)
        err = nvs_set_i32(h, "nmax", nivel_maximo);
    if (err == ESP_OK)
        err = nvs_commit(h);

    nvs_close(h);

    if (err == ESP_OK) {
        g_cfg.nivel_minimo = nivel_minimo;
        g_cfg.nivel_maximo = nivel_maximo;
        ESP_LOGI(TAG_CFG, "Limites de nivel salvos na NVS: min=%d max=%d", nivel_minimo, nivel_maximo);
    }

    return err;
}

void cfg_log(const device_cfg_t *cfg) {
    if (!cfg)
        return;
    ESP_LOGI("CFG", "CFG: prov=%d user=%s nome=%s unidade=%d tanque_id=%d lora_gtw=%d qtd=%d b0=%d b1=%d b2=%d perfil=%s",
             cfg->provisionado ? 1 : 0, cfg->api_user, cfg->nome_tanque, cfg->unidade_id, cfg->tanque_id,
             cfg->lora_gtw_id, cfg->qtd_bombas, cfg->bomba_id[0], cfg->bomba_id[1], cfg->bomba_id[2],
             (cfg->perfil == TANQUE_FULL) ? "FULL" : "NIVEL");
}

void cfg_guess_nome_tanque_from_user(char *out, size_t out_len) {
    // Regras simples:
    // "GTW_R04" -> "R-04"
    // "GTW_R0"  -> "R-0"
    // Se já vier "R-xx", mantém.
    if (!out || out_len == 0)
        return;
    out[0] = 0;

    const char *p = strstr(API_USER, "GTW_");
    p = p ? (p + 4) : API_USER; // após "GTW_"

    // p agora deve ser algo tipo "R04", "R0", "R-04"
    if (strchr(p, '-') != NULL) {
        snprintf(out, out_len, "%s", p);
        return;
    }

    // Se começa com R e depois número(s), vira "R-<resto>"
    if (p[0] == 'R')
        snprintf(out, out_len, "R-%s", (p + 1));
    else
        snprintf(out, out_len, "%s", p);
}

esp_err_t cfg_erase(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;

    err = nvs_erase_all(h);
    if (err == ESP_OK)
        err = nvs_commit(h);

    nvs_close(h);
    return err;
}

static bool cfg_same_bombas(const device_cfg_t *a, const device_cfg_t *b) {
    if (a->qtd_bombas != b->qtd_bombas)
        return false;

    for (int i = 0; i < MAX_BOMBAS; i++) {
        if (a->bomba_id[i] != b->bomba_id[i])
            return false;
    }
    return true;
}

bool cfg_important_equal(const device_cfg_t *a, const device_cfg_t *b, bool *need_restart) {
    bool restart = false;

    if (a->perfil != b->perfil)
        restart = true;
    if (a->tanque_id != b->tanque_id)
        restart = true;
    if (a->unidade_id != b->unidade_id)
        restart = true;
    if (a->lora_gtw_id != b->lora_gtw_id)
        restart = true;
    if (strcmp(a->api_user, b->api_user) != 0)
        restart = true;
    if (strcmp(a->api_pass, b->api_pass) != 0)
        restart = true;

    // bombas (não precisa reiniciar obrigatoriamente)
    bool same_bombas = cfg_same_bombas(a, b);

    if (need_restart)
        *need_restart = restart;

    // "igual" para fins de update geral
    if (!restart && same_bombas && a->ativo == b->ativo && a->automatico == b->automatico &&
        a->nivel_minimo == b->nivel_minimo && a->nivel_maximo == b->nivel_maximo) {
        return true;
    }
    return false;
}
