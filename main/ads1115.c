#include "ads1115.h"

#include <limits.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

static const char *TAG = "ADS1115";

// Tabela de FSR (Full-Scale Range) em Volts, conforme enum ads1115_fsr_t
static const float ADS1115_FSR_TABLE[] = {6.144f, 4.096f, 2.048f, 1.024f, 0.512f, 0.256f};

// Tabela de SPS (Samples Per Second) em Hz, conforme enum ads1115_sps_t
static const int ADS1115_SPS_TABLE[] = {8, 16, 32, 64, 128, 250, 475, 860};

static inline bool ads1115_handle_ok(const ads1115_t *ads)
{
  return (ads != NULL && ads->dev_handle != NULL);
}

static uint32_t ads1115_conv_time_ms(const ads1115_t *ads)
{
  // conversão em ms ~ (1 / SPS) * 1000
  // acrescenta uma folga (x2) pra evitar false timeout em RTOS
  int idx = (ads && ads->config.bit.DR <= ADS1115_SPS_860) ? ads->config.bit.DR : ADS1115_SPS_128;
  int sps = ADS1115_SPS_TABLE[idx];
  if (sps <= 0)
    sps = 128;

  uint32_t t = (uint32_t)((1000 + (sps - 1)) / sps); // ceil
  if (t == 0)
    t = 1;
  return t * 2; // folga
}

static esp_err_t ads1115_write_reg16(const ads1115_t *ads, uint8_t reg, uint16_t value)
{
  uint8_t cmd[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
  return i2c_master_transmit(ads->dev_handle, cmd, sizeof(cmd), ads->max_ticks);
}

static esp_err_t ads1115_read_reg16(const ads1115_t *ads, uint8_t reg, uint16_t *out)
{
  uint8_t buf[2] = {0};
  esp_err_t ret = i2c_master_transmit_receive(ads->dev_handle, &reg, 1, buf, 2, ads->max_ticks);
  if (ret != ESP_OK)
    return ret;
  *out = (uint16_t)((buf[0] << 8) | buf[1]);
  return ESP_OK;
}

// -----------------------------------------------------------------------------
// API pública
// -----------------------------------------------------------------------------

esp_err_t ads1115_init(ads1115_t *ads, i2c_master_dev_handle_t dev_handle)
{
  if (!ads || !dev_handle)
    return ESP_ERR_INVALID_ARG;

  *ads = (ads1115_t){0};
  ads->dev_handle = dev_handle;
  ads->max_ticks = pdMS_TO_TICKS(100);

  // Default “seguro” pra 4-20mA:
  ads->config.reg = 0;
  ads->config.bit.OS = 1; // start / status
  ads->config.bit.MODE = ADS1115_MODE_SINGLE;
  ads->config.bit.MUX = ADS1115_MUX_0_GND; // A0 single-ended
  ads->config.bit.PGA = ADS1115_FSR_2_048; // bom pra sinais até ~2V
  ads->config.bit.DR = ADS1115_SPS_128;
  ads->config.bit.COMP_QUE = 0x03; // desabilita comparador

  ads->changed = true;

  // Aplica config inicial
  esp_err_t ret = ads1115_apply_config(ads);
  if (ret != ESP_OK)
    return ret;

  ESP_LOGI(TAG, "ADS1115 inicializado (MODE=SINGLE, MUX=A0, PGA=2.048V, SPS=128)");
  return ESP_OK;
}

// Compatibilidade com seu código atual (assinatura antiga)
ads1115_t ads1115_config(i2c_master_bus_handle_t bus_handle, uint8_t address, i2c_master_dev_handle_t dev_handle)
{
  (void)bus_handle;
  (void)address;

  ads1115_t ads = {0};
  esp_err_t ret = ads1115_init(&ads, dev_handle);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Falha init ADS1115 (%s)", esp_err_to_name(ret));
  else
    ESP_LOGI(TAG, "ADS1115 pronto (addr=0x%02X)", address);

  return ads;
}

void ads1115_set_mux(ads1115_t *ads, ads1115_mux_t mux)
{
  if (!ads)
    return;
  ads->config.bit.MUX = mux;
  ads->changed = true;
}

void ads1115_set_pga(ads1115_t *ads, ads1115_fsr_t fsr)
{
  if (!ads)
    return;
  ads->config.bit.PGA = fsr;
  ads->changed = true;
}

void ads1115_set_mode(ads1115_t *ads, ads1115_mode_t mode)
{
  if (!ads)
    return;
  ads->config.bit.MODE = mode;
  ads->changed = true;
}

void ads1115_set_sps(ads1115_t *ads, ads1115_sps_t sps)
{
  if (!ads)
    return;
  ads->config.bit.DR = sps;
  ads->changed = true;
}

void ads1115_set_max_ticks(ads1115_t *ads, TickType_t max_ticks)
{
  if (!ads)
    return;
  ads->max_ticks = max_ticks;
}

esp_err_t ads1115_apply_config(ads1115_t *ads)
{
  if (!ads1115_handle_ok(ads))
    return ESP_ERR_INVALID_STATE;

  esp_err_t ret = ads1115_write_reg16(ads, ADS1115_CONFIG_REGISTER_ADDR, ads->config.reg);
  if (ret == ESP_OK)
    ads->changed = false;
  return ret;
}

esp_err_t ads1115_start_single_shot(ads1115_t *ads)
{
  if (!ads1115_handle_ok(ads))
    return ESP_ERR_INVALID_STATE;

  // OS=1 inicia conversão no modo single-shot
  ads->config.bit.OS = 1;
  ads->changed = true;

  return ads1115_apply_config(ads);
}

static esp_err_t ads1115_wait_ready(const ads1115_t *ads, int timeout_ms)
{
  if (!ads1115_handle_ok(ads))
    return ESP_ERR_INVALID_STATE;

  const int64_t t0 = esp_timer_get_time();
  const uint32_t deadline_us = (timeout_ms > 0) ? (uint32_t)timeout_ms * 1000u : 0;

  // 1) delay mínimo baseado no SPS (evita “poll” agressivo)
  vTaskDelay(pdMS_TO_TICKS(ads1115_conv_time_ms(ads)));

  // 2) polling do bit OS até ficar pronto (OS=1 indica ready)
  while (true)
  {
    uint16_t cfg = 0;
    esp_err_t ret = ads1115_read_reg16(ads, ADS1115_CONFIG_REGISTER_ADDR, &cfg);
    if (ret != ESP_OK)
      return ret;

    if (cfg & 0x8000u)
      return ESP_OK; // ready

    int64_t elapsed_us = esp_timer_get_time() - t0;
    if (deadline_us > 0 && (uint32_t)elapsed_us >= deadline_us)
      return ESP_ERR_TIMEOUT;

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

static esp_err_t ads1115_read_raw_timeout(ads1115_t *ads, int16_t *out_raw, int timeout_ms)
{
  if (!ads1115_handle_ok(ads) || out_raw == NULL)
    return ESP_ERR_INVALID_STATE;

  // Se houve mudanças de configuração e estamos em modo contínuo,
  // aplica antes de ler para não ficar com leituras inconsistentes.
  if (ads->changed && ads->config.bit.MODE == ADS1115_MODE_CONTINUOUS)
  {
    esp_err_t ret = ads1115_apply_config(ads);
    if (ret != ESP_OK)
      return ret;
    // Dá tempo de ao menos 1 conversão atualizar
    vTaskDelay(pdMS_TO_TICKS(ads1115_conv_time_ms(ads)));
  }

  // Garante uma conversão nova no modo single-shot
  if (ads->config.bit.MODE == ADS1115_MODE_SINGLE)
  {
    esp_err_t ret = ads1115_start_single_shot(ads);
    if (ret != ESP_OK)
      return ret;

    ret = ads1115_wait_ready(ads, timeout_ms);
    if (ret != ESP_OK)
      return ret;
  }

  // Lê registrador de conversão
  uint16_t raw_u = 0;
  esp_err_t ret = ads1115_read_reg16(ads, ADS1115_CONVERSION_REGISTER_ADDR, &raw_u);
  if (ret != ESP_OK)
    return ret;

  *out_raw = (int16_t)raw_u;
  return ESP_OK;
}

esp_err_t ads1115_read_raw(ads1115_t *ads, int16_t *out_raw)
{
  if (!ads)
    return ESP_ERR_INVALID_STATE;

  // timeout para conversão: 4x tempo de conversão (folga) + 50ms
  const int conv_ms = (int)ads1115_conv_time_ms(ads);
  const int timeout_ms = (conv_ms * 4) + 50;

  return ads1115_read_raw_timeout(ads, out_raw, timeout_ms);
}

static float ads1115_raw_to_voltage(const ads1115_t *ads, int16_t raw)
{
  int pga = (ads && ads->config.bit.PGA <= ADS1115_FSR_0_256) ? ads->config.bit.PGA : ADS1115_FSR_2_048;
  float fsr = ADS1115_FSR_TABLE[pga];

  // ADS1115 é signed (±FSR) => 32767
  return ((float)raw * fsr) / 32767.0f;
}

esp_err_t ads1115_read_voltage(ads1115_t *ads, float *out_v)
{
  if (!ads1115_handle_ok(ads) || !out_v)
    return ESP_ERR_INVALID_STATE;

  int16_t raw = 0;
  esp_err_t ret = ads1115_read_raw(ads, &raw);
  if (ret != ESP_OK)
    return ret;

  *out_v = ads1115_raw_to_voltage(ads, raw);
  return ESP_OK;
}

esp_err_t ads1115_read_voltage_avg(ads1115_t *ads, int samples, int delay_ms, float *out_v)
{
  if (!ads1115_handle_ok(ads) || !out_v || samples <= 0)
    return ESP_ERR_INVALID_ARG;

  double acc = 0.0;
  for (int i = 0; i < samples; i++)
  {
    float v = 0;
    esp_err_t ret = ads1115_read_voltage(ads, &v);
    if (ret != ESP_OK)
      return ret;

    acc += v;
    if (delay_ms > 0)
      vTaskDelay(pdMS_TO_TICKS(delay_ms));
  }

  *out_v = (float)(acc / (double)samples);
  return ESP_OK;
}

// -----------------------------------------------------------------------------
// Conveniência “estilo simples”
// -----------------------------------------------------------------------------

int16_t ads1115_get_raw(ads1115_t *ads)
{
  int16_t raw = 0;
  esp_err_t ret = ads1115_read_raw(ads, &raw);
  if (ret != ESP_OK)
    return INT16_MIN;
  return raw;
}

float ads1115_get_voltage(ads1115_t *ads)
{
  float v = 0;
  esp_err_t ret = ads1115_read_voltage(ads, &v);
  if (ret != ESP_OK)
    return -1.0f;
  return v;
}
