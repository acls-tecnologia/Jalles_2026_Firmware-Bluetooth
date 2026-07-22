#ifndef ADS1115_H
#define ADS1115_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

  // -----------------------------------------------------------------------------
  // Registradores
  // -----------------------------------------------------------------------------

  typedef enum
  {
    ADS1115_CONVERSION_REGISTER_ADDR = 0x00,
    ADS1115_CONFIG_REGISTER_ADDR = 0x01,
    ADS1115_LO_THRESH_REGISTER_ADDR = 0x02,
    ADS1115_HI_THRESH_REGISTER_ADDR = 0x03,
  } ads1115_register_addresses_t;

  // -----------------------------------------------------------------------------
  // Enums de configuração
  // -----------------------------------------------------------------------------

  typedef enum
  {
    // Diferencial
    ADS1115_MUX_0_1 = 0,
    ADS1115_MUX_0_3,
    ADS1115_MUX_1_3,
    ADS1115_MUX_2_3,
    // Single-ended
    ADS1115_MUX_0_GND, // Pino 4.4 - Corrente bomba 3 (se tiver)
    ADS1115_MUX_1_GND, // Pino 4.3 - Corrente bomba 2 (se tiver)
    ADS1115_MUX_2_GND, // Pino 4.2 - Corrente bomba 1
    ADS1115_MUX_3_GND, // Pino 4.1 - Nível do tanque
  } ads1115_mux_t;

  typedef enum
  {
    ADS1115_FSR_6_144 = 0,
    ADS1115_FSR_4_096,
    ADS1115_FSR_2_048,
    ADS1115_FSR_1_024,
    ADS1115_FSR_0_512,
    ADS1115_FSR_0_256,
  } ads1115_fsr_t;

  typedef enum
  {
    ADS1115_SPS_8 = 0,
    ADS1115_SPS_16,
    ADS1115_SPS_32,
    ADS1115_SPS_64,
    ADS1115_SPS_128,
    ADS1115_SPS_250,
    ADS1115_SPS_475,
    ADS1115_SPS_860,
  } ads1115_sps_t;

  typedef enum
  {
    ADS1115_MODE_CONTINUOUS = 0,
    ADS1115_MODE_SINGLE = 1,
  } ads1115_mode_t;

  // -----------------------------------------------------------------------------
  // Config Register (0x01)
  // -----------------------------------------------------------------------------

  typedef union
  {
    struct
    {
      uint16_t COMP_QUE : 2;  // bits 0..1  Comparator queue and disable
      uint16_t COMP_LAT : 1;  // bit 2      Latching Comparator
      uint16_t COMP_POL : 1;  // bit 3      Comparator Polarity
      uint16_t COMP_MODE : 1; // bit 4      Comparator Mode
      uint16_t DR : 3;        // bits 5..7  Data rate
      uint16_t MODE : 1;      // bit 8      Operating mode
      uint16_t PGA : 3;       // bits 9..11 Programmable gain
      uint16_t MUX : 3;       // bits 12..14 Mux
      uint16_t OS : 1;        // bit 15     Operational status / start conv
    } bit;
    uint16_t reg;
  } ads1115_config_reg_t;

  // -----------------------------------------------------------------------------
  // Handle
  // -----------------------------------------------------------------------------

  typedef struct
  {
    ads1115_config_reg_t config;
    i2c_master_dev_handle_t dev_handle;
    TickType_t max_ticks; // timeout do I2C
    bool changed;         // indica que a config local foi alterada e precisa ser aplicada
  } ads1115_t;

  // -----------------------------------------------------------------------------
  // API pública
  // -----------------------------------------------------------------------------

  // Mantém compatibilidade com a sua assinatura antiga (retorna struct por valor)
  ads1115_t ads1115_config(i2c_master_bus_handle_t bus_handle, uint8_t address, i2c_master_dev_handle_t dev_handle);

  // Nova init (recomendada)
  esp_err_t ads1115_init(ads1115_t *ads, i2c_master_dev_handle_t dev_handle);

  // Ajustes (marcam changed=true)
  void ads1115_set_mux(ads1115_t *ads, ads1115_mux_t mux);
  void ads1115_set_pga(ads1115_t *ads, ads1115_fsr_t fsr);
  void ads1115_set_mode(ads1115_t *ads, ads1115_mode_t mode);
  void ads1115_set_sps(ads1115_t *ads, ads1115_sps_t sps);
  void ads1115_set_max_ticks(ads1115_t *ads, TickType_t max_ticks);

  // Aplica config atual no registrador 0x01
  esp_err_t ads1115_apply_config(ads1115_t *ads);

  // Força início de uma conversão (single-shot)
  esp_err_t ads1115_start_single_shot(ads1115_t *ads);

  // Leituras robustas (com esp_err_t)
  esp_err_t ads1115_read_raw(ads1115_t *ads, int16_t *out_raw);
  esp_err_t ads1115_read_voltage(ads1115_t *ads, float *out_v);
  esp_err_t ads1115_read_voltage_avg(ads1115_t *ads, int samples, int delay_ms, float *out_v);

  // Conveniência “estilo simples”
  int16_t ads1115_get_raw(ads1115_t *ads);   // INT16_MIN em erro
  float ads1115_get_voltage(ads1115_t *ads); // <0 em erro

#ifdef __cplusplus
}
#endif

#endif // ADS1115_H
