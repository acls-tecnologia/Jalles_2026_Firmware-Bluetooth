#include "watchdog_rtc_raw.h"
#include "esp32/rom/rtc.h"
#include "esp_log.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc.h"
#include <inttypes.h>

// static const char *TAG = "RTC_WDT_RAW";

void rtc_wdt_init_raw()
{
    ESP_LOGI("RTC_WDT_RAW", "Iniciando RTC WDT...");

    // 1. Garante que o clock do domínio RTC esteja ligado
    rtc_clk_32k_enable(true); // ativa 32kHz interno (se disponível)

    // 2. Desativa o WDT primeiro
    REG_CLR_BIT(RTC_CNTL_WDTCONFIG0_REG, RTC_CNTL_WDT_EN);

    // 3. Configura timeout (~2 segundos)
    REG_WRITE(RTC_CNTL_WDTCONFIG1_REG, 65536);

    // 4. Ação: RESET_SYSTEM
    REG_WRITE(RTC_CNTL_WDTCONFIG2_REG, 3);
    REG_WRITE(RTC_CNTL_WDTCONFIG3_REG, 0);
    REG_WRITE(RTC_CNTL_WDTCONFIG4_REG, 0);

    // 5. Alimenta
    REG_WRITE(RTC_CNTL_WDTFEED_REG, 1);

    // 6. Habilita estágio 0 e o watchdog
    REG_SET_BIT(RTC_CNTL_WDTCONFIG0_REG, RTC_CNTL_WDT_STG0);
    REG_SET_BIT(RTC_CNTL_WDTCONFIG0_REG, RTC_CNTL_WDT_EN);

    // 7. Define tempos de reset e habilita modo runtime
    REG_SET_BIT(RTC_CNTL_WDTCONFIG0_REG, RTC_CNTL_WDT_SYS_RESET_LENGTH);
    REG_SET_BIT(RTC_CNTL_WDTCONFIG0_REG, RTC_CNTL_WDT_CPU_RESET_LENGTH);
    REG_SET_BIT(RTC_CNTL_WDTCONFIG0_REG, RTC_CNTL_WDT_FLASHBOOT_MOD_EN);

    ESP_LOGI("RTC_WDT_RAW", "Watchdog RTC habilitado.");
    ESP_LOGI("RTC_WDT_RAW", "CONFIG0: 0x%08" PRIx32, REG_READ(RTC_CNTL_WDTCONFIG0_REG));
}

void rtc_wdt_feed_raw()
{
    REG_WRITE(RTC_CNTL_WDTFEED_REG, 1);
}
