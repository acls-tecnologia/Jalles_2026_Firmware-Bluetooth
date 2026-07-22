/**
 * @brief Driver para o MCP23017
 *
 * @author:     Mouzarthy F. Soares
 *
 * Para implementar:
 *
 * Implementados:
 *
 * Alterações:
 * - Atualizado para usar driver/i2c_master.h (16/05/2025)
 * - Corrigido uso de i2c_master_transmit e i2c_master_transmit_receive (16/05/2025)
 *
 * Correções:
 *
 */
/*===================================================================================*/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/*===================================================================================*/
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/*===================================================================================*/
#include "esp_system.h"
#include "esp_log.h"
/*===================================================================================*/
#include "driver/i2c_master.h"
/*===================================================================================*/
#include "mcp23017.h"
/*===================================================================================*/
#define TAG "MCP23017"

#define INPUT 0xff
#define OUTPUT 0x00

/*===================================================================================*/
/**
 * Função para escrever nos registradores do MCP23017
 *
 * @param dev_handle  Handle do dispositivo MCP23017
 * @param Register    Registrador interno do MCP a ser escrito
 * @param Value       Byte a ser escrito no registrador do MCP
 */
uint8_t WriteRegisterMcp(i2c_master_dev_handle_t dev_handle, uint8_t Register, uint8_t Value)
{
    uint8_t data[2] = {Register, Value};
    esp_err_t err = i2c_master_transmit(dev_handle, data, sizeof(data), 1000 / portTICK_PERIOD_MS);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro ao escrever no registrador 0x%02x: %s", Register, esp_err_to_name(err));
        return 1;
    }

    return 0;
}

/*===================================================================================*/
/**
 * Função para ler um registrador do MCP23017
 *
 * @param dev_handle  Handle do dispositivo MCP23017
 * @param Register    Registrador interno do MCP a ser lido
 */
uint8_t ReadRegisterMcp(i2c_master_dev_handle_t dev_handle, uint8_t Register)
{
    uint8_t read_data = 0;
    esp_err_t err = i2c_master_transmit_receive(dev_handle, &Register, 1, &read_data, 1, 1000 / portTICK_PERIOD_MS);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Erro ao ler registrador 0x%02x: %s", Register, esp_err_to_name(err));
        return -1;
    }

    return (int16_t)read_data;
}

/*===================================================================================*/
/**
 * Função para inicializar o MCP23017, definindo as entradas e saídas
 */
uint8_t InitMcp(i2c_master_dev_handle_t dev_handle)
{
    if (WriteRegisterMcp(dev_handle, IODIRA, OUTPUT))
    {
        ESP_LOGE(TAG, "Erro ao configurar IODIRA");
        return 1;
    }
    if (WriteRegisterMcp(dev_handle, IODIRB, INPUT))
    {
        ESP_LOGE(TAG, "Erro ao configurar IODIRB");
        return 1;
    }
    if (WriteRegisterMcp(dev_handle, GPA, 0x00))
    {
        ESP_LOGE(TAG, "Erro ao configurar GPA");
        return 1;
    }
    if (WriteRegisterMcp(dev_handle, GPB, 0x00))
    {
        ESP_LOGE(TAG, "Erro ao configurar GPB");
        return 1;
    }

    ESP_LOGI(TAG, "MCP23017 inicializado com sucesso");
    return 0;
}

/*===================================================================================*/
/**
 * Função para tornar uma saída de um I/O do MCP23017 0 ou 1
 *
 * @param dev_handle  Handle do dispositivo MCP23017
 * @param Port        Porta de I/O do MCP (GPA ou GPB)
 * @param Pin         Pino de I/O das portas GPA ou GPB
 * @param Value       Define o nível lógico do pino, 0 ou 1
 */
uint8_t WritePinMcp(i2c_master_dev_handle_t dev_handle, uint8_t Port, uint8_t Pin, uint8_t Value)
{
    int16_t r = ReadRegisterMcp(dev_handle, Port);

    if (r < 0)
    {
        ESP_LOGE(TAG, "Erro ao ler porta 0x%02x", Port);
        return 1;
    }

    uint8_t Buffer = (uint8_t)r;

    if (Value == PIN_LOW)
        Buffer &= ~(1 << Pin);
    else
        Buffer |= (1 << Pin);

    if (WriteRegisterMcp(dev_handle, Port, Buffer))
    {
        ESP_LOGE(TAG, "Erro ao escrever na porta 0x%02x", Port);
        return 1;
    }

    return 0;
}

/*===================================================================================*/
/**
 * Função para ler um I/O do MCP23017
 *
 * @param dev_handle  Handle do dispositivo MCP23017
 * @param Port        Porta de I/O do MCP (GPA ou GPB)
 * @param Pin         Pino de I/O das portas GPA ou GPB a ser lido
 */
int8_t ReadPinMcp(i2c_master_dev_handle_t dev_handle, uint8_t Port, uint8_t Pin)
{
    int16_t Buffer = ReadRegisterMcp(dev_handle, Port);

    if (Buffer < 0)
    {
        ESP_LOGE(TAG, "Erro ao ler porta 0x%02x", Port);
        return -1;
    }

    return (Buffer & (1 << Pin)) ? 1 : 0;
}

/*===================================================================================*/