/*===================================================================================*/
#if !defined __MCP23017_H__
#define __MCP23017_H__
/*===================================================================================*/
#include "driver/i2c_master.h"

#define GPA 0x12    // Estado dos pinos GPA. 1-HIGH 0-LOW
#define GPB 0x13    // Estado dos pinos GPB. 1-HIGH 0-LOW
#define IODIRA 0x00 // Configura o modo dos pinos GPA. 1-INPUT 0-OUTPUT
#define IODIRB 0x01 // Configura o modo dos pinos GPB. 1-INPUT 0-OUTPUT

#define GPA_PIN_0 0  // Pino 8.2
#define GPA_PIN_1 1u // Pino 7.4 - Saída para selecionar fase de tensão (1)
#define GPA_PIN_2 2u // Pino 7.3
#define GPA_PIN_3 3u // Pino 8.4 - Saída para selecionar fase de tensão (2)
#define GPA_PIN_4 4u // Pino 7.2
#define GPA_PIN_5 5u // Pino 7.1 - Saída Bomba
#define GPA_PIN_6 6u // Pino 8.3

#define GPB_PIN_0 0  // Pino 5.4
#define GPB_PIN_1 1u // Pino 6.1
#define GPB_PIN_2 2u // Pino 6.3
#define GPB_PIN_3 3u // Pino 6.4
#define GPB_PIN_4 4u // Pino 5.1
#define GPB_PIN_5 5u // Pino 5.2
#define GPB_PIN_6 6u // PIno 5.3

/**
 * Configurar apenas como saída
 */
#define GPA_PIN_7 7u
#define GPB_PIN_7 7u

#define PIN_LOW 0
#define PIN_HIGH 1u

/*===================================================================================*/
uint8_t InitMcp(i2c_master_dev_handle_t dev_handle);
uint8_t WritePinMcp(i2c_master_dev_handle_t dev_handle, uint8_t Port, uint8_t Pin, uint8_t Value);
int8_t ReadPinMcp(i2c_master_dev_handle_t dev_handle, uint8_t Port, uint8_t Pin);
/*===================================================================================*/
#endif //__MCP23017_H__
       /*===================================================================================*/