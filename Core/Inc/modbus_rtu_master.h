#ifndef MODBUS_RTU_MASTER_H
#define MODBUS_RTU_MASTER_H

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *txen_port;
    uint16_t      txen_pin;
    bool          txen_active_high;
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
} ModbusRTU_Master_t;

void ModbusRTU_Master_Init(ModbusRTU_Master_t *ctx, UART_HandleTypeDef *huart);

void ModbusRTU_Master_SetTxEnable(ModbusRTU_Master_t *ctx,
                                 GPIO_TypeDef *port,
                                 uint16_t pin,
                                 bool active_high);

bool ModbusRTU_ReadHoldingRegisters(ModbusRTU_Master_t *ctx,
                                     uint8_t slave_id,
                                     uint16_t reg_addr,
                                     uint16_t reg_count,
                                     uint16_t *out_regs);

bool ModbusRTU_ReadHoldingRegister(ModbusRTU_Master_t *ctx,
                                  uint8_t slave_id,
                                  uint16_t reg_addr,
                                  uint16_t *out_reg);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_MASTER_H */
