#include "modbus_rtu_master.h"
#include <string.h>

#define MB_RTU_FUNC_READ_HOLDING  (0x03)
#define MB_RTU_MAX_FRAME         (256)

static uint16_t mb_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = (crc >> 1);
        }
    }
    return crc;
}

static void txen_set(ModbusRTU_Master_t *ctx, bool enable)
{
    if (ctx->txen_port == NULL)
        return;

    bool level = enable ? ctx->txen_active_high : !ctx->txen_active_high;
    HAL_GPIO_WritePin(ctx->txen_port, ctx->txen_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void rtu_drain_rx(ModbusRTU_Master_t *ctx)
{
    uint8_t discard;

    if (ctx == NULL || ctx->huart == NULL)
        return;

    while (HAL_UART_Receive(ctx->huart, &discard, 1, 2) == HAL_OK)
    {
    }

    __HAL_UART_CLEAR_OREFLAG(ctx->huart);
    __HAL_UART_FLUSH_DRREGISTER(ctx->huart);
}

static void rtu_inter_frame_delay(void)
{
    HAL_Delay(4);
}

void ModbusRTU_Master_Init(ModbusRTU_Master_t *ctx, UART_HandleTypeDef *huart)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->huart = huart;
    ctx->tx_timeout_ms = 100;
    ctx->rx_timeout_ms = 200;
}

void ModbusRTU_Master_SetTxEnable(ModbusRTU_Master_t *ctx,
                                 GPIO_TypeDef *port,
                                 uint16_t pin,
                                 bool active_high)
{
    ctx->txen_port = port;
    ctx->txen_pin = pin;
    ctx->txen_active_high = active_high;
    txen_set(ctx, false);
}

static bool rtu_exchange(ModbusRTU_Master_t *ctx,
                       const uint8_t *req, uint16_t req_len,
                       uint8_t *resp, uint16_t resp_len_expected)
{
    rtu_inter_frame_delay();
    rtu_drain_rx(ctx);

    txen_set(ctx, true);
    if (HAL_UART_Transmit(ctx->huart, (uint8_t *)req, req_len, ctx->tx_timeout_ms) != HAL_OK)
    {
        txen_set(ctx, false);
        return false;
    }

    while (__HAL_UART_GET_FLAG(ctx->huart, UART_FLAG_TC) == RESET)
    {
    }
    txen_set(ctx, false);
    HAL_Delay(2);

    if (HAL_UART_Receive(ctx->huart, resp, resp_len_expected, ctx->rx_timeout_ms) != HAL_OK)
    {
        rtu_drain_rx(ctx);
        return false;
    }

    rtu_drain_rx(ctx);
    return true;
}

bool ModbusRTU_ReadHoldingRegisters(ModbusRTU_Master_t *ctx,
                                   uint8_t slave_id,
                                   uint16_t reg_addr,
                                   uint16_t reg_count,
                                   uint16_t *out_regs)
{
    if (!ctx || !ctx->huart || !out_regs)
        return false;
    if (reg_count == 0 || reg_count > 125)
        return false;

    uint8_t req[8];
    req[0] = slave_id;
    req[1] = MB_RTU_FUNC_READ_HOLDING;
    req[2] = (uint8_t)(reg_addr >> 8);
    req[3] = (uint8_t)(reg_addr & 0xFF);
    req[4] = (uint8_t)(reg_count >> 8);
    req[5] = (uint8_t)(reg_count & 0xFF);

    uint16_t crc = mb_crc16(req, 6);
    req[6] = (uint8_t)(crc & 0xFF);
    req[7] = (uint8_t)(crc >> 8);

    uint16_t resp_len = (uint16_t)(5 + 2 * reg_count);
    uint8_t resp[MB_RTU_MAX_FRAME];

    if (resp_len > sizeof(resp))
        return false;

    if (!rtu_exchange(ctx, req, sizeof(req), resp, resp_len))
        return false;

    uint16_t crc_calc = mb_crc16(resp, (uint16_t)(resp_len - 2));
    uint16_t crc_recv = (uint16_t)resp[resp_len - 2] | ((uint16_t)resp[resp_len - 1] << 8);
    if (crc_calc != crc_recv)
        return false;

    if (resp[0] != slave_id)
        return false;
    if (resp[1] == (MB_RTU_FUNC_READ_HOLDING | 0x80))
        return false;
    if (resp[1] != MB_RTU_FUNC_READ_HOLDING)
        return false;

    uint8_t bytecount = resp[2];
    if (bytecount != (uint8_t)(2 * reg_count))
        return false;

    for (uint16_t i = 0; i < reg_count; i++)
    {
        uint16_t hi = resp[3 + 2 * i];
        uint16_t lo = resp[3 + 2 * i + 1];
        out_regs[i] = (uint16_t)((hi << 8) | lo);
    }

    return true;
}

bool ModbusRTU_ReadHoldingRegister(ModbusRTU_Master_t *ctx,
                                  uint8_t slave_id,
                                  uint16_t reg_addr,
                                  uint16_t *out_reg)
{
    return ModbusRTU_ReadHoldingRegisters(ctx, slave_id, reg_addr, 1, out_reg);
}
