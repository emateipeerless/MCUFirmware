#ifndef MODBUS_TCP_MINIMAL_H
#define MODBUS_TCP_MINIMAL_H

#include <stdint.h>

typedef struct
{
    char     ip[16];
    uint16_t port;
    uint8_t  unit_id;
    uint16_t tx_id;
    int      sock;   /* -1 when disconnected */
} ModbusTcpClient_t;

void ModbusTcp_Init(ModbusTcpClient_t *client,
                    const char *ip,
                    uint16_t port,
                    uint8_t unit_id);

int ModbusTcp_ReadHoldingRegister(ModbusTcpClient_t *client,
                                  uint16_t reg_addr,
                                  uint16_t *out_value);

void ModbusTcp_Disconnect(ModbusTcpClient_t *client);

#endif
