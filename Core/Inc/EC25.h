/*
 * EC25.h
 *
 *  Created on: Jan 19, 2026
 *      Author: 113294
 */

#ifndef INC_EC25_H_
#define INC_EC25_H_

#include "stm32h7xx_hal.h"


void lte_tx(UART_HandleTypeDef *huart, const char *s);
static void lte_tx_raw_len(const char *buf, size_t len);
int lte_cmd_expect(const char *cmd,const char *needle,uint32_t timeout_ms,UART_HandleTypeDef *huart);
void BlueOn();
void RedOn();
void BlueOFF();
void RedOFF();
void MQTTdisconnect(UART_HandleTypeDef *huart);
float CurrentToTemp(float mA);
void SendTemp(float temp, UART_HandleTypeDef *huart);
void CELLSetup(UART_HandleTypeDef *huart);
bool EC25_PUBLISH(const char *Message,int type,UART_HandleTypeDef *huart);

#endif /* INC_EC25_H_ */
