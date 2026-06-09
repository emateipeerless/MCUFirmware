/*
 * EC25.c
 *
 *  Created on: Jan 19, 2026
 *      Author: 113294
 */


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdarg.h>

#include "stm32h7xx_hal.h"
#include "LED.h"

const TESTTOPIC = "test/send";
const TELEM_TOPIC = "PeerConn/123/telemetry";


//RAW TX FUNCTION
char *result="";
int mat = 0;

void lte_tx(const char *s,UART_HandleTypeDef *huart)
{
    HAL_UART_Transmit(huart,
                      (uint8_t *)s,
                      (uint16_t)strlen(s),
                      1000);
}


static void lte_tx_raw_len(const char *buf, size_t len,UART_HandleTypeDef *huart){
for (size_t i = 0; i < len; i++) {
	char c[2] = { buf[i], 0 };
	lte_tx(c,huart);
	}
}


int lte_cmd_expect(const char *cmd,
                   const char *needle,
                   uint32_t timeout_ms,
				   UART_HandleTypeDef *huart) {
    char buf[512];
    size_t len = 0;
    const size_t max = sizeof(buf) - 1;

    uint32_t start = HAL_GetTick();
    uint32_t last_rx = start;

    int matched = 0;
    const uint32_t TAIL_IDLE_MS = 500; // how long to wait *after* match with no new data

    // 1) send command (if any)
    if (cmd && cmd[0]) {
        lte_tx(cmd,huart);
    }

    buf[0] = '\0';

    // 2) read response
    while ((HAL_GetTick() - start) < timeout_ms && len < max) {
        uint8_t ch;

        // short per-byte timeout
        if (HAL_UART_Receive(huart, &ch, 1, 500) == HAL_OK) {
            buf[len++] = (char)ch;
            buf[len] = '\0';

            last_rx = HAL_GetTick();

            // check for needle once we have some data
            if (!matched && strstr(buf, needle) != NULL) {
                matched = 1;
                // don't break yet — we want to slurp the trailing "\r\n"
                // and any extra URCs for a short idle period
            }
        } else {
            // no byte right now
            if (matched && (HAL_GetTick() - last_rx) > TAIL_IDLE_MS) {
                // we've already seen the needle AND the line is quiet:
                // treat this as the end of this response
                //break;
            }
            // if not matched yet, just keep waiting until overall timeout_ms
        }
    }
    result = strstr(buf,needle);
    mat = strcmp(result,needle);
    printf("Command:%s", buf);
    if (result && strncmp(result, needle, strlen(needle)) == 0) {
        printf("MATCHED Result: %s\n", result);
    }


    buf[len] = '\0';

    // Debug: show exactly what *this* call saw
    bzero(buf,sizeof(buf));

    return matched;
}
void CELLSetup(UART_HandleTypeDef *huart){
	BlueOn();
	RedOn();

	int ok = 1;
	ok &= lte_cmd_expect("AT\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOn();
	RedOn();
	ok &= lte_cmd_expect("AT+CPIN?\r\n", "READY", 500,huart);
	HAL_Delay(250);
	BlueOFF();
	RedOFF();
	ok &= lte_cmd_expect("AT+QMTCLOSE=0\r\n", "OK", 500,huart);
	ok &= lte_cmd_expect("AT+CREG?\r\n", "0,5", 500,huart);
	HAL_Delay(250);
	BlueOn();
	RedOn();
	ok &= lte_cmd_expect("AT+CSQ\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOFF();
	RedOFF();

	ok &= lte_cmd_expect("AT+COPS?\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOn();
	RedOn();
	ok &= lte_cmd_expect("AT+CGACT=1,1\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOFF();
	RedOFF();

	ok &= lte_cmd_expect("AT+QSSLCFG=\"cacert\",2,\"UFS:PC2root.pem\"\r\n", "OK", 1000,huart);//cacc.pem for first cell chiup
	HAL_Delay(250);
	BlueOn();
	RedOn();
	ok &= lte_cmd_expect("AT+QSSLCFG=\"clientcert\",2,\"UFS:PC2cert.pem\"\r\n", "OK", 1000,huart);
	HAL_Delay(250);
	BlueOFF();
	RedOFF();

	ok &= lte_cmd_expect("AT+QSSLCFG=\"clientkey\",2,\"UFS:PC2priv.pem\"\r\n", "OK", 1000,huart);
	HAL_Delay(250);
	BlueOn();
	RedOn();
	ok &= lte_cmd_expect("AT+QMTCFG=\"ssl\",0,1,2\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOFF();
	RedOFF();

	ok &= lte_cmd_expect("AT+QSSLCFG=\"seclevel\",2,2\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOn();
	RedOn();
	ok &= lte_cmd_expect("AT+QSSLCFG=\"sslversion\",2,4\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOFF();
	RedOFF();

	ok &= lte_cmd_expect("AT+QSSLCFG=\"ciphersuite\",2,0xFFFF\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOn();
	RedOn();
	ok &= lte_cmd_expect("AT+QSSLCFG=\"ignorelocaltime\",2,1\r\n", "OK", 500,huart);
	HAL_Delay(250);
	BlueOFF();
	RedOFF();

	ok &= lte_cmd_expect("AT+QMTOPEN=0,\"awdj19q5ciaa2-ats.iot.us-east-1.amazonaws.com\",8883\r\n", "0,0", 3400,huart);
	HAL_Delay(750);
	BlueOn();
	RedOn();
	ok &= lte_cmd_expect("AT+QMTCONN=0,\"PeerConn2\"\r\n", "0,0,0", 1000,huart); //TRFC2 for this ID1 cell chip
	HAL_Delay(750);
	BlueOFF();
	RedOFF();
	BlueOn();



}
static int lte_read_window(UART_HandleTypeDef *huart, char *buf, size_t buf_sz, uint32_t window_ms)
{
    size_t len = 0;
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < window_ms && len < (buf_sz - 1)) {
        uint8_t ch;
        if (HAL_UART_Receive(huart, &ch, 1, 50) == HAL_OK) {
            buf[len++] = (char)ch;
            buf[len] = '\0';
        }
    }
    if (len) {
        printf("[LTE RX %u ms][%u bytes]:\r\n%.*s\r\n", (unsigned)window_ms, (unsigned)len, (int)len, buf);
    }
    return (int)len;
}
uint16_t msgid=1;
bool EC25_PUBLISH(const char *Message,int type,UART_HandleTypeDef *huart)//type is 0 if rms publihs, 1 if ma publish, 2 if pump event
{
    const uint16_t len = (uint16_t)strlen(Message);

    const char *topictype = NULL;

    switch (type) {
    case 0: topictype = TESTTOPIC;    break;
    case 1: topictype = TELEM_TOPIC;  break;
    }
    // Send QMTPUBEX (QoS1, retain=0)
    char cmd[256];
    snprintf(cmd, sizeof(cmd),
             "AT+QMTPUBEX=0,%u,1,0,\"%s\",%u\r\n",
             (unsigned)msgid, topictype, (unsigned)len);

    int ok = lte_cmd_expect(cmd,">",2500,huart);
    HAL_Delay(100); // small gap to enter input state

    // Send exactly <len> bytes: NO CR/LF, NO Ctrl+Z (QMTPUBEX is length-delimited)
    HAL_UART_Transmit(huart, (uint8_t*)Message, len, 2000 + len);

    // Short read window to see result URC: +QMTPUBEX: 0,<msgid>,0 (success)
    char resp[512] = {0};
    lte_read_window(huart, resp, sizeof(resp), 4000);

    // Optional: scan resp for +QMTPUBEX: 0,<msgid>,0
    // Optional: if resp contains "+QMTSTAT: 0,1", reconnect before next publish.

    msgid = (msgid == 65) ? 1 : (msgid + 1);
    return (strstr(resp, "+QMTPUBEX: 0,") && strstr(resp, ",0")); // naive check
}


