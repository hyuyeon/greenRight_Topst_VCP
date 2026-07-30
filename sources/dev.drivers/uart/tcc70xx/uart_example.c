// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : uart_example.c
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/


#include <gpio.h>
#include <uart.h>
#include <uart_example.h>

void UART_Test(void)
{
    uint8 tx[] = "[VCP DBG] UART0 ready\r\n";
    UartParam_t param = {
        .sCh = UART_TEST_CH,
        .sBaudrate = UART_BAUDRATE,
        .sPriority = GIC_PRIORITY_NO_MEAN,
        .sMode = UART_POLLING_MODE,
        .sCtsRts = UART_CTSRTS_OFF,
        .sPortCfg = UART_PORTCFG,
        .sWordLength = WORD_LEN_8,
        .sFIFO = DISABLE_FIFO,
        .s2StopBit = TWO_STOP_BIT_OFF,
        .sParity = PARITY_SPACE,
        .sFnCallback = NULL_PTR
    };

    GPIO_Config(UART_TX_PIN, GPIO_FUNC(UART_PORTCFG));
    GPIO_Config(UART_RX_PIN, GPIO_FUNC(UART_PORTCFG));

    if (UART_Open(&param) == SAL_RET_SUCCESS) {
        (void)UART_Write(UART_TEST_CH, tx, sizeof(tx) - 1UL);
    }
}
