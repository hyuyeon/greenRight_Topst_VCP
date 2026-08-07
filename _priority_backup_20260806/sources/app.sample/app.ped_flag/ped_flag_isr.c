// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : ped_flag_isr.c
*
*   Description :
*       AI-G(카메라) 보드가 UART로 보내는 보행자 감지 플래그(pedFlag)를 수신.
*
*       원본 STM32 버전(ped_task_isr.c, PedTaskISR)과 동일한 동작을 목표로 함:
*         - "상태가 바뀔 때만" 1바이트를 보낸다고 가정 (edge-triggered)
*         - pedFlag 값:
*             0 = 보행자 없음
*             1 = 보행자 있음
*             2 = 카메라 인식 오류
*         - 전역 pedFlag(common.h/main.c에 이미 정의됨)를 받은 값으로 갱신
*         - 전역 maneuver(common.h/main.c에 이미 정의됨)가 MANEUVER_RIGHT_TURN이고,
*           값이 실제로 바뀐 경우에만 TurnJudge_Notify()를 호출해 TurnJudgeTask를 깨움
*         - 그 외 maneuver 상황이면 pedFlag 갱신만 하고 아무 것도 안 함
*
*       [이 브랜치 전용] pedFlag/maneuver는 이 파일에서 정의하지 않는다 (main.c에 이미 있음).
*       세마포어도 이 파일에서 만들지 않는다 (TurnJudgeTask가 자체 세마포어를 갖고
*       TurnJudge_Notify() API로 깨우는 구조이기 때문).
*
*       AI-G UART 링크: UART_CH0 (GPIO_A28=TX, GPIO_A29=RX). 이 프로젝트에서 UART_CH0을
*       쓰는 다른 모듈이 없음을 확인함.
*
*       LED 테스트 핀: GPIO_GPC(1). 이 보드에서 실측으로 확인된, 어떤 모듈과도 안 겹치는
*       범용 GPIO 헤더 핀. (참고: GPIO_GPB(2)는 BNO055 I2C1의 SCL과 겹치고,
*       GPIO_GPK(2)는 이 보드에서 CAN 통신용으로 브레이크아웃되어 있어 둘 다 쓰면 안 됨.)
*
***************************************************************************************************
*/

#include <uart.h>
#include <sal_api.h>
#include <debug.h>
#include <gic.h>
#include <gpio.h>
#include "ped_flag_isr.h"
#include "common.h"
#include "turnJudgeTask.h"

#define PED_FLAG_UART_CH        UART_CH0
#define PED_FLAG_UART_PORTCFG   0U
#define PED_FLAG_BAUDRATE       115200U
#define PED_FLAG_TASK_STK_SIZE  256U
#define PED_FLAG_RX_BUF_SIZE    16U
#define PED_FLAG_TEST_LED_PIN   GPIO_GPC(1)

/* 0으로 바꾸면 디버그 카운터 관련 코드가 전부 빠짐 (원본 STM32의 PED_DEBUG 와 동일한 용도) */
#define PED_DEBUG                1

#if (PED_DEBUG == 1)
static volatile uint32 sIrqCount         = 0U;
static volatile uint32 sRxByteCount      = 0U;
static volatile uint32 sValidByteCount   = 0U;
static volatile uint32 sInvalidByteCount = 0U;
static volatile uint32 sNotifyCount      = 0U;
static volatile uint8  sLastRawByte      = 0U;
static volatile uint8  sLastDecodedFlag  = 0U;
#endif

static UartParam_t     sPedFlagParam;
static uint32           sPedFlagTaskID = 0;
static uint32           sPedFlagTaskStk[PED_FLAG_TASK_STK_SIZE];

/* ai_link.c 호환용 상태 (AI_Link_GetFlag / AI_Link_HasNewFlag 용) */
static volatile uint8   sAiLinkLatestFlag = 0U;
static volatile boolean sAiLinkHasNewFlag = FALSE;

static boolean DecodeFlag(uint8 raw, uint8 *pOut);
static void    PedFlag_RxTask(void *pArg);

/*
***************************************************************************************************
*                                          DecodeFlag
*
*   원본 STM32 UART4_IRQHandler 의 디코딩 규칙과 동일:
*     raw <= 2               -> 그대로 사용 (바이너리 0/1/2)
*     raw가 아스키 '0'~'2'   -> 숫자로 변환
*     raw == 0xFF             -> 2 (에러) 로 처리
*     그 외                   -> 무효한 바이트
***************************************************************************************************
*/
static boolean DecodeFlag(uint8 raw, uint8 *pOut)
{
    boolean ok = TRUE;

    if (raw <= 2U)
    {
        *pOut = raw;
    }
    else if ((raw >= (uint8)'0') && (raw <= (uint8)'2'))
    {
        *pOut = (uint8)(raw - (uint8)'0');
    }
    else if (raw == 0xFFU)
    {
        *pOut = 2U;
    }
    else
    {
        ok = FALSE;
    }

    return ok;
}

/*
***************************************************************************************************
*                                          PedFlag_RxTask
*
*   AI-G로부터 받은 바이트를 처리하는 백그라운드 태스크.
*   (실제 하드웨어 인터럽트는 벤더 UART 드라이버의 UART_ISR 이 전담하고, 이 태스크는
*    UART_Read() 로 드라이버 내부 버퍼에서 꺼내오는 구조 -> 태스크 컨텍스트에서 동작)
***************************************************************************************************
*/
static void PedFlag_RxTask(void *pArg)
{
    uint8  rxBuf[PED_FLAG_RX_BUF_SIZE];
    sint32 n;
    static uint8   hasPedSample = 0U;
    static uint8   prevPedFlag  = 0U;

    (void)pArg;

    for (;;)
    {
        n = UART_Read(PED_FLAG_UART_CH, rxBuf, PED_FLAG_RX_BUF_SIZE);

        if (n > 0)
        {
            uint8 received = rxBuf[n - 1];
            uint8 decoded;

#if (PED_DEBUG == 1)
            sIrqCount++;
            sRxByteCount += (uint32)n;
            sLastRawByte = received;
#endif

            if (DecodeFlag(received, &decoded) == TRUE)
            {
                boolean shouldJudge = (boolean)((hasPedSample == 0U) || (decoded != prevPedFlag));

                pedFlag      = decoded;   /* main.c에 정의된 전역 갱신 */
                prevPedFlag  = decoded;
                hasPedSample = 1U;

                /* ai_link.c 호환: 최신 플래그 + "새 값 도착" 표시 */
                sAiLinkLatestFlag = decoded;
                sAiLinkHasNewFlag = shouldJudge;

                /* LED 테스트 표시 (1=켜짐, 0=꺼짐, 2=에러는 이전 상태 유지) */
                if (decoded == 1U)
                {
                    (void)GPIO_Set(PED_FLAG_TEST_LED_PIN, 1U);
                }
                else if (decoded == 0U)
                {
                    (void)GPIO_Set(PED_FLAG_TEST_LED_PIN, 0U);
                }
                else
                {
                    ; /* 2 = AI 인식 오류: LED 상태는 그대로 둠 */
                }

#if (PED_DEBUG == 1)
                sValidByteCount++;
                sLastDecodedFlag = decoded;
#endif
                mcu_printf("PedFlag: raw=0x%02X decoded=%d (changed=%d)\n",
                           received, (int)decoded, (int)shouldJudge);

                /* 값이 실제로 바뀌었고, 지금이 우회전 상황일 때만 TurnJudgeTask를 깨움 */
                if ((shouldJudge == TRUE) && (maneuver == (uint8_t)MANEUVER_RIGHT_TURN))
                {
                    TurnJudge_Notify();

#if (PED_DEBUG == 1)
                    sNotifyCount++;
#endif
                    mcu_printf("PedFlag: TurnJudge_Notify() called (maneuver=RIGHT_TURN, decoded=%d)\n",
                               (int)decoded);
                }
                /* 우회전 상황이 아니면: pedFlag 갱신만 하고 별도 동작 없음 */
            }
            else
            {
#if (PED_DEBUG == 1)
                sInvalidByteCount++;
#endif
            }
        }

        (void)SAL_TaskSleep(10);
    }
}

/*
***************************************************************************************************
*                                          PedFlag_Init
***************************************************************************************************
*/
void PedFlag_Init(void)
{
    sPedFlagParam.sCh          = PED_FLAG_UART_CH;
    sPedFlagParam.sPriority    = GIC_PRIORITY_NO_MEAN;
    sPedFlagParam.sBaudrate    = PED_FLAG_BAUDRATE;
    sPedFlagParam.sMode        = UART_INTR_MODE;
    sPedFlagParam.sCtsRts      = UART_CTSRTS_OFF;
    sPedFlagParam.sPortCfg     = PED_FLAG_UART_PORTCFG;
    sPedFlagParam.sWordLength  = WORD_LEN_8;
    sPedFlagParam.sFIFO        = ENABLE_FIFO;
    sPedFlagParam.s2StopBit    = TWO_STOP_BIT_OFF;
    sPedFlagParam.sParity      = PARITY_SPACE;
    sPedFlagParam.sFnCallback  = (GICIsrFunc)&UART_ISR;

    UART_Close(PED_FLAG_UART_CH);
    (void)UART_Open(&sPedFlagParam);

    /* LED 테스트 핀 초기화 */
    (void)GPIO_Config(PED_FLAG_TEST_LED_PIN, GPIO_FUNC(0) | GPIO_OUTPUT);
    (void)GPIO_Set(PED_FLAG_TEST_LED_PIN, 0U);

    (void)SAL_TaskCreate(&sPedFlagTaskID, (const uint8 *)"PedFlag_RxTask", (SALTaskFunc)&PedFlag_RxTask,
                          &sPedFlagTaskStk[0], PED_FLAG_TASK_STK_SIZE, SAL_PRIO_APP_CFG, NULL);

    mcu_printf("PedFlag: UART_CH0 opened (interrupt mode), LED test on GPIO_GPC(1)\n");
}

/*
***************************************************************************************************
*   ai_link.c 호환용 API
***************************************************************************************************
*/
uint8 AI_Link_GetFlag(void)
{
    sAiLinkHasNewFlag = FALSE;
    return sAiLinkLatestFlag;
}

boolean AI_Link_HasNewFlag(void)
{
    return sAiLinkHasNewFlag;
}

void PedFlag_GetDebugSnapshot(PedFlagDebugSnapshot_t *pSnapshot)
{
    if (pSnapshot == NULL_PTR)
    {
        return;
    }

#if (PED_DEBUG == 1)
    pSnapshot->irqCount         = sIrqCount;
    pSnapshot->rxByteCount      = sRxByteCount;
    pSnapshot->validByteCount   = sValidByteCount;
    pSnapshot->invalidByteCount = sInvalidByteCount;
    pSnapshot->notifyCount      = sNotifyCount;
    pSnapshot->lastRawByte      = sLastRawByte;
    pSnapshot->lastDecodedFlag  = sLastDecodedFlag;
#else
    pSnapshot->irqCount         = 0U;
    pSnapshot->rxByteCount      = 0U;
    pSnapshot->validByteCount   = 0U;
    pSnapshot->invalidByteCount = 0U;
    pSnapshot->notifyCount      = 0U;
    pSnapshot->lastRawByte      = 0U;
    pSnapshot->lastDecodedFlag  = 0U;
#endif
}
