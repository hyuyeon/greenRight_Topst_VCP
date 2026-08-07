// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : ped_flag_isr.h
*
*   Description :
*       AI-G(카메라) 보드로부터 UART로 들어오는 보행자 감지 플래그를 수신하는 모듈.
*       STM32(Nucleo-F429ZI) 버전의 ped_task_isr.c / personDetect.h 를 VCP-G(SAL/FreeRTOS) 로 포팅.
*
*       [이 브랜치(feature-sensor-integration) 전용 통합 방식]
*       팀의 최종 통합 코드에서는 pedFlag/maneuver가 이미 app.base/main.c에 전역으로 정의되어
*       있고(common.h에 extern 선언), TurnJudgeTask도 이미 포팅되어 있어 자체 세마포어
*       (gTurnJudgeSem, static)를 갖고 TurnJudge_Notify() 라는 API로 깨우는 구조이다.
*       그래서 이 모듈은 (예전 단독 브랜치와 달리) pedFlag/maneuver/세마포어를 자체적으로
*       만들지 않고, common.h의 pedFlag를 갱신하고 turnJudgeTask.h의 TurnJudge_Notify()를
*       호출하기만 한다.
*
***************************************************************************************************
*/

#ifndef PED_FLAG_ISR_HEADER
#define PED_FLAG_ISR_HEADER

#include <sal_internal.h>

typedef struct
{
    uint32 irqCount;
    uint32 rxByteCount;
    uint32 validByteCount;
    uint32 invalidByteCount;
    uint32 notifyCount;
    uint8  lastRawByte;
    uint8  lastDecodedFlag;
} PedFlagDebugSnapshot_t;

void PedFlag_Init(void);
void PedFlag_GetDebugSnapshot(PedFlagDebugSnapshot_t *pSnapshot);

/* ai_link.c 호환용 API (예전 브랜치에서 넘어온 코드가 참조해도 동작하도록 유지) */
uint8   AI_Link_GetFlag(void);
boolean AI_Link_HasNewFlag(void);

#endif /* PED_FLAG_ISR_HEADER */
