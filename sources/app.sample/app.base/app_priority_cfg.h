// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : app_priority_cfg.h
*
*   Description :
*       VCP-G 포팅 버전 RTOS Task 우선순위 정의 (설계 제안).
*
*       1차 프로젝트(STM32/CMSIS) 당시 사용하던 4단계 구조
*       (High > Above Normal > Normal > Below Normal)를 그대로 유지하면서,
*       포팅 과정에서 새로 추가된 태스크(PedFlagRxTask, SensorTask, CanTxTask 등)를
*       Normal 등급에 편입했다.
*
*       TCC SAL 우선순위 규약: 숫자가 낮을수록 우선순위가 높다 (sal_com.h 참고).
*
***************************************************************************************************
*/

#ifndef APP_PRIORITY_CFG_H
#define APP_PRIORITY_CFG_H

/* Highest      : CanTxTask - 20ms 주기 Ego 상태 송신 */
#define APP_PRIO_CAN_TX          (2U)

/* High         : CanRxTask - 수신 프레임 배출 및 파싱 */
#define APP_PRIO_CAN_RX          (3U)

/* Normal       : TurnJudgeTask - TTC 계산 및 우회전/비보호좌회전 판단 (gTurnJudgeSem) */
#define APP_PRIO_TURN_JUDGE      (4U)

/* Below Normal : DicisionDisplayTask - LCD 방향/경고 표시 갱신 (gDicisionQueue) */
#define APP_PRIO_DECISION_DISP   (5U)

/* Normal       : PedFlagRxTask, TlDisplayTask, SensorTask
 *                (TurnJudgeTask를 직접 깨우거나(notify), 주기적으로 상태만 갱신하는 태스크들) */
#define APP_PRIO_NORMAL          (4U)

/* Normal(유휴) : App Task Start - 부팅 초기화 후에는 하는 일이 없으므로
 *                다른 Normal 태스크보다 한 단계 낮춰 배경(background)으로 둔다 */
#define APP_PRIO_APP_START       (5U)

/* Below Normal : BuzzerTask - 경고음 재생 (gBuzzerQueue) */
#define APP_PRIO_BUZZER          (6U)

#endif /* APP_PRIORITY_CFG_H */
