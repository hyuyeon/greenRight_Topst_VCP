#include <stddef.h>

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "common.h"
#include "lcd.h"
#include "display_task.h"
#include "app_priority_cfg.h"

/* -------------------------------------------------------------------------- */
/* Task resources                                                             */
/* -------------------------------------------------------------------------- */

static uint32 gTlDisplayTaskId;
static uint32 gDicisionDisplayTaskId;

static uint32 gTlDisplayTaskStack[ACFG_TASK_MEDIUM_STK_SIZE];
static uint32 gDicisionDisplayTaskStack[ACFG_TASK_MEDIUM_STK_SIZE];

static SemaphoreHandle_t gTlDisplaySem;
static SemaphoreHandle_t gLcdMutex;
static QueueHandle_t gDicisionQueue;

static uint8_t gDisplayCreated;

/*
 * 기존 TurnJudgeTask에서 extern TrafficLight tl로 참조할 수 있도록
 * 전역 객체로 유지한다.
 */


/* -------------------------------------------------------------------------- */
/* Internal functions                                                         */
/* -------------------------------------------------------------------------- */

static void TlDisplayTask(void *pArg);
static void DicisionDisplayTask(void *pArg);

static void Display_LockLcd(void)
{
    if (gLcdMutex != NULL)
    {
        (void)xSemaphoreTake(gLcdMutex, portMAX_DELAY);
    }
}

static void Display_UnlockLcd(void)
{
    if (gLcdMutex != NULL)
    {
        (void)xSemaphoreGive(gLcdMutex);
    }
}

static TurnDirection Dicision_ToTurnDirection(const Dicision *decision)
{
    TurnDirection direction;

    if (decision == NULL)
    {
        return DIR_ERROR;
    }

    if ((decision->dataStatus & DECISION_DATA_STATUS_MQTT_COMM_ERROR) != 0U)
    {
        return DIR_ERROR;
    }

    switch (decision->turnState)
    {
        case MANEUVER_RIGHT_TURN:
            direction = DIR_RIGHT;
            break;

        case MANEUVER_LEFT_TURN_UNPROT:
        case MANEUVER_LEFT_TURN_PROT:
            direction = DIR_LEFT;
            break;

        case MANEUVER_STRAIGHT:
        default:
            direction = DIR_STRAIGHT;
            break;
    }

    return direction;
}

static uint8_t Dicision_ToWarningMask(const Dicision *decision)
{
    uint8_t mask = WARN_NONE;

    if (decision == NULL)
    {
        return WARN_COMM_ERROR;
    }

    if ((decision->dataStatus & DECISION_DATA_STATUS_MQTT_COMM_ERROR) != 0U)
    {
        mask |= WARN_COMM_ERROR;
    }

    /*
     * pedestrianFlag
     * 0: 보행자 없음
     * 1: 보행자 감지
     * 2: AI 인식 불가
     *
     * 1과 2 모두 LCD에 보행자 항목을 띄우고,
     * 실제 문구는 lcd.c에서 pedestrianFlag로 구분한다.
     */
    if (decision->pedestrianFlag != 0U)
    {
        mask |= WARN_PEDESTRIAN;
    }

    switch (decision->turnState)
    {
        case MANEUVER_RIGHT_TURN:
            if (decision->LStraightFlag != 0U)
            {
                mask |= WARN_STRAIGHT_VEHICLE;
            }

            if (decision->OppLeftFlag != 0U)
            {
                mask |= WARN_OPPOSITE_TURN;
            }
            break;

        case MANEUVER_LEFT_TURN_UNPROT:
            if (decision->OppStraightFlag != 0U)
            {
                mask |= WARN_STRAIGHT_VEHICLE;
            }

            if (decision->OppRightFlag != 0U)
            {
                mask |= WARN_OPPOSITE_TURN;
            }

            if (decision->tlWarningFlag != 0U)
            {
                mask |= WARN_TL;
            }
            break;

        case MANEUVER_LEFT_TURN_PROT:
            if (decision->OppStraightFlag != 0U)
            {
                mask |= WARN_STRAIGHT_VEHICLE;
            }

            if (decision->OppRightFlag != 0U)
            {
                mask |= WARN_OPPOSITE_TURN;
            }
            break;

        case MANEUVER_STRAIGHT:
        default:
            break;
    }

    return mask;
}

/* -------------------------------------------------------------------------- */
/* Public data update APIs                                                    */
/* -------------------------------------------------------------------------- */

void Display_GetTrafficLightSnapshot(TrafficLight *outTl)
{
    if (outTl == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *outTl = tl;
    taskEXIT_CRITICAL();
}

void Display_TrafficLightNotify(void)
{
    if (gTlDisplaySem != NULL)
    {
        (void)xSemaphoreGive(gTlDisplaySem);
    }
}

uint8_t Display_DicisionPost(const Dicision *newDecision)
{
    BaseType_t result;

    if ((newDecision == NULL) || (gDicisionQueue == NULL))
    {
        return 0U;
    }

    /*
     * DICISION_QUEUE_LEN이 1이므로 이전 결과가 남아 있으면
     * 가장 최신 판단 결과로 덮어쓴다.
     */
    result = xQueueOverwrite(gDicisionQueue, newDecision);

    return (result == pdPASS) ? 1U : 0U;
}

/* -------------------------------------------------------------------------- */
/* Display tasks                                                              */
/* -------------------------------------------------------------------------- */

static void TlDisplayTask(void *pArg)
{
    TrafficLight tlSnapshot;
    SignalColor signalColor;

    (void)pArg;

    for (;;)
    {
        if (xSemaphoreTake(gTlDisplaySem, portMAX_DELAY) == pdTRUE)
        {
            Display_GetTrafficLightSnapshot(&tlSnapshot);

            /*
             * 0, 1, 2는 SIG_RED/YELLOW/GREEN과 동일하다.
             * 255는 신호등 없음으로 전달되며, lcd.c에서는 세 점을 모두
             * 비활성 색상으로 출력한다.
             */
            signalColor = (SignalColor)tlSnapshot.color;

            Display_LockLcd();

            Dashboard_DrawSignalDots(signalColor);
            Dashboard_UpdateCountdown(tlSnapshot.time_left);

            Display_UnlockLcd();
        }
    }
}

static void DicisionDisplayTask(void *pArg)
{
    Dicision decision;
    TurnDirection direction;
    uint8_t warningMask;

    (void)pArg;

    for (;;)
    {
        if (xQueueReceive(
                gDicisionQueue,
                &decision,
                portMAX_DELAY) == pdTRUE)
        {
            direction = Dicision_ToTurnDirection(&decision);
            warningMask = Dicision_ToWarningMask(&decision);

            Display_LockLcd();

            Dashboard_DrawDirection(direction, decision.dataStatus);
            Dashboard_DrawWarnings(
                direction,
                warningMask,
                decision.pedestrianFlag
            );

            Display_UnlockLcd();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Task creation                                                              */
/* -------------------------------------------------------------------------- */

void Display_AppCreate(void)
{
    SALRetCode_t tlTaskResult;
    SALRetCode_t decisionTaskResult;

    if (gDisplayCreated != 0U)
    {
        return;
    }

    gLcdMutex = xSemaphoreCreateMutex();
    gTlDisplaySem = xSemaphoreCreateBinary();
    gDicisionQueue = xQueueCreate(
        DICISION_QUEUE_LEN,
        sizeof(Dicision)
    );

    if ((gLcdMutex == NULL) ||
        (gTlDisplaySem == NULL) ||
        (gDicisionQueue == NULL))
    {
        mcu_printf(
            "\n[DISPLAY] Sync object creation failed"
            " mutex=%p sem=%p queue=%p\n",
            gLcdMutex,
            gTlDisplaySem,
            gDicisionQueue
        );

        return;
    }

    /* 초기 신호등 상태: 참조 신호등 없음 */


    /*
     * main.c의 목데이터 대신 실제 DisplayTask를 사용할 때 표시할
     * 기본 초기 화면.
     */
    Display_LockLcd();

    Dashboard_DrawStatic(
        DIR_STRAIGHT,
        WARN_NONE,
        SIG_RED,
        0U,
        0U
    );

    Display_UnlockLcd();

    tlTaskResult = (SALRetCode_t)SAL_TaskCreate(
        &gTlDisplayTaskId,
        (const uint8 *)"TL Display",
        (SALTaskFunc)&TlDisplayTask,
        &gTlDisplayTaskStack[0],
        ACFG_TASK_MEDIUM_STK_SIZE,
        APP_PRIO_NORMAL,        /* Normal: 신호등 LCD 갱신 - 우선순위 설계 제안 */
        NULL
    );

    decisionTaskResult = (SALRetCode_t)SAL_TaskCreate(
        &gDicisionDisplayTaskId,
        (const uint8 *)"Decision Display",
        (SALTaskFunc)&DicisionDisplayTask,
        &gDicisionDisplayTaskStack[0],
        ACFG_TASK_MEDIUM_STK_SIZE,
        APP_PRIO_DECISION_DISP, /* Above Normal: 방향/경고 LCD 갱신 - 우선순위 설계 제안 */
        NULL
    );

    if ((tlTaskResult == SAL_RET_SUCCESS) &&
        (decisionTaskResult == SAL_RET_SUCCESS))
    {
        gDisplayCreated = 1U;

        mcu_printf("\n[DISPLAY] Tasks created\n");
    }
    else
    {
        mcu_printf(
            "\n[DISPLAY] Task creation failed: TL=%d DEC=%d\n",
            (sint32)tlTaskResult,
            (sint32)decisionTaskResult
        );
    }
}
