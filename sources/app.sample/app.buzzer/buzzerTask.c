// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : buzzerTask.c
*
*   Description : Buzzer application task
*
***************************************************************************************************
*/

#if (MCU_BSP_SUPPORT_APP_BUZZER == 1)

#include <sal_internal.h>
#include <debug.h>

#include <FreeRTOS.h>
#include <queue.h>

#include "buzzer.h"
#include "buzzerTask.h"

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/
#define BUZZER_TASK_STK_SIZE            (256U)
#define BUZZER_QUEUE_LEN                (1U)
#define BUZZER_DEFAULT_FREQUENCY_HZ     (1000UL)
#define BUZZER_DEFAULT_DURATION_MS      (1000UL)
#define BUZZER_DEFAULT_DUTY_PERCENT     (50UL)

/*
***************************************************************************************************
*                                             VARIABLES
***************************************************************************************************
*/
static QueueHandle_t                    gBuzzerQueue = NULL;

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static void BUZZER_Task
(
    void *                              pArgs
);

/*
***************************************************************************************************
*                                             FUNCTIONS
***************************************************************************************************
*/
static void BUZZER_Task
(
    void *                              pArgs
)
{
    uint8 state;
    SALRetCode_t ret;

    (void)pArgs;

    while (1)
    {
        if (xQueueReceive(gBuzzerQueue, &state, portMAX_DELAY) == pdTRUE)
        {
            if (state == BUZZER_ON)
            {
                ret = Buzzer_StartToneForce
                (
                    BUZZER_DEFAULT_FREQUENCY_HZ,
                    BUZZER_DEFAULT_DURATION_MS,
                    BUZZER_DEFAULT_DUTY_PERCENT
                );
            }
            else
            {
                ret = Buzzer_Stop();
            }

            if (ret != SAL_RET_SUCCESS)
            {
                mcu_printf("\nBuzzer command failed: state=%d, ret=%d\n",
                           (sint32)state,
                           (sint32)ret);
            }
        }
    }
}

SALRetCode_t BUZZER_Request
(
    uint8                               state
)
{
    BaseType_t result;
    SALRetCode_t ret = SAL_RET_FAILED;

    if ((gBuzzerQueue != NULL)
        && ((state == BUZZER_ON) || (state == BUZZER_OFF)))
    {
        result = xQueueOverwrite(gBuzzerQueue, &state);

        if (result == pdPASS)
        {
            ret = SAL_RET_SUCCESS;
        }
    }

    return ret;
}

SALRetCode_t BUZZER_RequestFromISR
(
    uint8                               state
)
{
    BaseType_t result;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    SALRetCode_t ret = SAL_RET_FAILED;

    if ((gBuzzerQueue != NULL)
        && ((state == BUZZER_ON) || (state == BUZZER_OFF)))
    {
        result = xQueueOverwriteFromISR
        (
            gBuzzerQueue,
            &state,
            &higherPriorityTaskWoken
        );

        if (result == pdPASS)
        {
            portYIELD_FROM_ISR(higherPriorityTaskWoken);
            ret = SAL_RET_SUCCESS;
        }
    }

    return ret;
}


void BUZZER_TaskCreate
(
    void
)
{
    static uint32 BuzzerTaskID;
    static uint32 BuzzerTaskStk[BUZZER_TASK_STK_SIZE];
    SALRetCode_t ret;

    if (gBuzzerQueue != NULL)
    {
        return;
    }

    gBuzzerQueue = xQueueCreate(BUZZER_QUEUE_LEN, sizeof(uint8));
    if (gBuzzerQueue == NULL)
    {
        mcu_printf("\nBuzzer queue create failed!\n");
        return;
    }

    ret = Buzzer_Init();
    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("\nBuzzer init failed: ret=%d\n", (sint32)ret);
        vQueueDelete(gBuzzerQueue);
        gBuzzerQueue = NULL;
        return;
    }

    ret = (SALRetCode_t)SAL_TaskCreate
    (
        &BuzzerTaskID,
        (const uint8 *)"BuzzerTask",
        (SALTaskFunc)&BUZZER_Task,
        (void * const)&BuzzerTaskStk[0],
        BUZZER_TASK_STK_SIZE,
        SAL_PRIO_KEY_APP,
        NULL_PTR
    );

    if (ret != SAL_RET_SUCCESS)
    {
        mcu_printf("\nBuzzer task create failed: ret=%d\n", (sint32)ret);
        vQueueDelete(gBuzzerQueue);
        gBuzzerQueue = NULL;
    }
}

#endif  // (MCU_BSP_SUPPORT_APP_BUZZER == 1)
