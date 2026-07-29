// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : buzzer.c
*
*   Description : Buzzer driver using PDM and a hardware timer
*
***************************************************************************************************
*/

#if (MCU_BSP_SUPPORT_APP_BUZZER == 1)

#include <sal_internal.h>

#include <gpio.h>
#include <pdm.h>
#include <timer.h>
#include <gic.h>

#include "buzzer.h"
#include "buzzerTask.h"

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/
#define BUZZER_PDM_CHANNEL              ((uint32)PDM_CHANNEL_0)
#define BUZZER_PDM_PORT                 ((uint32)GPIO_PERICH_CH0)
#define BUZZER_DURATION_TIMER_CHANNEL   (TIMER_CH_8)
#define BUZZER_NANOSECONDS_PER_SECOND   (1000000000UL)
#define BUZZER_MICROSECONDS_PER_MS      (1000UL)
#define BUZZER_MAX_DURATION_MS          (0xFFFFFFFFUL / BUZZER_MICROSECONDS_PER_MS)
#define BUZZER_PDM_IDLE_TIMEOUT_MS      (100UL)

/*
***************************************************************************************************
*                                             VARIABLES
***************************************************************************************************
*/
static PDMModeConfig_t                  gBuzzerPdmConfig;
static boolean                          gBuzzerInitialized = FALSE;

/*
***************************************************************************************************
*                                         FUNCTION PROTOTYPES
***************************************************************************************************
*/
static SALRetCode_t Buzzer_WaitForPdmIdle
(
    void
);

static sint32 Buzzer_TimerHandler
(
    TIMERChannel_t                      channel,
    void *                              args
);

/*
***************************************************************************************************
*                                             FUNCTIONS
***************************************************************************************************
*/
static SALRetCode_t Buzzer_WaitForPdmIdle
(
    void
)
{
    uint32 waitCount;
    SALRetCode_t ret = SAL_RET_FAILED;

    for (waitCount = 0U; waitCount < BUZZER_PDM_IDLE_TIMEOUT_MS; waitCount++)
    {
        if (PDM_GetChannelStatus(BUZZER_PDM_CHANNEL) == 0UL)
        {
            ret = SAL_RET_SUCCESS;
            break;
        }

        (void)SAL_TaskSleep(1U);
    }

    return ret;
}

static sint32 Buzzer_TimerHandler
(
    TIMERChannel_t                      channel,
    void *                              args
)
{
    (void)channel;
    (void)args;

    (void)BUZZER_RequestFromISR(BUZZER_OFF);

    return (sint32)SAL_RET_SUCCESS;
}

SALRetCode_t Buzzer_Init
(
    void
)
{
    (void)SAL_MemSet(&gBuzzerPdmConfig, 0, sizeof(PDMModeConfig_t));

    PDM_Init();

    gBuzzerPdmConfig.mcPortNumber       = BUZZER_PDM_PORT;
    gBuzzerPdmConfig.mcOperationMode    = PDM_OUTPUT_MODE_PHASE_1;
    gBuzzerPdmConfig.mcInversedSignal   = 0UL;
    gBuzzerPdmConfig.mcOutSignalInIdle  = 0UL;
    gBuzzerPdmConfig.mcLoopCount        = 0UL;
    gBuzzerPdmConfig.mcOutputCtrl       = 0UL;
    gBuzzerPdmConfig.mcPeriodNanoSec1   = 1000000UL;
    gBuzzerPdmConfig.mcDutyNanoSec1     = 500000UL;
    gBuzzerPdmConfig.mcPeriodNanoSec2   = 0UL;
    gBuzzerPdmConfig.mcDutyNanoSec2     = 0UL;

    gBuzzerInitialized = TRUE;

    return SAL_RET_SUCCESS;
}

SALRetCode_t Buzzer_StartToneForce
(
    uint32                              frequencyHz,
    uint32                              durationMs,
    uint32                              dutyPercent
)
{
    uint32 periodNs;
    uint32 dutyNs;
    uint32 durationUs;
    uint64 dutyCalculation;
    SALRetCode_t ret = SAL_RET_FAILED;

    if ((gBuzzerInitialized == FALSE)
        || (frequencyHz == 0UL)
        || (frequencyHz > BUZZER_NANOSECONDS_PER_SECOND)
        || (durationMs == 0UL)
        || (durationMs > BUZZER_MAX_DURATION_MS)
        || (dutyPercent > 100UL))
    {
        return SAL_RET_FAILED;
    }

    periodNs = BUZZER_NANOSECONDS_PER_SECOND / frequencyHz;
    dutyCalculation = (uint64)periodNs * (uint64)dutyPercent;
    dutyNs = (uint32)(dutyCalculation / 100UL);
    durationUs = durationMs * BUZZER_MICROSECONDS_PER_MS;

    (void)TIMER_Disable(BUZZER_DURATION_TIMER_CHANNEL);
    (void)TIMER_InterruptClear(BUZZER_DURATION_TIMER_CHANNEL);

    (void)PDM_Disable(BUZZER_PDM_CHANNEL, PMM_ON);

    ret = Buzzer_WaitForPdmIdle();
    if (ret != SAL_RET_SUCCESS)
    {
        return ret;
    }

    gBuzzerPdmConfig.mcPeriodNanoSec1 = periodNs;
    gBuzzerPdmConfig.mcDutyNanoSec1   = dutyNs;

    ret = PDM_SetConfig(BUZZER_PDM_CHANNEL, &gBuzzerPdmConfig);
    if (ret != SAL_RET_SUCCESS)
    {
        return ret;
    }

    ret = PDM_Enable(BUZZER_PDM_CHANNEL, PMM_ON);
    if (ret != SAL_RET_SUCCESS)
    {
        return ret;
    }

    ret = TIMER_EnableWithMode
    (
        BUZZER_DURATION_TIMER_CHANNEL,
        durationUs,
        TIMER_OP_ONESHOT,
        Buzzer_TimerHandler,
        NULL_PTR
    );

    if (ret == SAL_RET_SUCCESS)
    {
        ret = GIC_IntSrcEn
        (
            (uint32)GIC_TIMER_0 + (uint32)BUZZER_DURATION_TIMER_CHANNEL
        );
    }

    if (ret != SAL_RET_SUCCESS)
    {
        (void)TIMER_Disable(BUZZER_DURATION_TIMER_CHANNEL);
        (void)PDM_Disable(BUZZER_PDM_CHANNEL, PMM_ON);
    }

    return ret;
}

SALRetCode_t Buzzer_Stop
(
    void
)
{
    SALRetCode_t ret;

    if (gBuzzerInitialized == FALSE)
    {
        return SAL_RET_FAILED;
    }

    (void)TIMER_Disable(BUZZER_DURATION_TIMER_CHANNEL);
    (void)TIMER_InterruptClear(BUZZER_DURATION_TIMER_CHANNEL);

    ret = PDM_Disable(BUZZER_PDM_CHANNEL, PMM_ON);

    return ret;
}

#endif  // (MCU_BSP_SUPPORT_APP_BUZZER == 1)
