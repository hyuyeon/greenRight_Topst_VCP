// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : buzzer.h
*
*   Description : Buzzer driver using PDM and a hardware timer
*
***************************************************************************************************
*/

#ifndef MCU_BSP_BUZZER_HEADER
#define MCU_BSP_BUZZER_HEADER

#if (MCU_BSP_SUPPORT_APP_BUZZER == 1)

#include <sal_api.h>

SALRetCode_t Buzzer_Init
(
    void
);

SALRetCode_t Buzzer_StartToneForce
(
    uint32                              frequencyHz,
    uint32                              durationMs,
    uint32                              dutyPercent
);

SALRetCode_t Buzzer_Stop
(
    void
);

#endif  // (MCU_BSP_SUPPORT_APP_BUZZER == 1)

#endif  // MCU_BSP_BUZZER_HEADER
