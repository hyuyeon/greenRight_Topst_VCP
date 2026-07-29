// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : buzzerTask.h
*
*   Description : Buzzer application task
*
***************************************************************************************************
*/

#ifndef MCU_BSP_BUZZER_TASK_HEADER
#define MCU_BSP_BUZZER_TASK_HEADER

#if (MCU_BSP_SUPPORT_APP_BUZZER == 1)

#include <sal_api.h>

/*
***************************************************************************************************
*                                             DEFINITIONS
***************************************************************************************************
*/
#define BUZZER_OFF                      (0U)
#define BUZZER_ON                       (1U)

/*
***************************************************************************************************
*                                         MODULE INTERFACES
***************************************************************************************************
*/
SALRetCode_t BUZZER_Request
(
    uint8                               state
);

SALRetCode_t BUZZER_RequestFromISR
(
    uint8                               state
);

void BUZZER_TaskCreate
(
    void
);

#endif  // (MCU_BSP_SUPPORT_APP_BUZZER == 1)

#endif  // MCU_BSP_BUZZER_TASK_HEADER
