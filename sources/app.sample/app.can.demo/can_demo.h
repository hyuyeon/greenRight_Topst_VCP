// SPDX-License-Identifier: Apache-2.0

/*
***************************************************************************************************
*
*   FileName : can_demo.h
*
*   Copyright (c) Telechips Inc.
*
*   Description :
*
*
***************************************************************************************************
*/

#ifndef MCU_BSP_CAN_DEMO_HEADER
#define MCU_BSP_CAN_DEMO_HEADER

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

/**************************************************************************************************
*                                           INCLUDE FILES
**************************************************************************************************/

#include <can_config.h>
#include <can.h>


/**************************************************************************************************
*                                            DEFINITIONS
**************************************************************************************************/

#define CAN_DEMO_TASK_STK_SIZE          (2048UL)
#define CAN_MAX_TEST_MSG_NUM            (9UL)
#define CAN_DEMO_RX_BATCH_MAX           (32UL)
#define CAN_DEMO_TX_TASK_STK_SIZE       (1024UL)
#define CAN_DEMO_TX_CHANNEL             (0U)
#define CAN_DEMO_TX_ID                  (0x200UL)
#define CAN_DEMO_TX_PERIOD_MS           (50UL)  /* Change to 10UL for a 10 ms period. */
#define CAN_DEMO_STATUS_PERIOD_MS       (1000UL)
#define CAN_DEMO_FRAME_LOG_ENABLE       (1U)
#define CAN_DEMO_RX_EVENT_MASK          (( 1UL << CAN_CONTROLLER_NUMBER ) - 1UL)
#define CAN_DEMO_RX_EVENT( ch )         (1UL << ( ch ))
#define CAN_DEMO_NUCLEO_RX_CHANNEL      (0U)
#define CAN_DEMO_NUCLEO_RX_ID           (0x100UL)

//#define CAN_DEMO_RESPONSE_TEST          //for CAN response test

typedef struct CANDemoTestInfo
{
    uint8                               tiRecv;
    uint8                               tiSendRecv;
} CANDemoTestInfo_t;


/**************************************************************************************************
*                                          LOCAL VARIABLES
**************************************************************************************************/


/**************************************************************************************************
*                                        FUNCTION PROTOTYPES
**************************************************************************************************/

sint32 CAN_DemoInitialize
(
    void
);

void CAN_DemoTest
(
    uint8                               ucArgc,
    void *                              pArgv[]
);

void CAN_DemoCreateApp
(
    void
);

void CAN_DemoSetPeriodicTxData
(
    const uint8 *                       pucData,
    uint8                               ucLength
);

#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#endif  // MCU_BSP_CAN_DEMO_HEADER

