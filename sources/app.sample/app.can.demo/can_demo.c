// SPDX-License-Identifier: Apache-2.0

#if ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )

#include <app_cfg.h>
#include <stdio.h>

#include "bsp.h"
#include "gpio.h"
#include "debug.h"
#include "can_config.h"
#include "can.h"
#include "can_reg.h"
#include "can_drv.h"
#include "can_porting.h"
#include "can_demo.h"

static volatile uint8 gCanDemoRxEnabled;
static volatile uint8 gCanDemoTxEnabled;
static sint32 gCanDemoInitResult = -1;

static uint32 gCanDemoRxEvent;
static uint8 gCanDemoRxEventCreated;
static uint32 gCanDemoTxDataLock;
static uint8 gCanDemoTxDataLockCreated;
static uint32 gCanDemoLogLock;
static uint8 gCanDemoLogLockCreated;
static uint8 gCanDemoPeriodicTxData[8];

static volatile uint32 gCanDemoRxCount[CAN_CONTROLLER_NUMBER];
static volatile uint32 gCanDemoRxCallbackCount[CAN_CONTROLLER_NUMBER];
static volatile uint32 gCanDemoRxDropCount[CAN_CONTROLLER_NUMBER];
static volatile uint32 gCanDemoRxLastError[CAN_CONTROLLER_NUMBER];
static volatile uint32 gCanDemoTxRequestCount;
static volatile uint32 gCanDemoTxCompleteCount;
static volatile uint32 gCanDemoTxErrorCount;
static volatile uint32 gCanDemoLastRxId;
static volatile uint32 gCanDemoLastTxId;

static CANMessage_t gCanDemoRxBatch[CAN_CONTROLLER_NUMBER][CAN_DEMO_RX_BATCH_MAX];

static void CAN_DemoRxTask(void *pArg);
static void CAN_DemoTxTask(void *pArg);

static void CAN_DemoCallbackTxEvent
(
    uint8                               ucCh,
    CANTxInterruptType_t                uiIntType
)
{
    if( ( ucCh == CAN_DEMO_TX_CHANNEL ) &&
        ( uiIntType == CAN_TX_INT_TYPE_TRANSMIT_COMPLETED ) )
    {
        gCanDemoTxCompleteCount++;
    }
}

static void CAN_DemoCallbackRxEvent
(
    uint8                               ucCh,
    uint32                              uiRxIndex,
    CANMessageBufferType_t              uiRxBufferType,
    CANErrorType_t                      uiError
)
{
    ( void )uiRxIndex;
    ( void )uiRxBufferType;

    if( ucCh < CAN_CONTROLLER_NUMBER )
    {
        gCanDemoRxCallbackCount[ucCh]++;

        if( uiError != CAN_ERROR_NONE )
        {
            gCanDemoRxDropCount[ucCh]++;
            gCanDemoRxLastError[ucCh] = ( uint32 )uiError;
        }

        if( gCanDemoRxEventCreated == TRUE )
        {
            ( void )SAL_EventSet( gCanDemoRxEvent,
                                  CAN_DEMO_RX_EVENT( ucCh ),
                                  SAL_EVENT_OPT_FLAG_SET );
        }
    }
}

static void CAN_DemoCallbackErrorEvent
(
    uint8                               ucCh,
    CANErrorType_t                      uiError
)
{
    if( ucCh < CAN_CONTROLLER_NUMBER )
    {
        gCanDemoRxLastError[ucCh] = ( uint32 )uiError;

        if( gCanDemoRxEventCreated == TRUE )
        {
            ( void )SAL_EventSet( gCanDemoRxEvent,
                                  CAN_DEMO_RX_EVENT( ucCh ),
                                  SAL_EVENT_OPT_FLAG_SET );
        }
    }
}

static void CAN_DemoDrainRx
(
    uint8                               ucCh
)
{
    uint32 uiBatchCount;
    uint32 uiBatchIndex;
#if ( CAN_DEMO_FRAME_LOG_ENABLE == 1U )
    uint8 ucDataIndex;
#endif
    CANMessage_t *psRxMsg;

    do
    {
        uiBatchCount = 0UL;

        while( ( uiBatchCount < CAN_DEMO_RX_BATCH_MAX ) &&
               ( CAN_CheckNewRxMessage( ucCh ) > 0UL ) )
        {
            psRxMsg = &gCanDemoRxBatch[ucCh][uiBatchCount];
            if( CAN_GetNewRxMessage( ucCh, psRxMsg ) != CAN_ERROR_NONE )
            {
                break;
            }
            uiBatchCount++;
        }

        for( uiBatchIndex = 0UL; uiBatchIndex < uiBatchCount; uiBatchIndex++ )
        {
            psRxMsg = &gCanDemoRxBatch[ucCh][uiBatchIndex];
            gCanDemoRxCount[ucCh]++;
            gCanDemoLastRxId = psRxMsg->mId;

#if ( CAN_DEMO_FRAME_LOG_ENABLE == 1U )
            if( gCanDemoLogLockCreated == TRUE )
            {
                ( void )SAL_SemaphoreWait( gCanDemoLogLock, 0, SAL_OPT_BLOCKING );
            }

            mcu_printf( "[CAN RX] CH%d SEQ:%d %s %s ID:0x%X DLC:%d DATA:",
                        ucCh,
                        ( unsigned long )gCanDemoRxCount[ucCh],
                        ( psRxMsg->mExtendedId != 0U ) ? "EXT" : "STD",
                        ( psRxMsg->mRemoteTransmitRequest != 0U ) ? "RTR" : "DATA",
                        ( unsigned long )psRxMsg->mId,
                        psRxMsg->mDataLength );

            for( ucDataIndex = 0U;
                 ( psRxMsg->mRemoteTransmitRequest == 0U ) &&
                 ( ucDataIndex < psRxMsg->mDataLength );
                 ucDataIndex++ )
            {
                mcu_printf( " %02X", psRxMsg->mData[ucDataIndex] );
            }
            mcu_printf( "\n" );

            if( gCanDemoLogLockCreated == TRUE )
            {
                ( void )SAL_SemaphoreRelease( gCanDemoLogLock );
            }
#endif
        }
    }
    while( uiBatchCount == CAN_DEMO_RX_BATCH_MAX );
}

sint32 CAN_DemoInitialize
(
    void
)
{
    CANErrorType_t eResult;

    ( void )CAN_RegisterCallbackFunctionTx( &CAN_DemoCallbackTxEvent );
    ( void )CAN_RegisterCallbackFunctionRx( &CAN_DemoCallbackRxEvent );
    ( void )CAN_RegisterCallbackFunctionError( &CAN_DemoCallbackErrorEvent );

    eResult = CAN_Init();
    gCanDemoInitResult = ( eResult == CAN_ERROR_NONE ) ? 0 : -1;

    return gCanDemoInitResult;
}

void CAN_DemoSetPeriodicTxData
(
    const uint8 *                       pucData,
    uint8                               ucLength
)
{
    if( ( pucData != NULL_PTR ) && ( ucLength <= 8U ) )
    {
        if( gCanDemoTxDataLockCreated == TRUE )
        {
            ( void )SAL_SemaphoreWait( gCanDemoTxDataLock, 0, SAL_OPT_BLOCKING );
        }

        ( void )SAL_MemSet( gCanDemoPeriodicTxData, 0, sizeof( gCanDemoPeriodicTxData ) );
        ( void )SAL_MemCopy( gCanDemoPeriodicTxData, pucData, ucLength );

        if( gCanDemoTxDataLockCreated == TRUE )
        {
            ( void )SAL_SemaphoreRelease( gCanDemoTxDataLock );
        }
    }
}

void CAN_DemoTest
(
    uint8                               ucArgc,
    void *                              pArgv[]
)
{
    ( void )ucArgc;
    ( void )pArgv;

    mcu_printf( "CAN CH%d TX:req=%d done=%d err=%d "
                "RX:callback=%d task=%d drop=%d lastErr=%d PSR=0x%08X\n",
                CAN_DEMO_TX_CHANNEL,
                ( unsigned long )gCanDemoTxRequestCount,
                ( unsigned long )gCanDemoTxCompleteCount,
                ( unsigned long )gCanDemoTxErrorCount,
                ( unsigned long )gCanDemoRxCallbackCount[CAN_DEMO_TX_CHANNEL],
                ( unsigned long )gCanDemoRxCount[CAN_DEMO_TX_CHANNEL],
                ( unsigned long )gCanDemoRxDropCount[CAN_DEMO_TX_CHANNEL],
                ( unsigned long )gCanDemoRxLastError[CAN_DEMO_TX_CHANNEL],
                ( unsigned long )CAN_GetProtocolStatus( CAN_DEMO_TX_CHANNEL ) );
}

void CAN_DemoCreateApp
(
    void
)
{
    static uint32 uiCanDemoRxTaskID;
    static uint32 uiCanDemoRxTaskStk[CAN_DEMO_TASK_STK_SIZE];
    static uint32 uiCanDemoTxTaskID;
    static uint32 uiCanDemoTxTaskStk[CAN_DEMO_TX_TASK_STK_SIZE];

    gCanDemoRxEnabled = TRUE;
    gCanDemoTxEnabled = TRUE;
    gCanDemoRxEventCreated = FALSE;
    gCanDemoTxDataLockCreated = FALSE;
    gCanDemoLogLockCreated = FALSE;

    ( void )SAL_MemSet( gCanDemoPeriodicTxData, 0, sizeof( gCanDemoPeriodicTxData ) );

    ( void )GPIO_Set( CAN_0_STB, 0UL );
    ( void )GPIO_Set( CAN_1_STB, 0UL );
    ( void )GPIO_Set( CAN_2_STB, 0UL );

    if( SAL_EventCreate( &gCanDemoRxEvent,
                         ( const uint8 * )"CAN RX Event",
                         0UL ) == SAL_RET_SUCCESS )
    {
        gCanDemoRxEventCreated = TRUE;
    }

    if( SAL_SemaphoreCreate( &gCanDemoTxDataLock,
                             ( const uint8 * )"CAN TX Data",
                             1UL,
                             SAL_OPT_BLOCKING ) == SAL_RET_SUCCESS )
    {
        gCanDemoTxDataLockCreated = TRUE;
    }

    if( SAL_SemaphoreCreate( &gCanDemoLogLock,
                             ( const uint8 * )"CAN Log",
                             1UL,
                             SAL_OPT_BLOCKING ) == SAL_RET_SUCCESS )
    {
        gCanDemoLogLockCreated = TRUE;
    }

    ( void )SAL_TaskCreate( &uiCanDemoRxTaskID,
                            ( const uint8 * )"CAN RX Task",
                            ( SALTaskFunc )&CAN_DemoRxTask,
                            &uiCanDemoRxTaskStk[0],
                            CAN_DEMO_TASK_STK_SIZE,
                            SAL_PRIO_CAN_DEMO,
                            NULL_PTR );

    ( void )SAL_TaskCreate( &uiCanDemoTxTaskID,
                            ( const uint8 * )"CAN Periodic TX Task",
                            ( SALTaskFunc )&CAN_DemoTxTask,
                            &uiCanDemoTxTaskStk[0],
                            CAN_DEMO_TX_TASK_STK_SIZE,
                            SAL_PRIO_CAN_DEMO,
                            NULL_PTR );
}

static void CAN_DemoTxTask
(
    void *                              pArg
)
{
    CANMessage_t sTxMsg;
    CANErrorType_t eResult;
    uint8 ucTxBufferIndex;
    uint8 ucDataIndex;

    ( void )pArg;
    ( void )SAL_MemSet( &sTxMsg, 0, sizeof( sTxMsg ) );

    sTxMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    sTxMsg.mExtendedId = FALSE;
    sTxMsg.mRemoteTransmitRequest = FALSE;
    sTxMsg.mId = CAN_DEMO_TX_ID;
    sTxMsg.mFDFormat = FALSE;
    sTxMsg.mBitRateSwitching = FALSE;
    sTxMsg.mEventFIFOControl = FALSE;
    sTxMsg.mDataLength = 8U;

    while( 1 )
    {
        if( ( gCanDemoTxEnabled == TRUE ) && ( gCanDemoInitResult == 0 ) )
        {
            if( gCanDemoTxDataLockCreated == TRUE )
            {
                ( void )SAL_SemaphoreWait( gCanDemoTxDataLock, 0, SAL_OPT_BLOCKING );
            }

            ( void )SAL_MemCopy( sTxMsg.mData,
                                gCanDemoPeriodicTxData,
                                sizeof( gCanDemoPeriodicTxData ) );

            if( gCanDemoTxDataLockCreated == TRUE )
            {
                ( void )SAL_SemaphoreRelease( gCanDemoTxDataLock );
            }

            eResult = CAN_SendMessage( CAN_DEMO_TX_CHANNEL, &sTxMsg, &ucTxBufferIndex );
            if( eResult == CAN_ERROR_NONE )
            {
                gCanDemoTxRequestCount++;
                gCanDemoLastTxId = sTxMsg.mId;

                if( gCanDemoLogLockCreated == TRUE )
                {
                    ( void )SAL_SemaphoreWait( gCanDemoLogLock, 0, SAL_OPT_BLOCKING );
                }

                mcu_printf( "[CAN TX] CH%d SEQ:%d STD DATA ID:0x%X DLC:%d DATA:",
                            CAN_DEMO_TX_CHANNEL,
                            ( unsigned long )gCanDemoTxRequestCount,
                            ( unsigned long )sTxMsg.mId,
                            sTxMsg.mDataLength );
                for( ucDataIndex = 0U; ucDataIndex < sTxMsg.mDataLength; ucDataIndex++ )
                {
                    mcu_printf( " %02X", sTxMsg.mData[ucDataIndex] );
                }
                mcu_printf( "\n" );

                if( gCanDemoLogLockCreated == TRUE )
                {
                    ( void )SAL_SemaphoreRelease( gCanDemoLogLock );
                }
            }
            else
            {
                gCanDemoTxErrorCount++;
                if( gCanDemoLogLockCreated == TRUE )
                {
                    ( void )SAL_SemaphoreWait( gCanDemoLogLock, 0, SAL_OPT_BLOCKING );
                }
                mcu_printf( "[CAN TX ERROR] CH%d count=%d result=%d PSR=0x%08X\n",
                            CAN_DEMO_TX_CHANNEL,
                            ( unsigned long )gCanDemoTxErrorCount,
                            eResult,
                            ( unsigned long )CAN_GetProtocolStatus( CAN_DEMO_TX_CHANNEL ) );
                if( gCanDemoLogLockCreated == TRUE )
                {
                    ( void )SAL_SemaphoreRelease( gCanDemoLogLock );
                }
            }
        }

        ( void )SAL_TaskSleep( CAN_DEMO_TX_PERIOD_MS );
    }
}

static void CAN_DemoRxTask
(
    void *                              pArg
)
{
    uint8 ucCh;
    uint32 uiEventFlags;

    ( void )pArg;

    mcu_printf( "[VCP CAN] Init=%s RX=ALL-ID TX=CH%d/0x%X/%dms FRAME_LOG=%d\n",
                ( gCanDemoInitResult == 0 ) ? "OK" : "FAIL",
                CAN_DEMO_TX_CHANNEL,
                ( unsigned long )CAN_DEMO_TX_ID,
                ( unsigned long )CAN_DEMO_TX_PERIOD_MS,
                CAN_DEMO_FRAME_LOG_ENABLE );

    while( 1 )
    {
        if( gCanDemoRxEnabled == TRUE )
        {
            for( ucCh = 0U; ucCh < CAN_CONTROLLER_NUMBER; ucCh++ )
            {
                CAN_DemoDrainRx( ucCh );


            }
        }

        if( gCanDemoRxEventCreated == TRUE )
        {
            uiEventFlags = 0UL;
            ( void )SAL_EventGet( gCanDemoRxEvent,
                                  CAN_DEMO_RX_EVENT_MASK,
                                  0UL,
                                  ( uint32 )( SAL_EVENT_OPT_SET_ANY |
                                              SAL_EVENT_OPT_CONSUME |
                                              SAL_OPT_BLOCKING ),
                                  &uiEventFlags );
        }
        else
        {
            ( void )SAL_TaskSleep( 1UL );
        }
    }
}

#endif  // ( MCU_BSP_SUPPORT_CAN_DEMO == 1 )