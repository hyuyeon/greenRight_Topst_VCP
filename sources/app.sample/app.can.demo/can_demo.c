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
#include "common.h"
#include "turnJudgeTask.h"

#define CAN_DEMO_FRAME_SIZE                 (8U)
#define CAN_DEMO_MSG_ID_SHIFT               (60U)
#define CAN_DEMO_TIMESTAMP_SHIFT            (48U)
#define CAN_DEMO_TIMESTAMP_MASK             (0x0FFFU)

#define CAN_DEMO_MSG_EGO_STATUS             (0x0U)
#define CAN_DEMO_MSG_CANDIDATE_INTRO        (0x4U)
#define CAN_DEMO_MSG_CANDIDATE_STATUS       (0x5U)
#define CAN_DEMO_MSG_TRAFFIC_LIGHT          (0x6U)




static volatile uint8 gCanDemoRxEnabled;
static volatile uint8 gCanDemoTxEnabled;
static sint32 gCanDemoInitResult = -1;

static uint32 gCanDemoRxEvent;
static uint8 gCanDemoRxEventCreated;
static uint32 gCanDemoTlDisplaySem;
static uint8 gCanDemoTlDisplaySemCreated;
static uint32 gCanDemoLogLock;
static uint8 gCanDemoLogLockCreated;

static volatile uint32 gCanDemoRxCount[CAN_CONTROLLER_NUMBER];
static volatile uint32 gCanDemoRxCallbackCount[CAN_CONTROLLER_NUMBER];
static volatile uint32 gCanDemoRxDropCount[CAN_CONTROLLER_NUMBER];
static volatile uint32 gCanDemoRxLastError[CAN_CONTROLLER_NUMBER];
static volatile uint32 gCanDemoRxInvalidCount;
static volatile uint32 gCanDemoRxUnknownCount;
static volatile uint32 gCanDemoTxRequestCount;
static volatile uint32 gCanDemoTxCompleteCount;
static volatile uint32 gCanDemoTxErrorCount;
static volatile uint32 gCanDemoLastRxId;
static volatile uint32 gCanDemoLastTxId;

static CANMessage_t gCanDemoRxBatch[CAN_CONTROLLER_NUMBER][CAN_DEMO_RX_BATCH_MAX];

static void CAN_DemoRxTask(void *pArg);
static void CAN_DemoTxTask(void *pArg);

static uint64_t CAN_DemoLoadFrame
(
    const uint8 *                       pucData
)
{
    uint8 ucIndex;
    uint64_t ullFrame;

    ullFrame = 0ULL;

    for( ucIndex = 0U; ucIndex < CAN_DEMO_FRAME_SIZE; ucIndex++ )
    {
        ullFrame = ( ullFrame << 8U ) | ( uint64_t )pucData[ucIndex];
    }

    return ullFrame;
}

static void CAN_DemoStoreFrame
(
    uint64_t                            ullFrame,
    uint8 *                             pucData
)
{
    uint8 ucIndex;

    for( ucIndex = 0U; ucIndex < CAN_DEMO_FRAME_SIZE; ucIndex++ )
    {
        pucData[CAN_DEMO_FRAME_SIZE - 1U - ucIndex] =
            ( uint8 )( ullFrame & 0xFFULL );
        ullFrame >>= 8U;
    }
}

static void CAN_DemoPackEgoStatus
(
    const EgoVehicle *                  psEgo,
    uint8 *                             pucData
)
{
    uint64_t ullFrame;

    ullFrame =
        ( ( uint64_t )CAN_DEMO_MSG_EGO_STATUS << CAN_DEMO_MSG_ID_SHIFT ) |
        ( ( uint64_t )( psEgo->timestamp & CAN_DEMO_TIMESTAMP_MASK )
            << CAN_DEMO_TIMESTAMP_SHIFT ) |
        ( ( uint64_t )psEgo->speed << 32U ) |
        ( ( uint64_t )( psEgo->x & 0x03FFU ) << 22U ) |
        ( ( uint64_t )( psEgo->y & 0x07FFU ) << 11U ) |
        ( ( uint64_t )( psEgo->heading & 0x01FFU ) << 2U ) |
        ( uint64_t )( psEgo->turn_signal & 0x03U );

    CAN_DemoStoreFrame( ullFrame, pucData );
}

static uint8 CAN_DemoIsCandidateTypeValid
(
    uint8                               ucType
)
{
    uint8 ucValid;

    ucValid = FALSE;

    if( ( ucType == CAND_NONE ) ||
        ( ucType == CAND_COMM_ERROR ) ||
        ( ucType == CAND_RT_LEFT_STRAIGHT ) ||
        ( ucType == CAND_RT_OPP_LEFT ) ||
        ( ucType == CAND_LT_OPP_STRAIGHT ) ||
        ( ucType == CAND_LT_OPP_RIGHT ) )
    {
        ucValid = TRUE;
    }

    return ucValid;
}

static uint8 CAN_DemoIsTrafficLightTypeValid
(
    uint8                               ucType
)
{
    uint8 ucValid;

    ucValid = FALSE;

    if( ( ucType == TL_NONE ) ||
        ( ucType == TL_COMM_ERROR ) ||
        ( ( ucType >= 1U ) && ( ucType <= 4U ) ) )
    {
        ucValid = TRUE;
    }

    return ucValid;
}

static void CAN_DemoNotifyTlDisplay
(
    void
)
{
    if( gCanDemoTlDisplaySemCreated == TRUE )
    {
        ( void )SAL_SemaphoreRelease( gCanDemoTlDisplaySem );
    }
}

static uint8 CAN_DemoHandleCandidateIntro
(
    uint64_t                            ullFrame
)
{
    uint8 ucType;
    uint16 usConflictZoneX;
    uint16 usConflictZoneY;

    ucType = ( uint8 )( ( ullFrame >> 32U ) & 0xFFULL );
    usConflictZoneX = ( uint16 )( ( ullFrame >> 22U ) & 0x03FFULL );
    usConflictZoneY = ( uint16 )( ( ullFrame >> 11U ) & 0x07FFULL );

    if( CAN_DemoIsCandidateTypeValid( ucType ) == FALSE )
    {
        return FALSE;
    }

    ( void )SAL_CoreCriticalEnter();

    candidateVehicle.type = ucType;
    candidateVehicle.cz_x = usConflictZoneX;
    candidateVehicle.cz_y = usConflictZoneY;

    ( void )SAL_CoreCriticalExit();

    return TRUE;
}

static uint8 CAN_DemoHandleCandidateStatus
(
    uint64_t                            ullFrame
)
{
    CandidateVehicle sNewCandidate;
    uint8 ucType;
    uint32 uiReceivedTick;

    ucType = ( uint8 )( ( ullFrame >> 40U ) & 0xFFULL );

    if( CAN_DemoIsCandidateTypeValid( ucType ) == FALSE )
    {
        return FALSE;
    }

    uiReceivedTick = 0UL;
    ( void )SAL_GetTickCount( &uiReceivedTick );
    ( void )SAL_MemSet( &sNewCandidate, 0, sizeof( sNewCandidate ) );

    sNewCandidate.type = ucType;
    sNewCandidate.timestamp_ms =
        ( uint64_t )( ( ullFrame >> CAN_DEMO_TIMESTAMP_SHIFT ) &
                      CAN_DEMO_TIMESTAMP_MASK );
    sNewCandidate.received_timestamp = ( uint64_t )uiReceivedTick;

    if( ( ucType != CAND_NONE ) && ( ucType != CAND_COMM_ERROR ) )
    {
        ( void )SAL_CoreCriticalEnter();
        sNewCandidate.cz_x = candidateVehicle.cz_x;
        sNewCandidate.cz_y = candidateVehicle.cz_y;
        ( void )SAL_CoreCriticalExit();

        sNewCandidate.speed = ( uint8 )( ( ullFrame >> 32U ) & 0xFFULL );
        sNewCandidate.x = ( uint16 )( ( ullFrame >> 22U ) & 0x03FFULL );
        sNewCandidate.y = ( uint16 )( ( ullFrame >> 11U ) & 0x07FFULL );
        sNewCandidate.heading = ( uint16 )( ( ullFrame >> 2U ) & 0x01FFULL );
    }

    ( void )SAL_CoreCriticalEnter();
    candidateVehicle = sNewCandidate;
    ( void )SAL_CoreCriticalExit();

    TurnJudge_Notify();

    return TRUE;
}

static uint8 CAN_DemoHandleTrafficLight
(
    uint64_t                            ullFrame
)
{
    TrafficLight sNewTrafficLight;
    uint8 ucNewManeuver;
    uint8 ucPreviousManeuver;
    uint8 ucDisplayChanged;
    uint32 uiReceivedTick;

    ( void )SAL_MemSet( &sNewTrafficLight, 0, sizeof( sNewTrafficLight ) );

    sNewTrafficLight.type = ( uint8 )( ( ullFrame >> 32U ) & 0xFFULL );

    if( CAN_DemoIsTrafficLightTypeValid( sNewTrafficLight.type ) == FALSE )
    {
        return FALSE;
    }

    sNewTrafficLight.color = ( uint8 )( ( ullFrame >> 30U ) & 0x03ULL );
    sNewTrafficLight.time_left = ( uint8 )( ( ullFrame >> 26U ) & 0x0FULL );
    sNewTrafficLight.cz_x = ( uint16 )( ( ullFrame >> 16U ) & 0x03FFULL );
    sNewTrafficLight.cz_y = ( uint16 )( ( ullFrame >> 5U ) & 0x07FFULL );
    sNewTrafficLight.timestamp = ( uint16 )
        ( ( ullFrame >> CAN_DEMO_TIMESTAMP_SHIFT ) & CAN_DEMO_TIMESTAMP_MASK );

    uiReceivedTick = 0UL;
    ( void )SAL_GetTickCount( &uiReceivedTick );
    sNewTrafficLight.received_timestamp = ( uint64_t )uiReceivedTick;
    ucNewManeuver = ( uint8 )( ( ullFrame >> 3U ) & 0x03ULL );

    ( void )SAL_CoreCriticalEnter();

    ucPreviousManeuver = maneuver;
    ucDisplayChanged =
        ( ( tl.type != sNewTrafficLight.type ) ||
          ( tl.color != sNewTrafficLight.color ) ||
          ( tl.time_left != sNewTrafficLight.time_left ) ) ? TRUE : FALSE;

    tl = sNewTrafficLight;
    maneuver = ucNewManeuver;

    ( void )SAL_CoreCriticalExit();

    if( ucDisplayChanged == TRUE )
    {
        CAN_DemoNotifyTlDisplay();
    }

    if( ( ( ucPreviousManeuver == MANEUVER_RIGHT_TURN ) ||
          ( ucPreviousManeuver == MANEUVER_LEFT_TURN_UNPROT ) ) &&
        ( ucNewManeuver == MANEUVER_STRAIGHT ) )
    {
        TurnJudge_Notify();
    }

    return TRUE;
}

static void CAN_DemoHandleRxMessage
(
    const CANMessage_t *                psRxMsg
)
{
    uint8 ucMessageId;
    uint8 ucHandled;
    uint64_t ullFrame;

    if( ( psRxMsg->mExtendedId != FALSE ) ||
        ( psRxMsg->mRemoteTransmitRequest != FALSE ) ||
        ( psRxMsg->mFDFormat != FALSE ) ||
        ( psRxMsg->mDataLength != CAN_DEMO_FRAME_SIZE ) )
    {
        gCanDemoRxInvalidCount++;
        return;
    }

    ullFrame = CAN_DemoLoadFrame( psRxMsg->mData );
    ucMessageId = ( uint8 )( ( ullFrame >> CAN_DEMO_MSG_ID_SHIFT ) & 0x0FULL );
    ucHandled = FALSE;

    switch( ucMessageId )
    {
        case CAN_DEMO_MSG_CANDIDATE_INTRO:
        {
            ucHandled = CAN_DemoHandleCandidateIntro( ullFrame );
            break;
        }

        case CAN_DEMO_MSG_CANDIDATE_STATUS:
        {
            ucHandled = CAN_DemoHandleCandidateStatus( ullFrame );
            break;
        }

        case CAN_DEMO_MSG_TRAFFIC_LIGHT:
        {
            ucHandled = CAN_DemoHandleTrafficLight( ullFrame );
            break;
        }

        default:
        {
            gCanDemoRxUnknownCount++;
            return;
        }
    }

    if( ucHandled == FALSE )
    {
        gCanDemoRxInvalidCount++;
    }
}

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
    uint16 usTimestamp;
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

            CAN_DemoHandleRxMessage( psRxMsg );

#if ( CAN_DEMO_FRAME_LOG_ENABLE == 1U )
            usTimestamp = 0U;
            if( psRxMsg->mDataLength >= 2U )
            {
                usTimestamp =
                    ( uint16 )( ( ( uint16 )( psRxMsg->mData[0] & 0x0FU ) << 8U ) |
                               ( uint16 )psRxMsg->mData[1] );
            }

            if( gCanDemoLogLockCreated == TRUE )
            {
                ( void )SAL_SemaphoreWait( gCanDemoLogLock, 0, SAL_OPT_BLOCKING );
            }

            mcu_printf( "[CAN RX] CH%d SEQ:%d ID:0x%X TS:%d DLC:%d DATA:",
                        ucCh,
                        ( unsigned long )gCanDemoRxCount[ucCh],
                        ( unsigned long )psRxMsg->mId,
                        usTimestamp,
                        psRxMsg->mDataLength );

            for( ucDataIndex = 0U; ucDataIndex < psRxMsg->mDataLength; ucDataIndex++ )
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

sint32 CAN_DemoWaitTrafficLightDisplay
(
    uint32                              uiTimeout
)
{
    sint32 iResult;

    iResult = -1;

    if( gCanDemoTlDisplaySemCreated == TRUE )
    {
        iResult = ( sint32 )SAL_SemaphoreWait( gCanDemoTlDisplaySem,
                                               uiTimeout,
                                               SAL_OPT_BLOCKING );
    }

    return iResult;
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
                "RX:callback=%d task=%d drop=%d invalid=%d unknown=%d "
                "lastErr=%d PSR=0x%08X\n",
                CAN_DEMO_TX_CHANNEL,
                ( unsigned long )gCanDemoTxRequestCount,
                ( unsigned long )gCanDemoTxCompleteCount,
                ( unsigned long )gCanDemoTxErrorCount,
                ( unsigned long )gCanDemoRxCallbackCount[CAN_DEMO_TX_CHANNEL],
                ( unsigned long )gCanDemoRxCount[CAN_DEMO_TX_CHANNEL],
                ( unsigned long )gCanDemoRxDropCount[CAN_DEMO_TX_CHANNEL],
                ( unsigned long )gCanDemoRxInvalidCount,
                ( unsigned long )gCanDemoRxUnknownCount,
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
    gCanDemoTlDisplaySemCreated = FALSE;
    gCanDemoLogLockCreated = FALSE;

    ( void )SAL_CoreCriticalEnter();
    ( void )SAL_MemSet( &candidateVehicle, 0, sizeof( candidateVehicle ) );
    ( void )SAL_MemSet( &tl, 0, sizeof( tl ) );
    maneuver = MANEUVER_STRAIGHT;
    ( void )SAL_CoreCriticalExit();

    ( void )GPIO_Set( CAN_0_STB, 0UL );
    ( void )GPIO_Set( CAN_1_STB, 0UL );
    ( void )GPIO_Set( CAN_2_STB, 0UL );

    if( SAL_EventCreate( &gCanDemoRxEvent,
                         ( const uint8 * )"CAN RX Event",
                         0UL ) == SAL_RET_SUCCESS )
    {
        gCanDemoRxEventCreated = TRUE;
    }

    if( SAL_SemaphoreCreate( &gCanDemoTlDisplaySem,
                             ( const uint8 * )"CAN TL Display",
                             1UL,
                             SAL_OPT_BLOCKING ) == SAL_RET_SUCCESS )
    {
        gCanDemoTlDisplaySemCreated = TRUE;
        ( void )SAL_SemaphoreWait( gCanDemoTlDisplaySem,
                                   0UL,
                                   SAL_OPT_NON_BLOCKING );
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
                            ( const uint8 * )"CAN TX Task",
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
    EgoVehicle sEgoSnapshot;
    CANMessage_t sTxMsg;
    CANErrorType_t eResult;
    uint8 ucTxBufferIndex;
#if ( CAN_DEMO_FRAME_LOG_ENABLE == 1U )
    uint8 ucDataIndex;
#endif

    ( void )pArg;
    ( void )SAL_MemSet( &sTxMsg, 0, sizeof( sTxMsg ) );

    sTxMsg.mBufferType = CAN_TX_BUFFER_TYPE_FIFO;
    sTxMsg.mExtendedId = FALSE;
    sTxMsg.mRemoteTransmitRequest = FALSE;
    sTxMsg.mId = CAN_DEMO_TX_ID;
    sTxMsg.mFDFormat = FALSE;
    sTxMsg.mBitRateSwitching = FALSE;
    sTxMsg.mEventFIFOControl = FALSE;
    sTxMsg.mDataLength = CAN_DEMO_FRAME_SIZE;

    while( 1 )
    {
        if( ( gCanDemoTxEnabled == TRUE ) && ( gCanDemoInitResult == 0 ) )
        {
            ( void )SAL_CoreCriticalEnter();
            sEgoSnapshot = ego;
            ( void )SAL_CoreCriticalExit();

            CAN_DemoPackEgoStatus( &sEgoSnapshot, sTxMsg.mData );

            eResult = CAN_SendMessage( CAN_DEMO_TX_CHANNEL,
                                       &sTxMsg,
                                       &ucTxBufferIndex );
            if( eResult == CAN_ERROR_NONE )
            {
                gCanDemoTxRequestCount++;
                gCanDemoLastTxId = sTxMsg.mId;

#if ( CAN_DEMO_FRAME_LOG_ENABLE == 1U )
                if( gCanDemoLogLockCreated == TRUE )
                {
                    ( void )SAL_SemaphoreWait( gCanDemoLogLock,
                                               0UL,
                                               SAL_OPT_BLOCKING );
                }

                mcu_printf( "[CAN TX] CH%d SEQ:%d ID:0x%X MSG:%04d TS:%d DATA:",
                            CAN_DEMO_TX_CHANNEL,
                            ( unsigned long )gCanDemoTxRequestCount,
                            ( unsigned long )sTxMsg.mId,
                            CAN_DEMO_MSG_EGO_STATUS,
                            ( uint16 )( sEgoSnapshot.timestamp &
                                       CAN_DEMO_TIMESTAMP_MASK ) );
                for( ucDataIndex = 0U;
                     ucDataIndex < sTxMsg.mDataLength;
                     ucDataIndex++ )
                {
                    mcu_printf( " %02X", sTxMsg.mData[ucDataIndex] );
                }
                mcu_printf( "\n" );

                if( gCanDemoLogLockCreated == TRUE )
                {
                    ( void )SAL_SemaphoreRelease( gCanDemoLogLock );
                }
#endif
            }
            else
            {
                gCanDemoTxErrorCount++;
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

    mcu_printf( "[VCP CAN] Init=%s RX=MSG[4,5,6] TX=CH%d/ID0x%X/MSG0000/%dms\n",
                ( gCanDemoInitResult == 0 ) ? "OK" : "FAIL",
                CAN_DEMO_TX_CHANNEL,
                ( unsigned long )CAN_DEMO_TX_ID,
                ( unsigned long )CAN_DEMO_TX_PERIOD_MS );

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
