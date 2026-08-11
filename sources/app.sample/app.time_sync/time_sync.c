#include <app_cfg.h>
#include <sal_api.h>

#include "time_sync.h"
#include <sal_internal.h>
#include <debug.h>
/* =========================================================
 * Configuration
 * ========================================================= */

/*
 * TimeSync Task 실행 주기
 *
 * D3-G Sync Frame도 현재 설계상 100ms 주기이므로
 * 동일하게 100ms 주기로 offset을 보정한다.
 */
#define TIME_SYNC_TASK_PERIOD_MS              (100UL)


/*
 * |시간 오차| <= 50ms
 *      → Slew
 *
 * |시간 오차| > 50ms
 *      → Step
 */
#define TIME_SYNC_SLEW_THRESHOLD_MS           (50LL)


/*
 * 한 번의 TimeSync Task 실행에서
 * offset을 최대 1ms씩 보정한다.
 *
 * 100ms마다 1ms 보정
 * → 최대 1% 수준의 slew
 */
#define TIME_SYNC_MAX_SLEW_PER_PERIOD_MS      (1LL)


/*
 * 우선은 현재 프로젝트에 이미 존재하는
 * SAL_PRIO_CAN_DEMO를 사용한다.
 *
 * 추후 app_priority_cfg.h에
 * SAL_PRIO_TIME_SYNC를 따로 만들면 교체 가능.
 */
#define TIME_SYNC_TASK_PRIORITY               (SAL_PRIO_CAN_DEMO)


/* =========================================================
 * Internal State
 * ========================================================= */

typedef struct
{
    /*
     * 현재 실제 논리시계에 적용 중인 offset
     *
     * logical_time
     * = monotonic_time + currentOffsetMs
     */
    int64_t currentOffsetMs;


    /*
     * Slew 시 최종적으로 맞춰야 하는 offset
     */
    int64_t targetOffsetMs;


    /*
     * 가장 최근 Sync Frame 기준
     * D3-G 시각과 VCP-G 시각의 차이
     */
    int64_t lastDiffMs;


    /*
     * 최초 정상 Sync Frame을 받은 상태인지
     *
     * FALSE : 기준시각 없음
     * TRUE  : 기준시각 있음
     */
    uint8_t timeSynced;

} TimeSyncState_t;


/*
 * time_sync.c 내부에서만 접근 가능한 상태
 */
static TimeSyncState_t gTimeSync;


/* =========================================================
 * Internal Functions
 * ========================================================= */

/*
 * VCP-G 부팅 이후 경과시간을 ms 단위로 획득한다.
 *
 * TOPST SAL API 사용.
 */

 uint8 TimeSync_IsValid
(
    void
)
{
    return TimeSync_IsSynced();
}


uint64 TimeSync_GetCurrentTimeMs
(
    void
)
{
    uint64 ullCurrentTimeMs;

    ullCurrentTimeMs = 0ULL;

    if( TimeSync_GetCurrentMs(
            &ullCurrentTimeMs ) == FALSE )
    {
        return 0ULL;
    }

    return ullCurrentTimeMs;
}

void TimeSync_PrintCurrentTime
(
    void
)
{
    uint64 ullCurrentTimeMs;
    uint64 ullTimeOfDayMs;

    uint32 uiHour;
    uint32 uiMinute;
    uint32 uiSecond;

    if( TimeSync_IsValid() == FALSE )
    {
        mcu_printf( "[TIME SYNC] INVALID\n" );
        return;
    }

    ullCurrentTimeMs =
        TimeSync_GetCurrentTimeMs();

    /*
     * 2026-01-01 00:00:00 UTC 기준이므로
     * 하루(86400000ms)로 나눈 나머지가 현재 시각이다.
     */
    ullTimeOfDayMs =
        ullCurrentTimeMs % 86400000ULL;

    uiHour =
        ( uint32 )
        ( ullTimeOfDayMs / 3600000ULL );

    ullTimeOfDayMs %= 3600000ULL;

    uiMinute =
        ( uint32 )
        ( ullTimeOfDayMs / 60000ULL );

    ullTimeOfDayMs %= 60000ULL;

    uiSecond =
        ( uint32 )
        ( ullTimeOfDayMs / 1000ULL );

    mcu_printf(
        "[TIME SYNC] %02d:%02d:%02d UTC\n",
        uiHour,
        uiMinute,
        uiSecond
    );
}

static uint8_t TimeSync_GetMonotonicMs(
    uint64_t *pullMonotonicMs)
{
    uint32 uiTickMs;


    if (pullMonotonicMs == NULL)
    {
        return FALSE;
    }


    uiTickMs = 0UL;


    if (SAL_GetTickCount(&uiTickMs) != SAL_RET_SUCCESS)
    {
        return FALSE;
    }


    *pullMonotonicMs =
        (uint64_t)uiTickMs;


    return TRUE;
}


/*
 * signed 64bit 절댓값
 */
static uint64_t TimeSync_Abs64(
    int64_t llValue)
{
    if (llValue < 0LL)
    {
        return (uint64_t)(-llValue);
    }


    return (uint64_t)llValue;
}


/*
 * TimeSync 내부 상태 초기화
 */
static void TimeSync_Reset(void)
{
    (void)SAL_CoreCriticalEnter();


    gTimeSync.currentOffsetMs = 0LL;
    gTimeSync.targetOffsetMs  = 0LL;
    gTimeSync.lastDiffMs      = 0LL;
    gTimeSync.timeSynced      = FALSE;


    (void)SAL_CoreCriticalExit();
}


/*
 * currentOffset을 targetOffset 방향으로
 * 조금씩 이동시킨다.
 */
static void TimeSync_Process(void)
{
    int64_t llOffsetDiffMs;


    (void)SAL_CoreCriticalEnter();


    if (gTimeSync.timeSynced == FALSE)
    {
        (void)SAL_CoreCriticalExit();

        return;
    }


    llOffsetDiffMs =
        gTimeSync.targetOffsetMs
        - gTimeSync.currentOffsetMs;


    /*
     * 목표 offset이 현재보다 1ms 이상 크면
     * 이번 주기에는 1ms만 증가
     */
    if (llOffsetDiffMs >
        TIME_SYNC_MAX_SLEW_PER_PERIOD_MS)
    {
        gTimeSync.currentOffsetMs +=
            TIME_SYNC_MAX_SLEW_PER_PERIOD_MS;
    }

    /*
     * 목표 offset이 현재보다 1ms 이상 작으면
     * 이번 주기에는 1ms만 감소
     */
    else if (llOffsetDiffMs <
             -TIME_SYNC_MAX_SLEW_PER_PERIOD_MS)
    {
        gTimeSync.currentOffsetMs -=
            TIME_SYNC_MAX_SLEW_PER_PERIOD_MS;
    }

    /*
     * 남은 오차가 ±1ms 이내면
     * 목표 offset에 바로 맞춘다.
     */
    else
    {
        gTimeSync.currentOffsetMs =
            gTimeSync.targetOffsetMs;
    }


    (void)SAL_CoreCriticalExit();
}


/*
 * TimeSync Task
 *
 * 이 Task가 시간을 직접 증가시키는 것이 아니다.
 *
 * 시간 자체는 SAL_GetTickCount()가 계속 증가시키고,
 * 이 Task는 offset의 Slew 보정만 담당한다.
 */
static void TimeSync_Task(
    void *pArg)
{
    (void)pArg;


    while (1)
    {
        TimeSync_Process();


        (void)SAL_TaskSleep(
            TIME_SYNC_TASK_PERIOD_MS);
    }
}


/* =========================================================
 * Public Functions
 * ========================================================= */

uint8_t TimeSync_OnSyncFrame(
    uint64_t ullSyncEpochMs,
    uint8_t ucSyncStatus)
{
    uint64_t ullMonotonicMs;

    int64_t llDesiredOffsetMs;
    int64_t llCurrentLogicalMs;
    int64_t llDiffMs;


    /*
     * D3-G 자체가 NTP와 동기화되지 않았다면
     * 해당 시간을 기준시각으로 사용하지 않는다.
     */
    if (ucSyncStatus != TIME_SYNC_STATUS_OK)
    {
        (void)SAL_CoreCriticalEnter();

        gTimeSync.timeSynced = FALSE;

        (void)SAL_CoreCriticalExit();


        return FALSE;
    }


    /*
     * Sync Frame을 받은 순간의
     * VCP-G monotonic tick 획득
     */
    if (TimeSync_GetMonotonicMs(
            &ullMonotonicMs) == FALSE)
    {
        return FALSE;
    }


    /*
     * 기준식
     *
     * logical_time
     * = monotonic_time + offset
     *
     * 따라서
     *
     * offset
     * = received_time - monotonic_time
     */
    llDesiredOffsetMs =
        (int64_t)ullSyncEpochMs
        - (int64_t)ullMonotonicMs;


    (void)SAL_CoreCriticalEnter();


    /* =====================================================
     * 최초 동기화
     * ===================================================== */

    if (gTimeSync.timeSynced == FALSE)
    {
        /*
         * 최초에는 VCP-G가 기준시각 자체를 모르므로
         * D3-G가 전달한 시각에 즉시 맞춘다.
         *
         * STEP
         */
        gTimeSync.currentOffsetMs =
            llDesiredOffsetMs;

        gTimeSync.targetOffsetMs =
            llDesiredOffsetMs;

        gTimeSync.lastDiffMs =
            0LL;

        gTimeSync.timeSynced =
            TRUE;


        (void)SAL_CoreCriticalExit();


        return TRUE;
    }


    /* =====================================================
     * 이미 동기화된 상태
     * ===================================================== */

    /*
     * 현재 VCP-G의 자체 논리시각
     */
    llCurrentLogicalMs =
        (int64_t)ullMonotonicMs
        + gTimeSync.currentOffsetMs;


    /*
     * D3-G 기준시각과의 오차
     *
     * +5ms
     * → D3가 VCP보다 5ms 앞섬
     * → VCP가 5ms 느림
     *
     * -5ms
     * → VCP가 5ms 빠름
     */
    llDiffMs =
        (int64_t)ullSyncEpochMs
        - llCurrentLogicalMs;


    gTimeSync.lastDiffMs =
        llDiffMs;


    /* =====================================================
     * STEP / SLEW
     * ===================================================== */

    if (TimeSync_Abs64(llDiffMs) >
        (uint64_t)TIME_SYNC_SLEW_THRESHOLD_MS)
    {
        /*
         * 오차가 50ms보다 큼
         *
         * 정상적인 작은 drift로 보기 어려우므로
         * 즉시 기준값으로 보정한다.
         *
         * STEP
         */
        gTimeSync.currentOffsetMs =
            llDesiredOffsetMs;

        gTimeSync.targetOffsetMs =
            llDesiredOffsetMs;
    }
    else
    {
        /*
         * 오차가 50ms 이하
         *
         * currentOffset을 즉시 변경하지 않고
         * targetOffset만 변경한다.
         *
         * 이후 TimeSync_Task에서
         * 조금씩 따라간다.
         *
         * SLEW
         */
        gTimeSync.targetOffsetMs =
            llDesiredOffsetMs;
    }


    (void)SAL_CoreCriticalExit();


    return TRUE;
}


uint8_t TimeSync_IsSynced(void)
{
    uint8_t ucSynced;


    (void)SAL_CoreCriticalEnter();

    ucSynced =
        gTimeSync.timeSynced;

    (void)SAL_CoreCriticalExit();


    return ucSynced;
}


uint8_t TimeSync_GetCurrentMs(
    uint64_t *pullCurrentMs)
{
    uint64_t ullMonotonicMs;

    int64_t llOffsetMs;
    int64_t llLogicalTimeMs;

    uint8_t ucSynced;


    if (pullCurrentMs == NULL)
    {
        return FALSE;
    }


    /*
     * 현재 SAL monotonic tick 획득
     */
    if (TimeSync_GetMonotonicMs(
            &ullMonotonicMs) == FALSE)
    {
        return FALSE;
    }


    /*
     * 공유 상태는 짧게 Snapshot만 뜬다.
     */
    (void)SAL_CoreCriticalEnter();


    ucSynced =
        gTimeSync.timeSynced;

    llOffsetMs =
        gTimeSync.currentOffsetMs;


    (void)SAL_CoreCriticalExit();


    if (ucSynced == FALSE)
    {
        return FALSE;
    }


    /*
     * VCP-G 자체 기준시계
     *
     * logical_time
     * = RTOS monotonic + synchronized offset
     */
    llLogicalTimeMs =
        (int64_t)ullMonotonicMs
        + llOffsetMs;


    if (llLogicalTimeMs < 0LL)
    {
        return FALSE;
    }


    *pullCurrentMs =
        (uint64_t)llLogicalTimeMs;


    return TRUE;
}


uint8_t TimeSync_GetTimestamp12(
    uint16_t *pusTimestamp12)
{
    uint64_t ullCurrentMs;


    if (pusTimestamp12 == NULL)
    {
        return FALSE;
    }


    if (TimeSync_GetCurrentMs(
            &ullCurrentMs) == FALSE)
    {
        return FALSE;
    }


    /*
     * 전체 논리시각의 하위 12bit만 사용
     *
     * 0 ~ 4095ms 반복
     */
    *pusTimestamp12 =
        (uint16_t)(
            ullCurrentMs & 0x0FFFULL);


    return TRUE;
}


int64_t TimeSync_GetLastDiffMs(void)
{
    int64_t llDiffMs;


    (void)SAL_CoreCriticalEnter();

    llDiffMs =
        gTimeSync.lastDiffMs;

    (void)SAL_CoreCriticalExit();


    return llDiffMs;
}


void TimeSync_AppCreate(void)
{
    static uint32 uiTimeSyncTaskID = 0UL;

    static uint32 uiTimeSyncTaskStk[
        ACFG_TASK_MEDIUM_STK_SIZE];


    /*
     * Task 생성 전 상태 초기화
     */
    TimeSync_Reset();


    /*
     * 기존 VCP-G App과 동일하게
     * SAL_TaskCreate() 사용
     */
    (void)SAL_TaskCreate(
        &uiTimeSyncTaskID,
        (const uint8 *)"Time Sync Task",
        (SALTaskFunc)&TimeSync_Task,
        &uiTimeSyncTaskStk[0],
        ACFG_TASK_MEDIUM_STK_SIZE,
        TIME_SYNC_TASK_PRIORITY,
        NULL_PTR);
}