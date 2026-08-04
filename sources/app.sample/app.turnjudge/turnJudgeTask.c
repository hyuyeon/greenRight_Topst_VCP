#include "turnJudgeTask.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include <sal_api.h>
#include <app_cfg.h>
#include <debug.h>

#include "common.h"
#include "display_task.h"
#include "ttc.h"

#if (MCU_BSP_SUPPORT_APP_BUZZER == 1)
#include "buzzerTask.h"
#endif

/* -------------------------------------------------------------------------- */
/* Configuration                                                              */
/* -------------------------------------------------------------------------- */

/*
 * 실제 차량 기준 5.5초를 프로젝트 축척에 맞게 변환한 값.
 * 1차 프로젝트에서 사용하던 값 유지.
 */
#define RIGHT_TURN_JUDGE_CRITICAL_GAP_SEC       (5.29)
#define LEFT_TURN_JUDGE_CRITICAL_GAP_SEC       (5.21)
#define TURN_JUDGE_YELLOW_DURATION_SEC   (1.0)

/*
 * 기존 CMSIS 태스크의 stack_size는 512 * 4 bytes였다.
 * VCP-G에서는 uint32 스택 배열을 사용한다.
 */
#define TURN_JUDGE_TASK_STACK_SIZE        ACFG_TASK_MEDIUM_STK_SIZE

#define TURN_JUDGE_LOG_ENABLE             (1U)


/* -------------------------------------------------------------------------- */
/* Task resources                                                             */
/* -------------------------------------------------------------------------- */

static uint32 gTurnJudgeTaskId;
static uint32 gTurnJudgeTaskStack[TURN_JUDGE_TASK_STACK_SIZE];
static uint8_t gTurnJudgeCreated;
static uint32 gTurnJudgeSem;
static uint8_t gTurnJudgeSemCreated;

/* -------------------------------------------------------------------------- */
/* Internal function declarations                                             */
/* -------------------------------------------------------------------------- */

static void TurnJudgeTask(void *pArg);
static void TurnJudge_Wait(void);

static void TurnJudge_GetSnapshots(
    EgoVehicle *egoSnap,
    CandidateVehicle *candidateSnap,
    TrafficLight *trafficLightSnap,
    uint8_t *maneuverSnap,
    uint8_t *pedestrianSnap
);

static uint8_t TurnJudge_IsSameDecision(
    const Dicision *left,
    const Dicision *right
);

static uint8_t TurnJudge_GetCandidateTurnLeft(
    uint8_t candidateType
);

static uint8_t JudgeRightTurnLeftStraight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
);

static uint8_t JudgeRightTurnOppLeft(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
);

static void BuildRightTurnDecision(
    Dicision *decision,
    uint8_t pedestrianFlag,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
);

static uint8_t JudgeLeftTurnTlTime(
    const EgoVehicle *egoSnap,
    const TrafficLight *trafficLightSnap
);

static uint8_t JudgeLeftTurnCandidate(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap
);

static void BuildLeftTurnDecision(
    Dicision *decision,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
);

/* -------------------------------------------------------------------------- */
/* Snapshot                                                                   */
/* -------------------------------------------------------------------------- */

static void TurnJudge_GetSnapshots(
    EgoVehicle *egoSnap,
    CandidateVehicle *candidateSnap,
    TrafficLight *trafficLightSnap,
    uint8_t *maneuverSnap,
    uint8_t *pedestrianSnap
)
{
    if ((egoSnap == NULL) ||
        (candidateSnap == NULL) ||
        (trafficLightSnap == NULL) ||
        (maneuverSnap == NULL) ||
        (pedestrianSnap == NULL))
    {
        return;
    }

    /* 판단에 사용되는 공유 상태를 동일한 시점의 값으로 복사한다. */
    (void)SAL_CoreCriticalEnter();

    *egoSnap = ego;
    *candidateSnap = candidateVehicle;
    *trafficLightSnap = tl;
    *maneuverSnap = maneuver;
    *pedestrianSnap = pedFlag;

    (void)SAL_CoreCriticalExit();
}


/* -------------------------------------------------------------------------- */
/* Candidate maneuver helper                                                  */
/* -------------------------------------------------------------------------- */

static uint8_t TurnJudge_GetCandidateTurnLeft(
    uint8_t candidateType
)
{
    /*
     * 후보 차량이 좌회전 중인 경우만 turn_left = 1로 취급한다.
     * (CAND_RT_OPP_LEFT: 자차 우회전 vs 대향 보호좌회전 후보)
     * 그 외(직진, 우회전 후보)는 turn_left = 0으로 두면
     * calculate_TTC 내부에서 heading이 충돌구역을 향할 때
     * 자연스럽게 직선거리로 계산된다.
     */
    if ((candidateType & CAND_RT_OPP_LEFT) != 0U)
    {
        return 1U;
    }

    return 0U;
}


/* -------------------------------------------------------------------------- */
/* Right-turn judgement                                                       */
/* -------------------------------------------------------------------------- */

//The reason "Why Judge~~ function return type is int" is very simple.
//This Function returns 0 when the ego vehicle can go, and returns 1 when the ego vehicle should stop.
//simply, 0 means zero error, and 1 means there is a warning. So, we can use int type to return the result.
static uint8_t JudgeRightTurnLeftStraightBackup(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
)
{
    //후보 차량 있을 때만 타는 함수, 없으면 걍 경고 안뜸

    double egoTtc;
    double passableTimeSec;

    /*
     * 충돌구역 좌표가 없으면 이미 해당 판단 구역을 벗어난 것이므로
     * 경고를 띄우지 않음
     */
    if ((trafficLightSnap->cz_x == 0U) &&
        (trafficLightSnap->cz_y == 0U))
    {
        return 0U;
    }

    /*
     * 좌측 직진 후보 차량이 움직이고 있으면 진입 금지.
     */
    if (candidateSnap->speed > 0U)
    {
        return 1U;
    }

    /*
     * 신호등 데이터가 없으면 신호 시간 판단을 건너뛴다.
     *
     */
    if (trafficLightSnap->type == TL_NONE)
    {
        return 0U;
    }

    egoTtc = calculate_TTC(
        egoSnap->x,
        egoSnap->y,
        egoSnap->heading,
        egoSnap->speed,
        trafficLightSnap->cz_x,
        trafficLightSnap->cz_y,
        0U
    );

    passableTimeSec = (double)trafficLightSnap->time_left;

    if (trafficLightSnap->color == SIG_GREEN)
    {
        passableTimeSec += TURN_JUDGE_YELLOW_DURATION_SEC;
    }

    /*
     * 녹색 또는 황색 신호이며,
     * 자차가 신호 종료 전에 충돌구역을 통과할 수 있으면 안전.
     */
    if (((trafficLightSnap->color == SIG_GREEN) ||
         (trafficLightSnap->color == SIG_YELLOW)) &&
        (passableTimeSec > egoTtc))
    {
        return 0U;
    }

    return 1U;
}

static uint8_t JudgeRightTurnLeftStraight(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
)
{
    double egoTtc;
    double candidateTtc;
    double ttcGap;
    double passableTimeSec;

    /*
     * 이 함수는 CAND_RT_LEFT_STRAIGHT candidate가 있을 때만 호출된다.
     * 자차에게 남은 conflict zone이 없으면 이미 판단 구역을
     * 통과한 것으로 보고 경고하지 않는다.
     */
    if ((trafficLightSnap->cz_x == 0U) &&
        (trafficLightSnap->cz_y == 0U))
    {
        return 0U;
    }

    /*
     * 정지 중인 candidate는 자차 신호가 녹색 또는 황색일 때
     * 신호가 끝나기 전에 자차가 통과할 수 있는지 먼저 판단한다.
     */
    if (candidateSnap->speed == 0U)
    {
        if ((trafficLightSnap->color == SIG_GREEN) ||
            (trafficLightSnap->color == SIG_YELLOW))
        {
            egoTtc = calculate_TTC(
                egoSnap->x,
                egoSnap->y,
                egoSnap->heading,
                egoSnap->speed,
                trafficLightSnap->cz_x,
                trafficLightSnap->cz_y,
                0U
            );

            passableTimeSec =
                (double)trafficLightSnap->time_left;

            if (trafficLightSnap->color == SIG_GREEN)
            {
                passableTimeSec +=
                    TURN_JUDGE_YELLOW_DURATION_SEC;
            }

            if (passableTimeSec > egoTtc)
            {
                return 0U;
            }
        }
    }

    /*
     * 신호 시간만으로 안전하다고 판단하지 못했거나 candidate가
     * 주행 중이면 공유 conflict zone 기준으로 두 차량의 TTC를
     * 계산한다. 정지 candidate에는 calculate_TTC 내부의 최소
     * 크리핑 속도가 적용된다.
     */
    egoTtc = calculate_TTC(
        egoSnap->x,
        egoSnap->y,
        egoSnap->heading,
        egoSnap->speed,
        candidateSnap->cz_x,
        candidateSnap->cz_y,
        0U
    );

    candidateTtc = calculate_TTC(
        candidateSnap->x,
        candidateSnap->y,
        candidateSnap->heading,
        candidateSnap->speed,
        candidateSnap->cz_x,
        candidateSnap->cz_y,
        0U
    );

    ttcGap = fabs(candidateTtc - egoTtc);

    if (ttcGap > RIGHT_TURN_JUDGE_CRITICAL_GAP_SEC)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t JudgeRightTurnOppLeft(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
)
{
    double egoTtc;
    double candidateTtc;
    double ttcGap;
    double passableTimeSec;

    /*
     * 이 함수는 CAND_RT_OPP_LEFT candidate가 있을 때만 호출된다.
     * 자차에게 남은 conflict zone이 없으면 이미 판단 구역을
     * 통과한 것으로 보고 경고하지 않는다.
     */
    if ((trafficLightSnap->cz_x == 0U) &&
        (trafficLightSnap->cz_y == 0U))
    {
        return 0U;
    }

    /*
     * 현재 정책에서는 자차 신호가 빨간색이면 대향 좌회전
     * candidate의 신호도 빨간색이라고 가정한다. 정지 중인
     * candidate에 대해서는 빨간불이 끝나기 전에 자차가
     * 통과할 수 있는지 먼저 판단한다.
     */
    if ((candidateSnap->speed == 0U) &&
        (trafficLightSnap->color == SIG_RED))
    {
        egoTtc = calculate_TTC(
            egoSnap->x,
            egoSnap->y,
            egoSnap->heading,
            egoSnap->speed,
            trafficLightSnap->cz_x,
            trafficLightSnap->cz_y,
            0U
        );

        passableTimeSec =
            (double)trafficLightSnap->time_left;

        if (passableTimeSec > egoTtc)
        {
            return 0U;
        }
    }

    /*
     * 빨간불 시간만으로 안전하다고 판단하지 못했거나 candidate가
     * 주행 중이면 공유 conflict zone 기준으로 두 차량의 TTC를
     * 계산한다. candidate는 좌회전 경로이므로 turn_left = 1이다.
     */
    egoTtc = calculate_TTC(
        egoSnap->x,
        egoSnap->y,
        egoSnap->heading,
        egoSnap->speed,
        candidateSnap->cz_x,
        candidateSnap->cz_y,
        0U
    );

    candidateTtc = calculate_TTC(
        candidateSnap->x,
        candidateSnap->y,
        candidateSnap->heading,
        candidateSnap->speed,
        candidateSnap->cz_x,
        candidateSnap->cz_y,
        1U
    );

    ttcGap = fabs(candidateTtc - egoTtc);

    if (ttcGap > RIGHT_TURN_JUDGE_CRITICAL_GAP_SEC)
    {
        return 0U;
    }

    return 1U;
}

static void BuildRightTurnDecision(
    Dicision *decision,
    uint8_t pedestrianFlag,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
)
{
    if (decision == NULL)
    {
        return;
    }

    (void)memset(decision, 0, sizeof(Dicision));

    decision->turnState = MANEUVER_RIGHT_TURN;

    if (pedestrianFlag != 0U)
    {
        decision->pedestrianFlag = pedestrianFlag;
    }

    if ((candidateSnap->type & CAND_RT_LEFT_STRAIGHT) != 0U)
    {
        if (JudgeRightTurnLeftStraight(
                egoSnap,
                candidateSnap,
                trafficLightSnap) != 0U)
        {
            decision->LStraightFlag = 1U;
        }
    }

    if ((candidateSnap->type & CAND_RT_OPP_LEFT) != 0U)
    {
        if (JudgeRightTurnOppLeft(
                egoSnap,
                candidateSnap,
                trafficLightSnap) != 0U)
        {
            decision->OppLeftFlag = 1U;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Left-turn judgement                                                        */
/* -------------------------------------------------------------------------- */

static uint8_t JudgeLeftTurnTlTime(
    const EgoVehicle *egoSnap,
    const TrafficLight *trafficLightSnap
)
{
    double egoTtc;
    double passableTimeSec;

    if ((trafficLightSnap->cz_x == 0U) &&
        (trafficLightSnap->cz_y == 0U))
    {
        return 0U;
    }

    if (trafficLightSnap->type == TL_NONE)
    {
        return 0U;
    }

    /*
    * 빨간불에는 신호 시간 경고를 표시하지 않는다.
    * 노란불에는 즉시 경고하고,
    * 녹색일 때는 잔여 시간과 ego TTC를 비교한다.
    */
    if (trafficLightSnap->color == SIG_YELLOW)
    {
        return 1U;
    }

    if (trafficLightSnap->color != SIG_GREEN)
    {
        return 0U; 
    }

    egoTtc = calculate_TTC(
        egoSnap->x,
        egoSnap->y,
        egoSnap->heading,
        egoSnap->speed,
        trafficLightSnap->cz_x,
        trafficLightSnap->cz_y,
        1U
    );

    passableTimeSec =
        (double)trafficLightSnap->time_left +
        TURN_JUDGE_YELLOW_DURATION_SEC;

    if (egoTtc < passableTimeSec)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t JudgeLeftTurnCandidate(
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap
)
{
    double egoTtc;
    double candidateTtc;
    double ttcGap;
    uint8_t candidateTurnLeft;

    egoTtc = calculate_TTC(
        egoSnap->x,
        egoSnap->y,
        egoSnap->heading,
        egoSnap->speed,
        candidateSnap->cz_x,
        candidateSnap->cz_y,
        1U
    );

    /*
     * 후보 차량도 자차와 동일하게 heading 기반 원호 거리로
     * 계산한다. 좌회전 후보(CAND_RT_OPP_LEFT)는 turn_left = 1,
     * 그 외(직진, 우회전)는 turn_left = 0으로 계산되며,
     * 직진 후보는 heading이 충돌구역 방향과 거의 일치하므로
     * 내부적으로 직선거리로 자연스럽게 수렴한다.
     */
    candidateTurnLeft = TurnJudge_GetCandidateTurnLeft(
        candidateSnap->type
    );

    candidateTtc = calculate_TTC(
        candidateSnap->x,
        candidateSnap->y,
        candidateSnap->heading,
        candidateSnap->speed,
        candidateSnap->cz_x,
        candidateSnap->cz_y,
        candidateTurnLeft
    );

    ttcGap = fabs(egoTtc - candidateTtc);

    /*
     * TTC 차이가 임계값보다 작으면 충돌 위험 경고.
     */
    if (ttcGap < LEFT_TURN_JUDGE_CRITICAL_GAP_SEC)
    {
        return 1U;
    }

    return 0U;
}

static void BuildLeftTurnDecision(
    Dicision *decision,
    const EgoVehicle *egoSnap,
    const CandidateVehicle *candidateSnap,
    const TrafficLight *trafficLightSnap
)
{
    if (decision == NULL)
    {
        return;
    }

    (void)memset(decision, 0, sizeof(Dicision));

    decision->turnState = MANEUVER_LEFT_TURN_UNPROT;

    if (JudgeLeftTurnTlTime(
            egoSnap,
            trafficLightSnap) != 0U)
    {
        decision->tlWarningFlag = 1U;
    }

    if ((candidateSnap->type & CAND_LT_OPP_STRAIGHT) != 0U)
    {
        if (JudgeLeftTurnCandidate(
                egoSnap,
                candidateSnap) != 0U)
        {
            decision->OppStraightFlag = 1U;
        }
    }

    if ((candidateSnap->type & CAND_LT_OPP_RIGHT) != 0U)
    {
        if (JudgeLeftTurnCandidate(
                egoSnap,
                candidateSnap) != 0U)
        {
            decision->OppRightFlag = 1U;
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Decision helpers                                                           */
/* -------------------------------------------------------------------------- */

static uint8_t TurnJudge_IsSameDecision(
    const Dicision *left,
    const Dicision *right
)
{
    if ((left == NULL) || (right == NULL))
    {
        return 0U;
    }

    if (left->turnState != right->turnState)
    {
        return 0U;
    }

    if (left->pedestrianFlag != right->pedestrianFlag)
    {
        return 0U;
    }

    if (left->LStraightFlag != right->LStraightFlag)
    {
        return 0U;
    }

    if (left->OppLeftFlag != right->OppLeftFlag)
    {
        return 0U;
    }

    if (left->tlWarningFlag != right->tlWarningFlag)
    {
        return 0U;
    }

    if (left->OppStraightFlag != right->OppStraightFlag)
    {
        return 0U;
    }

    if (left->OppRightFlag != right->OppRightFlag)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t DecisionBuzzerMask(const Dicision *decision)
{
    uint8_t mask = 0U;

    if (decision->pedestrianFlag == 1U) mask |= 0x01U;
    if (decision->LStraightFlag != 0U)  mask |= 0x02U;
    if (decision->OppLeftFlag != 0U)    mask |= 0x04U;
    if (decision->tlWarningFlag != 0U)  mask |= 0x08U;
    if (decision->OppStraightFlag != 0U) mask |= 0x10U;
    if (decision->OppRightFlag != 0U)   mask |= 0x20U;

    return mask;
}

/* -------------------------------------------------------------------------- */
/* TurnJudge task                                                             */
/* -------------------------------------------------------------------------- */

static void TurnJudge_Wait(void)
{
    if (gTurnJudgeSemCreated != 0U)
    {
        (void)SAL_SemaphoreWait(
            gTurnJudgeSem,
            0L,
            SAL_OPT_BLOCKING
        );
    }
}

void TurnJudge_Notify(void)
{
    if (gTurnJudgeSemCreated != 0U)
    {
        (void)SAL_SemaphoreRelease(gTurnJudgeSem);
    }
}

static void TurnJudgeTask(void *pArg)
{
    Dicision previousDecision;
    uint8_t hasPreviousDecision;

    (void)pArg;

    (void)memset(
        &previousDecision,
        0,
        sizeof(previousDecision)
    );

    hasPreviousDecision = 0U;

    mcu_printf("[JUDGE] Task started\n");

    for (;;)
    {
        EgoVehicle egoSnapshot;
        CandidateVehicle candidateSnapshot;
        TrafficLight trafficLightSnapshot;
        Dicision decision;

        uint8_t maneuverSnapshot;
        uint8_t pedestrianSnapshot;
        uint8_t shouldPost;
        uint8_t postResult;
        uint8_t isFirstDecision;

        /* 최신 공유 상태를 다시 판단하라는 알림을 기다린다. */
        TurnJudge_Wait();

        TurnJudge_GetSnapshots(
            &egoSnapshot,
            &candidateSnapshot,
            &trafficLightSnapshot,
            &maneuverSnapshot,
            &pedestrianSnapshot
        );

        (void)memset(
            &decision,
            0,
            sizeof(decision)
        );

        shouldPost = 1U;

        /*
         * 통신 오류가 있으면 LCD 통신 오류 화면으로 전달.
         */
        if ((candidateSnapshot.type == CAND_COMM_ERROR) ||
            (trafficLightSnapshot.type == TL_COMM_ERROR))
        {
            decision.turnState = 255U;
            decision.pedestrianFlag = pedestrianSnapshot;
        }
        else
        {
            switch (maneuverSnapshot)
            {
                case MANEUVER_STRAIGHT:
                {
                    decision.turnState = MANEUVER_STRAIGHT;
                    break;
                }

                case MANEUVER_RIGHT_TURN:
                {
                    BuildRightTurnDecision(
                        &decision,
                        pedestrianSnapshot,
                        &egoSnapshot,
                        &candidateSnapshot,
                        &trafficLightSnapshot
                    );
                    break;
                }

                case MANEUVER_LEFT_TURN_UNPROT:
                {
                    BuildLeftTurnDecision(
                        &decision,
                        &egoSnapshot,
                        &candidateSnapshot,
                        &trafficLightSnapshot
                    );
                    break;
                }

                case MANEUVER_LEFT_TURN_PROT:
                {
                    /*
                     * 보호 좌회전은 현재 별도 위험 판단 없이
                     * 좌회전 방향만 표시한다.
                     */
                    decision.turnState =
                        MANEUVER_LEFT_TURN_PROT;
                    break;
                }

                default:
                {
                    shouldPost = 0U;
                    break;
                }
            }
        }

        if (shouldPost == 0U)
        {
            hasPreviousDecision = 0U;
            continue;
        }

        /*
         * 이전 판단 결과와 달라졌을 때만 LCD Queue에 전달.
         */
        isFirstDecision = (hasPreviousDecision == 0U) ? 1U : 0U;

        if ((isFirstDecision != 0U) ||
            (TurnJudge_IsSameDecision(
                &previousDecision,
                &decision) == 0U))
        {
            postResult = Display_DicisionPost(&decision);

            if (postResult != 0U)
            {
                previousDecision = decision;
                hasPreviousDecision = 1U;

#if (MCU_BSP_SUPPORT_APP_BUZZER == 1)
                /* 최초 판단은 초기 상태로만 저장하고 부저를 울리지 않는다. */
                if ((isFirstDecision == 0U) &&
                    (DecisionBuzzerMask(&decision) != 0U))
                {
                    if (BUZZER_Request(BUZZER_ON) != SAL_RET_SUCCESS)
                    {
                        mcu_printf("[JUDGE] Buzzer request failed\n");
                    }
                }
#endif
            }
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Task creation                                                              */
/* -------------------------------------------------------------------------- */

void TurnJudge_AppCreate(void)
{
    SALRetCode_t result;

    if (gTurnJudgeCreated != 0U)
    {
        return;
    }

    if (gTurnJudgeSemCreated == 0U)
    {
        result = (SALRetCode_t)SAL_SemaphoreCreate(
            &gTurnJudgeSem,
            (const uint8 *)"Turn Judge",
            1UL,
            SAL_OPT_BLOCKING
        );

        if (result != SAL_RET_SUCCESS)
        {
            mcu_printf(
                "[JUDGE] Semaphore create failed: %d\n",
                (sint32)result
            );
            return;
        }

        gTurnJudgeSemCreated = 1U;

        /* 생성 직후의 초기 토큰을 제거해 첫 알림까지 태스크를 재운다. */
        (void)SAL_SemaphoreWait(
            gTurnJudgeSem,
            0L,
            SAL_OPT_NON_BLOCKING
        );
    }

    result = (SALRetCode_t)SAL_TaskCreate(
        &gTurnJudgeTaskId,
        (const uint8 *)"Turn Judge",
        (SALTaskFunc)&TurnJudgeTask,
        &gTurnJudgeTaskStack[0],
        TURN_JUDGE_TASK_STACK_SIZE,
        SAL_PRIO_APP_CFG,
        NULL
    );

    if (result == SAL_RET_SUCCESS)
    {
        gTurnJudgeCreated = 1U;

        mcu_printf("[JUDGE] Task created\n");
    }
    else
    {
        mcu_printf(
            "[JUDGE] Task create failed: %d\n",
            (sint32)result
        );
    }
}
