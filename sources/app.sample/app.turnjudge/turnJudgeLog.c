#include "turnJudgeLog.h"

#include <stddef.h>
#include <string.h>

#include <debug.h>
#include <sal_api.h>

/* 1: every judgement, 0: only successful decision posts */
#define TURN_JUDGE_LOG_ENABLE             (1U)
#define TURN_JUDGE_VALUE_UNAVAILABLE      (-1.0)

static const char *TurnJudgeLog_CaseName(TurnJudgeCase judgeCase)
{
    switch (judgeCase)
    {
        case TURN_JUDGE_CASE_RT_LEFT_STRAIGHT: return "RT_LEFT_STRAIGHT";
        case TURN_JUDGE_CASE_RT_OPP_LEFT:      return "RT_OPP_LEFT";
        case TURN_JUDGE_CASE_LT_TL_TIME:       return "LT_TL_TIME";
        case TURN_JUDGE_CASE_LT_OPP_STRAIGHT: return "LT_OPP_STRAIGHT";
        case TURN_JUDGE_CASE_LT_OPP_RIGHT:     return "LT_OPP_RIGHT";
        default:                               return "NONE";
    }
}

static const char *TurnJudgeLog_ReasonName(TurnJudgeReason reason)
{
    switch (reason)
    {
        case TURN_JUDGE_REASON_CZ_PASSED:           return "CZ_PASSED";
        case TURN_JUDGE_REASON_NO_TRAFFIC_LIGHT:    return "NO_TRAFFIC_LIGHT";
        case TURN_JUDGE_REASON_SIGNAL_PASSABLE:     return "SIGNAL_PASSABLE";
        case TURN_JUDGE_REASON_SIGNAL_NOT_GREEN:    return "SIGNAL_NOT_GREEN";
        case TURN_JUDGE_REASON_SIGNAL_YELLOW:       return "SIGNAL_YELLOW";
        case TURN_JUDGE_REASON_SIGNAL_TIME_SHORT:   return "SIGNAL_TIME_SHORT";
        case TURN_JUDGE_REASON_TTC_GAP_SAFE:        return "TTC_GAP_SAFE";
        case TURN_JUDGE_REASON_TTC_GAP_WARNING:     return "TTC_GAP_WARNING";
        case TURN_JUDGE_REASON_PEDESTRIAN:          return "PEDESTRIAN";
        case TURN_JUDGE_REASON_AI_ERROR:            return "AI_ERROR";
        case TURN_JUDGE_REASON_COMM_ERROR:          return "COMM_ERROR";
        case TURN_JUDGE_REASON_NONE:
        default:                                    return "NONE";
    }
}

static sint32 TurnJudgeLog_SecondsToMs(double seconds)
{
    if (seconds < 0.0)
    {
        return -1;
    }

    if (seconds >= 2147483.0)
    {
        return 2147483000;
    }

    return (sint32)((seconds * 1000.0) + 0.5);
}

static uint8_t TurnJudgeLog_HasWarning(const Dicision *decision)
{
    if ((decision->turnState == 255U) ||
        (decision->pedestrianFlag != 0U) ||
        (decision->LStraightFlag != 0U) ||
        (decision->OppLeftFlag != 0U) ||
        (decision->tlWarningFlag != 0U) ||
        (decision->OppStraightFlag != 0U) ||
        (decision->OppRightFlag != 0U))
    {
        return 1U;
    }

    return 0U;
}

TurnJudgeResult TurnJudgeLog_MakeResult(TurnJudgeCase judgeCase)
{
    TurnJudgeResult result;

    (void)memset(&result, 0, sizeof(result));

    result.evaluated = 1U;
    result.judgeCase = judgeCase;
    result.reason = TURN_JUDGE_REASON_NONE;
    result.egoTtcSec = TURN_JUDGE_VALUE_UNAVAILABLE;
    result.candidateTtcSec = TURN_JUDGE_VALUE_UNAVAILABLE;
    result.ttcGapSec = TURN_JUDGE_VALUE_UNAVAILABLE;
    result.referenceSec = TURN_JUDGE_VALUE_UNAVAILABLE;

    return result;
}

static const TurnJudgeResult *TurnJudgeLog_SelectResult(
    const Dicision *decision,
    const TurnJudgeLogContext *logContext,
    TurnJudgeResult *specialResult
)
{
    const TurnJudgeResult *result;
    uint8_t index;

    for (index = 0U; index < (uint8_t)TURN_JUDGE_CASE_COUNT; index++)
    {
        result = &logContext->results[index];
        if ((result->evaluated != 0U) &&
            (result->warning != 0U) &&
            (result->candidateTtcSec >= 0.0))
        {
            return result;
        }
    }

    for (index = 0U; index < (uint8_t)TURN_JUDGE_CASE_COUNT; index++)
    {
        result = &logContext->results[index];
        if ((result->evaluated != 0U) && (result->warning != 0U))
        {
            return result;
        }
    }

    if ((decision->turnState == 255U) || (decision->pedestrianFlag != 0U))
    {
        *specialResult = TurnJudgeLog_MakeResult(TURN_JUDGE_CASE_COUNT);
        specialResult->warning = 1U;

        if (decision->turnState == 255U)
        {
            specialResult->reason = TURN_JUDGE_REASON_COMM_ERROR;
        }
        else if (decision->pedestrianFlag == 2U)
        {
            specialResult->reason = TURN_JUDGE_REASON_AI_ERROR;
        }
        else
        {
            specialResult->reason = TURN_JUDGE_REASON_PEDESTRIAN;
        }

        return specialResult;
    }

    for (index = 0U; index < (uint8_t)TURN_JUDGE_CASE_COUNT; index++)
    {
        result = &logContext->results[index];
        if ((result->evaluated != 0U) && (result->candidateTtcSec >= 0.0))
        {
            return result;
        }
    }

    for (index = 0U; index < (uint8_t)TURN_JUDGE_CASE_COUNT; index++)
    {
        result = &logContext->results[index];
        if (result->evaluated != 0U)
        {
            return result;
        }
    }

    return NULL;
}

void TurnJudgeLog_Build(
    TurnJudgeLogMode mode,
    const Dicision *decision,
    const TurnJudgeLogContext *logContext
)
{
    const TurnJudgeResult *selected;
    const TurnJudgeResult *result;
    TurnJudgeResult specialResult;
    uint8_t index;

    if ((decision == NULL) || (logContext == NULL))
    {
        return;
    }

    if (mode == TURN_JUDGE_LOG_DETAIL)
    {
#if (TURN_JUDGE_LOG_ENABLE == 1U)
        for (index = 0U; index < (uint8_t)TURN_JUDGE_CASE_COUNT; index++)
        {
            result = &logContext->results[index];
            if (result->evaluated == 0U)
            {
                continue;
            }

            mcu_printf(
                "[JUDGE][DETAIL] case=%s result=%s reason=%s "
                "ego=%dms cand=%dms gap=%dms ref=%dms\n",
                TurnJudgeLog_CaseName(result->judgeCase),
                (result->warning != 0U) ? "WARNING" : "SAFE",
                TurnJudgeLog_ReasonName(result->reason),
                (sint32)TurnJudgeLog_SecondsToMs(result->egoTtcSec),
                (sint32)TurnJudgeLog_SecondsToMs(result->candidateTtcSec),
                (sint32)TurnJudgeLog_SecondsToMs(result->ttcGapSec),
                (sint32)TurnJudgeLog_SecondsToMs(result->referenceSec)
            );
        }
#endif
        return;
    }

    selected = TurnJudgeLog_SelectResult(
        decision,
        logContext,
        &specialResult
    );

    if (selected != NULL)
    {
        mcu_printf(
            "[JUDGE][POST] turn=%d ped=%d LS=%d OL=%d TL=%d OS=%d OR=%d "
            "case=%s result=%s reason=%s ego=%dms cand=%dms gap=%dms ref=%dms\n",
            (sint32)decision->turnState,
            (sint32)decision->pedestrianFlag,
            (sint32)decision->LStraightFlag,
            (sint32)decision->OppLeftFlag,
            (sint32)decision->tlWarningFlag,
            (sint32)decision->OppStraightFlag,
            (sint32)decision->OppRightFlag,
            TurnJudgeLog_CaseName(selected->judgeCase),
            (selected->warning != 0U) ? "WARNING" : "SAFE",
            TurnJudgeLog_ReasonName(selected->reason),
            (sint32)TurnJudgeLog_SecondsToMs(selected->egoTtcSec),
            (sint32)TurnJudgeLog_SecondsToMs(selected->candidateTtcSec),
            (sint32)TurnJudgeLog_SecondsToMs(selected->ttcGapSec),
            (sint32)TurnJudgeLog_SecondsToMs(selected->referenceSec)
        );
    }
    else
    {
        mcu_printf(
            "[JUDGE][POST] turn=%d ped=%d LS=%d OL=%d TL=%d OS=%d OR=%d "
            "case=NONE result=%s reason=NONE\n",
            (sint32)decision->turnState,
            (sint32)decision->pedestrianFlag,
            (sint32)decision->LStraightFlag,
            (sint32)decision->OppLeftFlag,
            (sint32)decision->tlWarningFlag,
            (sint32)decision->OppStraightFlag,
            (sint32)decision->OppRightFlag,
            (TurnJudgeLog_HasWarning(decision) != 0U) ? "WARNING" : "SAFE"
        );
    }
}
