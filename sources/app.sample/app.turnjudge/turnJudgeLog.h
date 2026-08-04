#ifndef APP_TURN_JUDGE_LOG_H
#define APP_TURN_JUDGE_LOG_H

#include <stdint.h>

#include "common.h"

typedef enum
{
    TURN_JUDGE_CASE_RT_LEFT_STRAIGHT = 0,
    TURN_JUDGE_CASE_RT_OPP_LEFT,
    TURN_JUDGE_CASE_LT_TL_TIME,
    TURN_JUDGE_CASE_LT_OPP_STRAIGHT,
    TURN_JUDGE_CASE_LT_OPP_RIGHT,
    TURN_JUDGE_CASE_COUNT
} TurnJudgeCase;

typedef enum
{
    TURN_JUDGE_REASON_NONE = 0,
    TURN_JUDGE_REASON_CZ_PASSED,
    TURN_JUDGE_REASON_NO_TRAFFIC_LIGHT,
    TURN_JUDGE_REASON_SIGNAL_PASSABLE,
    TURN_JUDGE_REASON_SIGNAL_NOT_GREEN,
    TURN_JUDGE_REASON_SIGNAL_YELLOW,
    TURN_JUDGE_REASON_SIGNAL_TIME_SHORT,
    TURN_JUDGE_REASON_TTC_GAP_SAFE,
    TURN_JUDGE_REASON_TTC_GAP_WARNING,
    TURN_JUDGE_REASON_PEDESTRIAN,
    TURN_JUDGE_REASON_AI_ERROR,
    TURN_JUDGE_REASON_COMM_ERROR
} TurnJudgeReason;

typedef struct
{
    uint8_t evaluated;
    uint8_t warning;
    TurnJudgeCase judgeCase;
    TurnJudgeReason reason;
    double egoTtcSec;
    double candidateTtcSec;
    double ttcGapSec;
    double referenceSec;
} TurnJudgeResult;

typedef struct
{
    TurnJudgeResult results[TURN_JUDGE_CASE_COUNT];
} TurnJudgeLogContext;

typedef enum
{
    TURN_JUDGE_LOG_DETAIL = 0,
    TURN_JUDGE_LOG_POST
} TurnJudgeLogMode;

TurnJudgeResult TurnJudgeLog_MakeResult(
    TurnJudgeCase judgeCase
);

void TurnJudgeLog_Build(
    TurnJudgeLogMode mode,
    const Dicision *decision,
    const TurnJudgeLogContext *logContext
);

#endif /* APP_TURN_JUDGE_LOG_H */
