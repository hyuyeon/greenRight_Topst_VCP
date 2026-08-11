#include "temporal_qos.h"

#include <math.h>
#include <stddef.h>

#define TEMPORAL_QOS_PI  (3.14159265358979323846)

//현재 동기화 시각의 하위 12 bit랑 sourceTimestamp(원래 12 bit)를 비교해서 age를 구함
uint16_t TemporalQos_CalculateAgeMs(
    uint16_t currentTimestamp,
    uint16_t sourceTimestamp
)
{
    return (uint16_t)(
        (currentTimestamp - sourceTimestamp) &
        TEMPORAL_QOS_TIMESTAMP_MASK
    );
}

//지연 보상 적용
void TemporalQos_CompensateLatency(
    const CandidateVehicle *candiOrigin,
    CandidateVehicle *compensatedCandi,
    uint16_t latencyMs)
{
    //candiOrigin의 x,y를 예측
    //원래의 x,y에서 latencyms동안 candiOrigin->speed, candiOrigin->heading으로 진행한 위치로 보정해서 compensatedCandi를 수정
    double headingRadian;
    double distanceCm;
    double predictedX;
    double predictedY;

    if ((candiOrigin == NULL) || (compensatedCandi == NULL))
    {
        return;
    }

    /* 원본의 모든 필드를 복사한 후 x, y만 보정한다. */
    *compensatedCandi = *candiOrigin;

    headingRadian =
        ((double)candiOrigin->heading * TEMPORAL_QOS_PI) / 180.0;

    distanceCm =
        ((double)candiOrigin->speed * (double)latencyMs) / 1000.0;

    predictedX =
        (double)candiOrigin->x +
        (distanceCm * sin(headingRadian));

    predictedY =
        (double)candiOrigin->y +
        (distanceCm * cos(headingRadian));

    /*
     * uint16_t 변환 전에 좌표 범위 제한 필요.
     * 실제 지도 좌표 최댓값이 2047이라면 65535 대신 2047 사용.
     */
    if (predictedX < 0.0)
    {
        predictedX = 0.0;
    }
    else if (predictedX > 2047.0)
    {
        predictedX = 2047.0;
    }

    if (predictedY < 0.0)
    {
        predictedY = 0.0;
    }
    else if (predictedY > 2047.0)
    {
        predictedY = 2047.0;
    }

    compensatedCandi->x = (uint16_t)(predictedX + 0.5);
    compensatedCandi->y = (uint16_t)(predictedY + 0.5);
}