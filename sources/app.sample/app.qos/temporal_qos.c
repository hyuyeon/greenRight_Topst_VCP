#include "temporal_qos.h"

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

//현재 동기화 시각의 하위 12 bit랑 sourceTimestamp(원래 12 bit)를 비교해서 freshness를 체크
uint8_t TemporalQos_CheckFreshness(
    uint16_t currentTimestamp,
    uint16_t sourceTimestamp
)
{
    uint16_t ageMs;

    ageMs = TemporalQos_CalculateAgeMs(
        currentTimestamp,
        sourceTimestamp
    );

    return (ageMs <= TEMPORAL_QOS_FRESHNESS_LIMIT_MS)
        ? 1U
        : 0U;
}
