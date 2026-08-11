#ifndef TEMPORAL_QOS_H
#define TEMPORAL_QOS_H

#include <stdint.h>

#include "common.h"

#define TEMPORAL_QOS_TIMESTAMP_MASK       (0x0FFFU)
#define TEMPORAL_QOS_FRESHNESS_LIMIT_MS   (300U)

uint16_t TemporalQos_CalculateAgeMs(
    uint16_t currentTimestamp,
    uint16_t sourceTimestamp
);

void TemporalQos_CompensateLatency(
    const CandidateVehicle *candiOrigin,
    CandidateVehicle *compensatedCandi,
    uint16_t latencyMs
);

#endif /* TEMPORAL_QOS_H */
