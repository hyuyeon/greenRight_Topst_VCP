#include <math.h>
#include <stdint.h>

#include "ttc.h"

/*
 * 거리 단위 변환
 * cm -> m
 */
#define TTC_CM_TO_M                    (0.01)

/*
 * 속도 단위 변환
 * m/min -> m/s
 */
#define TTC_MIN_TO_SEC                 (60.0)

/*
 * 부동소수점 속도 0 비교용
 */
#define TTC_SPEED_EPS_M_PER_SEC        (1.0e-6)

/*
 * 충돌구역 도착 판정 허용거리
 * 단위: cm
 */
#define TTC_CZ_ARRIVED_DIST_CM         (5.0)

/*
 * 거리 및 벡터 계산에서 0으로 취급할 값
 */
#define TTC_DEGENERATE_LEN_EPS         (1.0e-3)

#define TTC_PI                         (3.14159265358979323846)

/* -------------------------------------------------------------------------- */
/* Internal functions                                                         */
/* -------------------------------------------------------------------------- */

static double TTC_SpeedMPerMinToMPerSec(
    uint8_t speedMPerMin
);

static double TTC_DistanceCmToM(
    double distanceCm
);

static double TTC_ComputeArcDistanceByHeading(
    Point position,
    double headingDegree,
    uint8_t turnLeft,
    Point conflictZone
);

/* -------------------------------------------------------------------------- */
/* Public functions                                                           */
/* -------------------------------------------------------------------------- */

double get_distance(
    double x1,
    double y1,
    double x2,
    double y2
)
{
    double dx;
    double dy;

    dx = x2 - x1;
    dy = y2 - y1;

    return sqrt((dx * dx) + (dy * dy));
}

double calculate_Ego_TTC(
    EgoVehicle egoVehicle,
    uint16_t cz_x,
    uint16_t cz_y,
    uint8_t turn_left
)
{
    Point egoPosition;
    Point conflictZone;

    double speedMps;
    double headingDegree;
    double distanceCm;
    double distanceM;

    speedMps = TTC_SpeedMPerMinToMPerSec(egoVehicle.speed);

    if (speedMps < TTC_SPEED_EPS_M_PER_SEC)
    {
        return TTC_SAFE;
    }

    egoPosition.x = (double)egoVehicle.x;
    egoPosition.y = (double)egoVehicle.y;

    conflictZone.x = (double)cz_x;
    conflictZone.y = (double)cz_y;

    /*
     * 프로젝트 heading 좌표계:
     * 0도   = +y 방향
     * 90도  = +x 방향
     * 시계 방향으로 증가
     */
    headingDegree = (double)egoVehicle.heading;

    distanceCm = TTC_ComputeArcDistanceByHeading(
        egoPosition,
        headingDegree,
        turn_left,
        conflictZone
    );

    if (distanceCm <= TTC_CZ_ARRIVED_DIST_CM)
    {
        return 0.0;
    }

    distanceM = TTC_DistanceCmToM(distanceCm);

    return distanceM / speedMps;
}

double calculate_Cand_TTC(
    CandidateVehicle candidate
)
{
    Point candidatePosition;
    Point conflictZone;

    double speedMps;
    double distanceCm;
    double distanceM;

    speedMps = TTC_SpeedMPerMinToMPerSec(candidate.speed);

    if (speedMps < TTC_SPEED_EPS_M_PER_SEC)
    {
        return TTC_SAFE;
    }

    candidatePosition.x = (double)candidate.x;
    candidatePosition.y = (double)candidate.y;

    conflictZone.x = (double)candidate.cz_x;
    conflictZone.y = (double)candidate.cz_y;

    /*
     * 현재 후보 차량은 충돌구역까지 직진한다고 가정한다.
     */
    distanceCm = get_distance(
        candidatePosition.x,
        candidatePosition.y,
        conflictZone.x,
        conflictZone.y
    );

    if (distanceCm <= TTC_CZ_ARRIVED_DIST_CM)
    {
        return 0.0;
    }

    distanceM = TTC_DistanceCmToM(distanceCm);

    return distanceM / speedMps;
}

/* -------------------------------------------------------------------------- */
/* Internal functions                                                         */
/* -------------------------------------------------------------------------- */

static double TTC_SpeedMPerMinToMPerSec(
    uint8_t speedMPerMin
)
{
    uint8_t effectiveSpeedMPerMin;

    effectiveSpeedMPerMin = speedMPerMin;

    /*
     * 저속 또는 정지 상태에서도 계산용 최소 속도를 적용한다.
     * 기존 1차 프로젝트 동작을 그대로 유지한 부분이다.
     */
    if (effectiveSpeedMPerMin <= TTC_CREEP_SPEED_M_PER_MIN)
    {
        effectiveSpeedMPerMin =
            TTC_CREEP_SPEED_M_PER_MIN;
    }

    return ((double)effectiveSpeedMPerMin) /
           TTC_MIN_TO_SEC;
}

static double TTC_DistanceCmToM(
    double distanceCm
)
{
    return distanceCm * TTC_CM_TO_M;
}

static double TTC_ComputeArcDistanceByHeading(
    Point position,
    double headingDegree,
    uint8_t turnLeft,
    Point conflictZone
)
{
    double headingRadian;

    double directionX;
    double directionY;

    double normalX;
    double normalY;

    double dx;
    double dy;

    double distanceSquared;
    double straightDistance;
    double dotProduct;

    double radius;
    double ratio;
    double theta;

    headingRadian =
        headingDegree * TTC_PI / 180.0;

    /*
     * 프로젝트 heading은 0도가 +y이고 시계 방향으로 증가하므로
     * 일반적인 (cos, sin)이 아니라 (sin, cos)을 사용한다.
     */
    directionX = sin(headingRadian);
    directionY = cos(headingRadian);

    /*
     * 진행 방향에 수직인 회전 중심 방향 벡터.
     *
     * 좌회전: 진행 방향에서 반시계 방향 90도
     * 우회전: 진행 방향에서 시계 방향 90도
     */
    if (turnLeft != 0U)
    {
        normalX = -directionY;
        normalY = directionX;
    }
    else
    {
        normalX = directionY;
        normalY = -directionX;
    }

    dx = conflictZone.x - position.x;
    dy = conflictZone.y - position.y;

    distanceSquared = (dx * dx) + (dy * dy);
    straightDistance = sqrt(distanceSquared);

    /*
     * 현재 위치가 충돌구역과 거의 같으면 이동거리 0.
     */
    if (straightDistance < TTC_DEGENERATE_LEN_EPS)
    {
        return 0.0;
    }

    /*
     * 현재 위치에서 충돌구역으로 향하는 벡터를
     * 회전 중심 방향 벡터에 투영한다.
     */
    dotProduct =
        (dx * normalX) +
        (dy * normalY);

    /*
     * 지정한 회전 방향으로 원호가 만들어지지 않거나
     * 곡률 계산이 불안정한 경우 직선거리로 대체한다.
     */
    if (dotProduct < TTC_DEGENERATE_LEN_EPS)
    {
        return straightDistance;
    }

    /*
     * 현재 heading을 접선으로 가지며 현재 위치와 충돌구역을
     * 모두 지나는 원의 반지름.
     */
    radius =
        distanceSquared /
        (2.0 * dotProduct);

    /*
     * 현재 위치와 충돌구역 사이의 직선거리를 현으로 보고
     * 중심각을 계산한다.
     */
    ratio =
        straightDistance /
        (2.0 * radius);

    /*
     * 부동소수점 오차 때문에 asin() 입력 범위가
     * 1보다 약간 커지는 것을 방지한다.
     */
    if (ratio > 1.0)
    {
        ratio = 1.0;
    }

    theta = 2.0 * asin(ratio);

    /*
     * 원호 길이 = 반지름 × 중심각
     */
    return radius * theta;
}