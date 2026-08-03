#include <math.h>
#include <stdint.h>

#include "ttc.h"

/*
 * 거리 단위: cm (좌표계와 동일하게 사용, 별도 m 변환 없음)
 */

/*
 * speed 필드는 이미 cm/s 단위로 들어온다.
 * 별도의 단위 변환 없이 최소 속도(크리핑) 클램프만 적용한다.
 */

/*
 * 부동소수점 속도 0 비교용 (cm/s 기준)
 */
#define TTC_SPEED_EPS_CM_PER_SEC       (1.0e-4)

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

static double TTC_ApplyCreepFloorCmPerSec(
    uint8_t speedCmPerSec
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

double calculate_TTC(
    uint16_t x,
    uint16_t y,
    uint16_t heading,
    uint8_t speed,
    uint16_t cz_x,
    uint16_t cz_y,
    uint8_t turn_left
)
{
    Point position;
    Point conflictZone;

    double speedCmps;
    double headingDegree;
    double distanceCm;

    speedCmps = TTC_ApplyCreepFloorCmPerSec(speed);

    if (speedCmps < TTC_SPEED_EPS_CM_PER_SEC)
    {
        return TTC_SAFE;
    }

    position.x = (double)x;
    position.y = (double)y;

    conflictZone.x = (double)cz_x;
    conflictZone.y = (double)cz_y;

    /*
     * 프로젝트 heading 좌표계:
     * 0도   = +y 방향
     * 90도  = +x 방향
     * 시계 방향으로 증가
     *
     * 자차, 후보 차량 모두 동일한 규칙을 사용한다.
     */
    headingDegree = (double)heading;

    distanceCm = TTC_ComputeArcDistanceByHeading(
        position,
        headingDegree,
        turn_left,
        conflictZone
    );

    if (distanceCm <= TTC_CZ_ARRIVED_DIST_CM)
    {
        return 0.0;
    }

    /*
     * cm / (cm/s) = s
     */
    return distanceCm / speedCmps;
}

/* -------------------------------------------------------------------------- */
/* Internal functions                                                         */
/* -------------------------------------------------------------------------- */

static double TTC_ApplyCreepFloorCmPerSec(
    uint8_t speedCmPerSec
)
{
    uint8_t effectiveSpeedCmPerSec;

    effectiveSpeedCmPerSec = speedCmPerSec;

    /*
     * 저속 또는 정지 상태에서도 계산용 최소 속도를 적용한다.
     * 기존 1차 프로젝트에서 15 m/min으로 클램프하던 것을
     * 동일한 물리적 속도(25 cm/s)로 환산해 유지한다.
     */
    if (effectiveSpeedCmPerSec <= TTC_CREEP_SPEED_CM_PER_SEC)
    {
        effectiveSpeedCmPerSec =
            TTC_CREEP_SPEED_CM_PER_SEC;
    }

    /*
     * speed 필드는 이미 cm/s 단위이므로 추가 변환 없이 반환한다.
     */
    return (double)effectiveSpeedCmPerSec;
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
     *
     * 직진하는 차량(heading이 충돌구역 방향과 거의 일치하는 경우)도
     * 이 분기를 통해 자연스럽게 직선거리로 계산된다.
     */
    if (dotProduct < TTC_DEGENERATE_LEN_EPS)
    {
        return straightDistance;
    }

    /*
     * 현재 heading을 접선으로 하며 현재 위치와 충돌구역을
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
