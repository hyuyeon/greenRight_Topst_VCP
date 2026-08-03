#ifndef APP_TURN_JUDGE_TTC_H
#define APP_TURN_JUDGE_TTC_H

#include <stdint.h>

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif
#define TTC_SAFE                       (1000000.0)
#define TTC_CREEP_SPEED_M_PER_MIN      (15U)
/*
 * TTC 계산 내부에서 사용하는 2차원 좌표
 */
typedef struct
{
    double x;
    double y;
} Point;

/*
 * 두 좌표 사이의 직선거리 계산
 *
 * 입력 단위: cm
 * 반환 단위: cm
 */
double get_distance(
    double x1,
    double y1,
    double x2,
    double y2
);

/*
 * 자차가 충돌구역에 도착하기까지 걸리는 시간 계산
 *
 * ego       : 자차 위치, 속도, heading
 * cz_x/y    : 충돌구역 좌표(cm)
 * turn_left : 0 = 우회전 경로, 1 = 좌회전 경로
 *
 * 반환 단위: 초
 */
double calculate_Ego_TTC(
    EgoVehicle egoVehicle,
    uint16_t cz_x,
    uint16_t cz_y,
    uint8_t turn_left
);

/*
 * 후보 차량이 충돌구역에 도착하기까지 걸리는 시간 계산
 *
 * 반환 단위: 초
 */
double calculate_Cand_TTC(
    CandidateVehicle candidate
);

#ifdef __cplusplus
}
#endif

#endif /* APP_TURN_JUDGE_TTC_H */