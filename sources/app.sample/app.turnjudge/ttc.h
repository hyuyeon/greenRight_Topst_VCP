#ifndef APP_TURN_JUDGE_TTC_H
#define APP_TURN_JUDGE_TTC_H

#include <stdint.h>

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif
#define TTC_SAFE                       (1000000.0)
/*
 * 최소 속도(크리핑) 클램프 값.
 * 물리적으로 기존 15 m/min과 동일한 속도를 cm/s로 환산한 값.
 * (15 m/min = 0.25 m/s = 25 cm/s)
 */
#define TTC_CREEP_SPEED_CM_PER_SEC     (25U)
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
 * 차량(자차 또는 후보 차량)이 충돌구역에 도착하기까지 걸리는 시간 계산
 *
 * x, y      : 차량 위치(cm)
 * heading   : 차량 heading (0=+y, 90=+x, 시계방향 증가)
 * speed     : 차량 속도(cm/s)
 * cz_x/y    : 충돌구역 좌표(cm)
 * turn_left : 0 = 우회전/직진 경로, 1 = 좌회전 경로
 *
 * 반환 단위: 초
 */
double calculate_TTC(
    uint16_t x,
    uint16_t y,
    uint16_t heading,
    uint8_t speed,
    uint16_t cz_x,
    uint16_t cz_y,
    uint8_t turn_left
);

#ifdef __cplusplus
}
#endif

#endif /* APP_TURN_JUDGE_TTC_H */
