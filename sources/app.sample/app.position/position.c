/*
 * position.c
 */

#include "position.h"

#include <sal_internal.h>
#include <math.h>

#define PI_F                      3.1415926535f
#define SPEED_STOP_THRESHOLD_MPS  0.02f
#define POSITION_START_X_CM       267
#define POSITION_START_Y_CM       10

static float total_distance_m = 0.0f;
static float pos_x_m = 0.0f;
static float pos_y_m = 0.0f;

static uint32 last_pos_time = 0U;

void Position_Reset(void)
{
    uint32 now = 0U;

    pos_x_m = (float)POSITION_START_X_CM / 100.0f;
    pos_y_m = (float)POSITION_START_Y_CM / 100.0f;
    total_distance_m = 0.0f;

    if (SAL_GetTickCount(&now) == SAL_RET_SUCCESS)
    {
        last_pos_time = now;
    }
    else
    {
        last_pos_time = 0U;
    }
}

void Position_Update(
    float current_speed_mps,
    int32 heading_x100)
{
    uint32 now = 0U;
    uint32 elapsed_ms;

    float dt;
    float heading_deg;
    float heading_rad;
    float distance_m;

    /*
     * 현재 시간 읽기에 실패하면 이번 위치 갱신을 건너뛴다.
     */
    if (SAL_GetTickCount(&now) != SAL_RET_SUCCESS)
    {
        return;
    }

    if (last_pos_time == 0U)
    {
        last_pos_time = now;
        return;
    }

    /*
     * unsigned 뺄셈이므로 tick overflow가 발생해도
     * 한 번의 wrap 범위 안에서는 경과시간 계산이 가능하다.
     */
    elapsed_ms = now - last_pos_time;
    last_pos_time = now;

    if (current_speed_mps < SPEED_STOP_THRESHOLD_MPS)
    {
        return;
    }

    dt = (float)elapsed_ms / 1000.0f;

    heading_deg = (float)heading_x100 / 100.0f;
    heading_rad = heading_deg * PI_F / 180.0f;

    distance_m = current_speed_mps * dt;

    total_distance_m += distance_m;

    /*
     * Heading 0도: +Y
     * Heading 90도: +X
     */
    pos_x_m += distance_m * sinf(heading_rad);
    pos_y_m += distance_m * cosf(heading_rad);
}

int32 Position_GetXcm(void)
{
    return (int32)(pos_x_m * 100.0f);
}

int32 Position_GetYcm(void)
{
    return (int32)(pos_y_m * 100.0f);
}

int32 Position_GetTotalDistanceCm(void)
{
    return (int32)(total_distance_m * 100.0f);
}