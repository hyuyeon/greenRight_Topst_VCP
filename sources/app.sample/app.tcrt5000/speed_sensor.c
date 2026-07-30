// speed_sensor.c
#include <gpio.h>
#include <timer.h>
#include <sal_api.h>
#include "speed_sensor.h"
#include <gic.h>  
#define PI_F                    3.1415926535f
#define WHEEL_DIAMETER_CM       7.0f
#define PULSES_PER_ROTATION     3.0f
#define SPEED_SAMPLE_PERIOD_MS  1000U

/* JSD104 16번 핀 = GPIO_A(6) : TCRT5000 콜렉터(디지털 입력)     */
/* JSD104 17번 핀 = GPIO_A(7) : 외장 LED(디지털 출력)            */
#define IR_SENSOR_PIN           GPIO_GPA(6)
#define LED_PIN                 GPIO_GPA(7)

volatile uint32 handler_call_cnt = 0; 
static volatile uint8  stable_state           = 0;
static volatile uint32 speed_sample_elapsed_ms = 0;
static volatile uint32 pulse_count            = 0;
static volatile uint32 rejected_cnt           = 0;
static volatile float  speed_mps              = 0.0f;



static sint32 SpeedSensor_1msHandler(TIMERChannel_t iChannel, void *pArgs)
{
    handler_call_cnt++;
    static uint8 last_raw = 0;
    static uint8 same_cnt = 0;
    (void)iChannel;
    (void)pArgs;

    uint8 raw = GPIO_Get(IR_SENSOR_PIN);

    if (raw == last_raw)
    {
        if (same_cnt < 3U) { same_cnt++; }
    }
    else
    {
        same_cnt = 0;
        last_raw = raw;
    }

    if ((same_cnt >= 3U) && (raw != stable_state))
    {
        stable_state = raw;
        if (stable_state == 1U)
        {
            pulse_count++;
            uint8 cur = GPIO_Get(LED_PIN);
            (void)GPIO_Set(LED_PIN, (cur == 0U) ? 1U : 0U);   // 검출될 때마다 LED 토글
        }
        else
        {
            rejected_cnt++;
        }
    }

    speed_sample_elapsed_ms++;
    if (speed_sample_elapsed_ms >= SPEED_SAMPLE_PERIOD_MS)
    {
        speed_sample_elapsed_ms = 0;
        uint32 captured_pulses;

        (void)SAL_CoreCriticalEnter();
        captured_pulses = pulse_count;
        pulse_count = 0;
        (void)SAL_CoreCriticalExit();

        float rotations = (float)captured_pulses / PULSES_PER_ROTATION;
        float wheel_circumference_cm = WHEEL_DIAMETER_CM * PI_F;
        float distance_m = (rotations * wheel_circumference_cm) / 100.0f;

        speed_mps = distance_m / (SPEED_SAMPLE_PERIOD_MS / 1000.0f);
        if (captured_pulses == 0U) { speed_mps = 0.0f; }
    }

    return 0;
}

void SpeedSensor_Init(void)
{
    (void)GPIO_Config(IR_SENSOR_PIN, GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLDN);
    (void)GPIO_Config(LED_PIN, GPIO_FUNC(0) | GPIO_OUTPUT);
    (void)GPIO_Set(LED_PIN, 0U);

    (void)TIMER_EnableWithMode(TIMER_CH_2, 1000U, TIMER_OP_FREERUN, &SpeedSensor_1msHandler, NULL);
   
    GIC_IntSrcEn((uint32)GIC_TIMER_0 + (uint32)TIMER_CH_2);   // 이걸 추가해야 실제로 인터럽트가 켜짐
}

float Speed_GetMps(void)
{
    return speed_mps;
}