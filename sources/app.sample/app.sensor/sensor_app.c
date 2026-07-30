/*
 * sensor_app.c
 *
 * Integrated sensor task for VCP-G.
 *
 * Shared I2C1 bus:
 *   BNO055  : 0x28
 *   ADXL345 : 0x53
 *
 * Update period:
 *   BNO055   : 20 ms
 *   ADXL345  : 20 ms
 *   Button   : 20 ms
 *   LED blink: 100 ms
 */

#include "sensor_app.h"

#include "bno055.h"
#include "bno055_app.h"
#include "adxl345.h"
// #include "adxl345_app.h"
#include "i2c1_bus.h"
#include "led.h"
#include "exti_button.h"
#include "speed_sensor.h"
#include "position.h"

#include <sal_internal.h>
#include <app_cfg.h>
#include <debug.h>
#include <math.h>

#define SENSOR_UPDATE_PERIOD_MS   20U
#define LED_BLINK_PERIOD_MS      500U
typedef enum
{
    IDLE = 0,
    WAIT_STEER_RIGHT,
    WAIT_RETURN_RIGHT,
    WAIT_STEER_LEFT,
    WAIT_RETURN_LEFT
} State_t;

State_t state = IDLE;

static void Sensor_Task(void *pArg)
{
    uint8_t adxl_available = 0U;

    uint8_t blink_on = 0U;
    uint32_t blink_elapsed_ms = 0U;

    int16_t ax = 0;
    int16_t ay = 0;
    int16_t az = 0;

    float roll = 0.0f;
    float pitch = 0.0f;
    float speed_mps = 0.0f;

    uint32_t speed_log_elapsed_ms = 0U;

    int32_t raw_heading_x100 = 0;
    int32_t filtered_heading_x100 = 0;
    int32_t current_heading_x100 = 0;

    uint8_t heading_valid = 0U;
    uint8_t bno_read_ok = 0U;
    uint8_t zero_count = 0U;

    (void)pArg;

    /*
     * BNO055와 ADXL345가 I2C1을 공유한다.
     *
     * SCL: GPIO_B2
     * SDA: GPIO_B3
     */
    I2C1_Bus_Init();

    /*
     * GPIO 및 센서 초기화
     */
    LED_Init();
    EXTI_Button_Init();
    SpeedSensor_Init();
    Position_Reset();

    /*
     * BNO055 초기화
     */
    if (!BNO055_Init())
    {
        mcu_printf("[BNO055] Init Failed\n");

        heading_valid = 0U;
        BNO055_UpdateSharedData(0, 0U);
    }
    else
    {
        mcu_printf("[BNO055] Init OK\n");
    }

    /*
     * ADXL345 초기화
     */
    mcu_printf("[ADXL345] Initialization...\n");

    if (ADXL345_Init())
    {
        adxl_available = 1U;

        mcu_printf("[ADXL345] Ready!\n");
    }
    else
    {
        adxl_available = 0U;

        mcu_printf(
            "[ADXL345] disabled. Continue without accel.\n");
    }

    while (1)
    {
        /*
         * 이번 반복 주기의 BNO055 읽기 상태 초기화
         */
        bno_read_ok = 0U;

        /*
         * BNO055 Heading 처리
         */
        if (BNO055_ReadHeading())
        {
            bno_read_ok = 1U;

            raw_heading_x100 =
                IMU_GetHeadingX100();

            filtered_heading_x100 =
                ApplyHeadingDeadband_X100(
                    raw_heading_x100);

            current_heading_x100 =
                filtered_heading_x100;

            heading_valid =
                IMU_IsValid();

            BNO055_UpdateSharedData(
                current_heading_x100,
                heading_valid);

            /*
             * Heading이 3회 연속 0이면
             * BNO055의 동작 모드를 확인한다.
             */
            if (raw_heading_x100 == 0)
            {
                zero_count++;

                if (zero_count == 3U)
                {
                    uint8_t mode = 0U;
                    uint8_t sys = 0U;
                    uint8_t err = 0U;

                    uint8_t mode_ok;
                    uint8_t sys_ok;
                    uint8_t err_ok;

                    uint8_t actual_mode;

                    mode_ok =
                        I2C1_ReadReg(
                            0x28U,
                            0x3DU,
                            &mode);

                    sys_ok =
                        I2C1_ReadReg(
                            0x28U,
                            0x39U,
                            &sys);

                    err_ok =
                        I2C1_ReadReg(
                            0x28U,
                            0x3AU,
                            &err);

                    actual_mode =
                        mode & 0x0FU;

                    mcu_printf(
                        "[BNO DBG] "
                        "opr_raw=%d mode=%d "
                        "sys=%d err=%d "
                        "read_ok=%d/%d/%d\n",
                        (int)mode,
                        (int)actual_mode,
                        (int)sys,
                        (int)err,
                        (int)mode_ok,
                        (int)sys_ok,
                        (int)err_ok);

                    /*
                     * OPR_MODE 하위 4비트가 0이면
                     * CONFIGMODE 상태이다.
                     */
                    if ((mode_ok != 0U) &&
                        (actual_mode == 0U))
                    {
                        mcu_printf(
                            "[BNO055] CONFIGMODE detected. "
                            "Reinitializing...\n");

                        /*
                         * 현재 주기에서는 Heading을
                         * 위치 계산에 사용하지 않는다.
                         */
                        heading_valid = 0U;
                        bno_read_ok = 0U;

                        BNO055_UpdateSharedData(
                            0,
                            0U);

                        if (BNO055_Init())
                        {
                            mcu_printf(
                                "[BNO055] Reinit OK\n");

                            /*
                             * 다음 반복에서 새 Heading을
                             * 읽도록 카운터를 초기화한다.
                             */
                            zero_count = 0U;
                        }
                        else
                        {
                            mcu_printf(
                                "[BNO055] Reinit Failed\n");

                            /*
                             * zero_count가 3인 상태를 유지해
                             * 매 주기 반복 초기화되는 것을 막는다.
                             */
                        }
                    }
                }
            }
            else
            {
                zero_count = 0U;
            }
        }
        else
        {
            mcu_printf(
                "[BNO055] Read Failed\n");

            bno_read_ok = 0U;
            heading_valid = 0U;

            BNO055_UpdateSharedData(
                0,
                0U);
        }

        /*
         * TCRT5000에서 계산된 최신 속도 조회
         */
        speed_mps = Speed_GetMps();

        /*
         * 속도와 Heading을 이용한 위치 추정
         *
         * Heading이 유효하지 않을 때도 속도 0으로 호출하여
         * position.c 내부의 기준 시간은 계속 갱신한다.
         */
        if (heading_valid != 0U)
        {
            Position_Update(
                speed_mps,
                current_heading_x100);
        }
        else
        {
            Position_Update(
                0.0f,
                current_heading_x100);
        }

        /*
         * 위치 갱신 이후 Heading과 X, Y 좌표 출력
         */
        if (bno_read_ok != 0U)
        {
            mcu_printf(
                "[BNO055] "
                "heading=%d.%02d deg "
                "(raw=%d) "
                "x=%d cm y=%d cm\n",
                filtered_heading_x100 / 100,
                (filtered_heading_x100 < 0
                    ? -filtered_heading_x100
                    : filtered_heading_x100) % 100,
                (int)raw_heading_x100,
                (int)Position_GetXcm(),
                (int)Position_GetYcm());
        }

        /*
         * 버튼 인터럽트 이벤트 처리
         */
        EXTI_Button_Process(
            SENSOR_UPDATE_PERIOD_MS);

        /*
         * ADXL345 처리
         */
        if (adxl_available != 0U)
        {
            if (!ADXL345_ReadXYZ(
                    &ax,
                    &ay,
                    &az))
            {
                adxl_available = 0U;
                state = IDLE;

                LED_AllOff();

                mcu_printf(
                    "[ADXL345] read failed. "
                    "Accel disabled.\n");
            }
            else
            {
                RollPitch_Calc(
                    ax,
                    ay,
                    az,
                    &roll,
                    &pitch);

                /*
                 * 현재는 Roll만 사용하므로
                 * Pitch의 미사용 경고를 방지한다.
                 */
                (void)pitch;

                switch (state)
                {
                    case IDLE:
                    {
                        LED_AllOff();
                        blink_on = 0U;
                        break;
                    }

                    case WAIT_STEER_RIGHT:
                    {
                        if (roll >= 40.0f)
                        {
                            state =
                                WAIT_RETURN_RIGHT;
                        }

                        break;
                    }

                    case WAIT_RETURN_RIGHT:
                    {
                        if (fabsf(roll) <= 5.0f)
                        {
                            state = IDLE;
                        }

                        break;
                    }

                    case WAIT_STEER_LEFT:
                    {
                        if (roll <= -40.0f)
                        {
                            state =
                                WAIT_RETURN_LEFT;
                        }

                        break;
                    }

                    case WAIT_RETURN_LEFT:
                    {
                        if (fabsf(roll) <= 5.0f)
                        {
                            state = IDLE;
                        }

                        break;
                    }

                    default:
                    {
                        state = IDLE;
                        break;
                    }
                }
            }
        }

        /*
         * 방향지시등 LED 점멸 처리
         */
        if (state != IDLE)
        {
            blink_elapsed_ms +=
                SENSOR_UPDATE_PERIOD_MS;

            if (blink_elapsed_ms >=
                LED_BLINK_PERIOD_MS)
            {
                blink_elapsed_ms = 0U;

                blink_on =
                    (blink_on == 0U)
                    ? 1U
                    : 0U;
            }

            if ((state == WAIT_STEER_RIGHT) ||
                (state == WAIT_RETURN_RIGHT))
            {
                if (blink_on != 0U)
                {
                    LED_RightOn();
                }
                else
                {
                    LED_RightOff();
                }

                LED_LeftOff();
            }
            else if (
                (state == WAIT_STEER_LEFT) ||
                (state == WAIT_RETURN_LEFT))
            {
                if (blink_on != 0U)
                {
                    LED_LeftOn();
                }
                else
                {
                    LED_LeftOff();
                }

                LED_RightOff();
            }
        }
        else
        {
            LED_AllOff();

            blink_on = 0U;
            blink_elapsed_ms = 0U;
        }

        /*
         * 속도 및 누적 위치를 1초마다 출력
         */
        speed_log_elapsed_ms +=
            SENSOR_UPDATE_PERIOD_MS;

        if (speed_log_elapsed_ms >= 1000U)
        {
            int whole;
            int frac;

            speed_log_elapsed_ms = 0U;

            whole =
                (int)speed_mps;

            frac =
                (int)(
                    (speed_mps -
                     (float)whole) *
                    100.0f);

            if (frac < 0)
            {
                frac = -frac;
            }

            mcu_printf(
                "[POSITION] "
                "speed=%d.%02d m/s "
                "x=%d cm y=%d cm "
                "distance=%d cm\n",
                whole,
                frac,
                (int)Position_GetXcm(),
                (int)Position_GetYcm(),
                (int)Position_GetTotalDistanceCm());
        }

        SAL_TaskSleep(
            SENSOR_UPDATE_PERIOD_MS);
    }
}

void Sensor_AppCreate(void)
{
    static uint32 taskID = 0UL;
    static uint32 taskStk[
        ACFG_TASK_MEDIUM_STK_SIZE];

    (void)SAL_TaskCreate(
        &taskID,
        (const uint8 *)"Sensor Task",
        (SALTaskFunc)&Sensor_Task,
        &taskStk[0],
        ACFG_TASK_MEDIUM_STK_SIZE,
        SAL_PRIO_BNO055_APP,
        NULL);
}