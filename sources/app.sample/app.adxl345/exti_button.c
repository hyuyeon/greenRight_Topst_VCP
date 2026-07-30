/*
 * exti_button.c
 *
 * J18D100:
 *   GPIO_B22 = right button
 *   GPIO_B21 = left button
 *
 * Current button behavior:
 *   - active-low button input
 *   - falling-edge detection
 *   - one shared 200 ms debounce interval
 *
 * Button wiring:
 *   GPIO internal pull-up enabled
 *   GPIO -> push button -> GND
 *
 * Note:
 *   The current implementation uses 20 ms polling,
 *   not a hardware external interrupt.
 */

#include "exti_button.h"
#include "adxl345_app.h"

#include <gpio.h>
#include <bsp.h>
#include <sal_api.h>
#include <gic.h>

#define RIGHT_BUTTON_PIN_NUM  22UL
#define LEFT_BUTTON_PIN_NUM   21UL
#define RIGHT_BUTTON_INT_ID  GIC_EXT0
#define LEFT_BUTTON_INT_ID   GIC_EXT1
#define BUTTON_DEBOUNCE_MS    200U

#define RIGHT_BUTTON_PIN      GPIO_GPB(RIGHT_BUTTON_PIN_NUM)
#define LEFT_BUTTON_PIN       GPIO_GPB(LEFT_BUTTON_PIN_NUM)
static volatile uint8 s_right_button_event = 0U;
static volatile uint8 s_left_button_event  = 0U;

static uint32 s_debounce_elapsed_ms = BUTTON_DEBOUNCE_MS;
/*
 * 오른쪽 버튼 외부 인터럽트 콜백
 *
 * EDGE_BOTH로 등록하므로 눌림과 해제 시 모두 호출될 수 있다.
 * 실제 GPIO가 LOW일 때만 버튼 눌림 이벤트로 저장한다.
 */
static void EXTI_RightButtonHandler(void *pArg)
{
    (void)pArg;

    if (GPIO_Get(RIGHT_BUTTON_PIN) == 0U)
    {
        s_right_button_event = 1U;
    }
}


/*
 * 왼쪽 버튼 외부 인터럽트 콜백
 */
static void EXTI_LeftButtonHandler(void *pArg)
{
    (void)pArg;

    if (GPIO_Get(LEFT_BUTTON_PIN) == 0U)
    {
        s_left_button_event = 1U;
    }
}


void EXTI_Button_Init(void)
{
    /*
     * 버튼은 GPIO와 GND 사이에 연결되어 있으므로
     * 내부 Pull-up을 사용한다.
     */
    (void)GPIO_Config(
        RIGHT_BUTTON_PIN,
        GPIO_FUNC(0UL) |
        GPIO_INPUT |
        GPIO_INPUTBUF_EN |
        GPIO_PULLUP);

    (void)GPIO_Config(
        LEFT_BUTTON_PIN,
        GPIO_FUNC(0UL) |
        GPIO_INPUT |
        GPIO_INPUTBUF_EN |
        GPIO_PULLUP);

    /*
     * 설정 중 불필요한 인터럽트가 들어오지 않도록
     * 외부 인터럽트 소스를 먼저 비활성화한다.
     */
    (void)GIC_IntSrcDis((uint32)RIGHT_BUTTON_INT_ID);
    (void)GIC_IntSrcDis((uint32)LEFT_BUTTON_INT_ID);

    /*
     * 인터럽트 콜백 등록
     *
     * BCC 문서에서는 EDGE_FALLING이 unused define으로 표시되므로,
     * 외부 인터럽트에서 지원되는 EDGE_BOTH를 사용하고
     * 콜백에서 GPIO가 LOW인 경우만 처리한다.
     */
    (void)GIC_IntVectSet(
        (uint32)RIGHT_BUTTON_INT_ID,
        (uint32)GIC_PRIORITY_NO_MEAN,
        (uint8)GIC_INT_TYPE_EDGE_BOTH,
        &EXTI_RightButtonHandler,
        NULL_PTR);

    (void)GIC_IntVectSet(
        (uint32)LEFT_BUTTON_INT_ID,
        (uint32)GIC_PRIORITY_NO_MEAN,
        (uint8)GIC_INT_TYPE_EDGE_BOTH,
        &EXTI_LeftButtonHandler,
        NULL_PTR);

    /*
     * 외부 인터럽트 라인과 실제 GPIO 연결
     *
     * 현재 SDK에 GPIO_IntExtSet()이 있으므로 해당 함수를 사용한다.
     * SDK 버전에 따라 같은 역할의 함수명이
     * GIC_IntExtSet()일 수도 있다.
     */
    (void)GPIO_IntExtSet(
        (uint32)RIGHT_BUTTON_INT_ID,
        RIGHT_BUTTON_PIN);

    (void)GPIO_IntExtSet(
        (uint32)LEFT_BUTTON_INT_ID,
        LEFT_BUTTON_PIN);

    /*
     * 이벤트 및 디바운스 상태 초기화
     */
    s_right_button_event = 0U;
    s_left_button_event  = 0U;
    s_debounce_elapsed_ms = BUTTON_DEBOUNCE_MS;

    /*
     * 외부 인터럽트 활성화
     */
    (void)GIC_IntSrcEn((uint32)RIGHT_BUTTON_INT_ID);
    (void)GIC_IntSrcEn((uint32)LEFT_BUTTON_INT_ID);
}


void EXTI_Button_Process(uint32 elapsed_ms)
{
    uint8 right_event;
    uint8 left_event;

    /*
     * 공통 디바운스 시간 갱신
     */
    if (s_debounce_elapsed_ms < BUTTON_DEBOUNCE_MS)
    {
        s_debounce_elapsed_ms += elapsed_ms;

        if (s_debounce_elapsed_ms > BUTTON_DEBOUNCE_MS)
        {
            s_debounce_elapsed_ms = BUTTON_DEBOUNCE_MS;
        }
    }

    /*
     * ISR과 공유하는 이벤트를 안전하게 복사하고 제거한다.
     *
     * 이벤트가 디바운스 시간 중에 발생했더라도 이번 호출에서
     * 제거되므로 기존 폴링 코드와 마찬가지로 나중에 지연 실행되지 않는다.
     */
    (void)SAL_CoreCriticalEnter();

    right_event = s_right_button_event;
    left_event  = s_left_button_event;

    s_right_button_event = 0U;
    s_left_button_event  = 0U;

    (void)SAL_CoreCriticalExit();

    /*
     * 기존 코드와 동일하게 오른쪽 버튼을 먼저 처리한다.
     */
    if ((right_event != 0U) &&
        (s_debounce_elapsed_ms >= BUTTON_DEBOUNCE_MS))
    {
        if (state == IDLE)
        {
            state = WAIT_STEER_RIGHT;
        }
        else
        {
            state = IDLE;
        }

        s_debounce_elapsed_ms = 0U;
    }

    /*
     * 오른쪽 버튼이 먼저 처리됐다면 debounce 시간이 0이 되므로
     * 같은 순간의 왼쪽 버튼 이벤트는 무시된다.
     * 기존의 공통 debounce 동작과 같다.
     */
    if ((left_event != 0U) &&
        (s_debounce_elapsed_ms >= BUTTON_DEBOUNCE_MS))
    {
        if (state == IDLE)
        {
            state = WAIT_STEER_LEFT;
        }
        else
        {
            state = IDLE;
        }

        s_debounce_elapsed_ms = 0U;
    }
}
