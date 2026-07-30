/*
 * led.c
 *
 * J18D100:
 *   GPIO_B24 = right LED
 *   GPIO_B23 = left LED
 *
 * LED wiring:
 *   GPIO -> current limiting resistor -> LED -> GND
 *   GPIO HIGH turns the LED on.
 */

#include "led.h"

#include <gpio.h>

#define RIGHT_LED_PIN  GPIO_GPB(24UL)
#define LEFT_LED_PIN   GPIO_GPB(23UL)

void LED_Init(void)
{
    (void)GPIO_Config(RIGHT_LED_PIN,
                      GPIO_FUNC(0UL) | GPIO_OUTPUT);

    (void)GPIO_Config(LEFT_LED_PIN,
                      GPIO_FUNC(0UL) | GPIO_OUTPUT);

    LED_AllOff();
}

void LED_RightOn(void)
{
    (void)GPIO_Set(RIGHT_LED_PIN, 1UL);
}

void LED_RightOff(void)
{
    (void)GPIO_Set(RIGHT_LED_PIN, 0UL);
}

void LED_LeftOn(void)
{
    (void)GPIO_Set(LEFT_LED_PIN, 1UL);
}

void LED_LeftOff(void)
{
    (void)GPIO_Set(LEFT_LED_PIN, 0UL);
}

void LED_AllOff(void)
{
    LED_RightOff();
    LED_LeftOff();
}
