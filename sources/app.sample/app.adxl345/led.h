/*
 * led.h
 *
 * VCP-G left/right indicator LED interface.
 */

#ifndef INC_LED_H_
#define INC_LED_H_

void LED_Init(void);

void LED_RightOn(void);
void LED_RightOff(void);

void LED_LeftOn(void);
void LED_LeftOff(void);

void LED_AllOff(void);

#endif /* INC_LED_H_ */
