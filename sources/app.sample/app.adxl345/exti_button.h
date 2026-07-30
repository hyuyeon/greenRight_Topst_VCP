/*
 * exti_button.h
 *
 * Button handling interface.
 *
 * The first VCP-G port test reads the two GPIO inputs from the
 * 20 ms ADXL345 task and reproduces the original rising-edge and
 * 200 ms debounce behavior.
 */

#ifndef INC_EXTI_BUTTON_H_
#define INC_EXTI_BUTTON_H_

#include <stdint.h>

void EXTI_Button_Init(void);
void EXTI_Button_Process(uint32_t elapsed_ms);

#endif /* INC_EXTI_BUTTON_H_ */
