/*
 * adxl345.h
 *
 * VCP-G port of the original STM32 ADXL345 interface.
 */

#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_

#include <stdint.h>

#define ADXL345_DEVID  0x00U

uint8_t ADXL345_Init(void);
uint8_t ADXL345_ReadXYZ(int16_t *ax, int16_t *ay, int16_t *az);
uint8_t ADXL345_ReadReg_Test(uint8_t reg, uint8_t *value);  // 테스트용, 나중에 제거
void RollPitch_Calc(int16_t ax, int16_t ay, int16_t az,
                    float *roll, float *pitch);

#endif /* INC_ADXL345_H_ */
