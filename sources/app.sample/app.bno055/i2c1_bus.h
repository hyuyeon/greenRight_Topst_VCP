#ifndef INC_I2C1_BUS_H_
#define INC_I2C1_BUS_H_

#include <stdint.h>

void    I2C1_Bus_Init(void);
uint8_t I2C1_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value);
uint8_t I2C1_ReadReg(uint8_t addr7, uint8_t reg, uint8_t *value);

#endif