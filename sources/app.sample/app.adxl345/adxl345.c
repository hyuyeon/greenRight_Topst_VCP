/*
 * adxl345.c
 *
 * VCP-G port.
 * ADXL345 register values, XYZ conversion and Roll/Pitch equations
 * are kept from the original STM32 project.
 */

#include "adxl345.h"
#include "i2c1_bus.h"

#include <math.h>
#include <debug.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define ADXL345_ADDR_7BIT       0x53U

#define ADXL345_POWER_CTL       0x2DU
#define ADXL345_DATA_FORMAT     0x31U
#define ADXL345_DATAX0          0x32U

#define ADXL345_EXPECTED_DEVID  229U

static uint8_t ADXL345_WriteReg(uint8_t reg, uint8_t value)
{
    return I2C1_WriteReg(ADXL345_ADDR_7BIT,
                         reg,
                         value);
}

static uint8_t ADXL345_ReadReg(uint8_t reg, uint8_t *value)
{
    return I2C1_ReadReg(ADXL345_ADDR_7BIT,
                        reg,
                        value);
}

uint8_t ADXL345_Init(void)
{
    uint8_t dev_id = 0U;

    if (!ADXL345_ReadReg(ADXL345_DEVID, &dev_id))
    {
        mcu_printf("[ADXL345] DEVID read failed\n");
        return 0U;
    }

    mcu_printf("[ADXL345] DEVID=%d\n", (int)dev_id);

    if (dev_id != ADXL345_EXPECTED_DEVID)
    {
        mcu_printf("[ADXL345] invalid DEVID\n");
        return 0U;
    }

    if (!ADXL345_WriteReg(ADXL345_POWER_CTL, 0x08U))
    {
        mcu_printf("[ADXL345] POWER_CTL write failed\n");
        return 0U;
    }

    if (!ADXL345_WriteReg(ADXL345_DATA_FORMAT, 0x0BU))
    {
        mcu_printf("[ADXL345] DATA_FORMAT write failed\n");
        return 0U;
    }

    return 1U;
}

uint8_t ADXL345_ReadXYZ(int16_t *ax,
                        int16_t *ay,
                        int16_t *az)
{
    uint8_t buffer[6];

    if (!I2C1_ReadReg(ADXL345_ADDR_7BIT,
                      0x32U,
                      &buffer[0]))
    {
        mcu_printf("fail@0x32\n");
        return 0U;
    }

    if (!I2C1_ReadReg(ADXL345_ADDR_7BIT,
                      0x33U,
                      &buffer[1]))
    {
        mcu_printf("fail@0x33\n");
        return 0U;
    }

    if (!I2C1_ReadReg(ADXL345_ADDR_7BIT,
                      0x34U,
                      &buffer[2]))
    {
        mcu_printf("fail@0x34\n");
        return 0U;
    }

    if (!I2C1_ReadReg(ADXL345_ADDR_7BIT,
                      0x35U,
                      &buffer[3]))
    {
        mcu_printf("fail@0x35\n");
        return 0U;
    }

    if (!I2C1_ReadReg(ADXL345_ADDR_7BIT,
                      0x36U,
                      &buffer[4]))
    {
        mcu_printf("fail@0x36\n");
        return 0U;
    }

    if (!I2C1_ReadReg(ADXL345_ADDR_7BIT,
                      0x37U,
                      &buffer[5]))
    {
        mcu_printf("fail@0x37\n");
        return 0U;
    }

    *ax = (int16_t)(
        ((uint16_t)buffer[1] << 8U) |
        (uint16_t)buffer[0]);

    *ay = (int16_t)(
        ((uint16_t)buffer[3] << 8U) |
        (uint16_t)buffer[2]);

    *az = (int16_t)(
        ((uint16_t)buffer[5] << 8U) |
        (uint16_t)buffer[4]);

    return 1U;
}

void RollPitch_Calc(int16_t ax,
                    int16_t ay,
                    int16_t az,
                    float *roll,
                    float *pitch)
{
    *roll = atan2f((float)ay,
                   (float)az)
            * (180.0f / M_PI);

    *pitch = atan2f(
                 -(float)ax,
                 sqrtf(
                     ((float)ay * (float)ay) +
                     ((float)az * (float)az)))
             * (180.0f / M_PI);
}

uint8_t ADXL345_ReadReg_Test(uint8_t reg,
                            uint8_t *value)
{
    return I2C1_ReadReg(ADXL345_ADDR_7BIT,
                        reg,
                        value);
}