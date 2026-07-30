#include "i2c1_bus.h"
#include <i2c.h>      /* dev.drivers/i2c/i2c.h */
#include <i2c_reg.h>


#define BNO055_I2C_CH      I2C_CH_1
#define BNO055_I2C_PORT    2U
#define BNO055_I2C_SPEED   100U   

static uint8_t s_i2c1_inited = 0U;

void I2C1_Bus_Init(void)
{
    if (s_i2c1_inited == 0U)
    {
        
        (void)I2C_Open(BNO055_I2C_CH, BNO055_I2C_PORT, BNO055_I2C_SPEED, NULL_PTR, NULL_PTR);
        s_i2c1_inited = 1U;
    }
}

uint8_t I2C1_WriteReg(uint8_t addr7, uint8_t reg, uint8_t value)
{
    I2CXfer_t xfer = {0};
    uint8_t   cmd  = reg;
    uint8_t   data = value;

    xfer.xCmdBuf = &cmd;
    xfer.xCmdLen = 1U;
    xfer.xOutBuf = &data;
    xfer.xOutLen = 1U;

    /* 7비트 슬레이브 주소를 1비트 왼쪽 시프트 (bit0가 R/W 플래그 자리) */
    return (I2C_Xfer(BNO055_I2C_CH, (uint8_t)(addr7 << 1U), xfer, 0U) == SAL_RET_SUCCESS) ? 1U : 0U;
}

uint8_t I2C1_ReadReg(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    I2CXfer_t xfer = {0};
    uint8_t   cmd  = reg;

    xfer.xCmdBuf = &cmd;
    xfer.xCmdLen = 1U;
    xfer.xInBuf  = value;
    xfer.xInLen  = 1U;

    return (I2C_Xfer(BNO055_I2C_CH, (uint8_t)(addr7 << 1U), xfer, 0U) == SAL_RET_SUCCESS) ? 1U : 0U;
}