#ifndef INC_BNO055_APP_H_
#define INC_BNO055_APP_H_

#include <stdint.h>

void BNO055_AppCreate(void);

/* 다른 태스크(CAN 송신 등)에서 최신 heading을 안전하게 읽어가기 위한 인터페이스 */
typedef struct
{
    int32_t heading_x100;
    uint8_t valid;
} IMU_Data_t;

IMU_Data_t BNO055_GetSharedData(void);

void BNO055_UpdateSharedData(int32_t heading, uint8_t valid);
#endif