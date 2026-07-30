#include "bno055_app.h"
#include "bno055.h"
#include "i2c1_bus.h"
#include <sal_internal.h>
#include <app_cfg.h>
#include <debug.h>

static volatile IMU_Data_t s_imuShared = {0, 0};

/* 이 태스크 내부에서만 heading을 갱신하므로, 쓰기는 여기 한 곳뿐 */
void BNO055_UpdateSharedData(int32_t heading, uint8_t valid)
{
    SAL_CoreCriticalEnter();
    s_imuShared.heading_x100 = heading;
    s_imuShared.valid = valid;
    SAL_CoreCriticalExit();
}

/* 다른 태스크가 호출하는 안전한 조회 함수 */
IMU_Data_t BNO055_GetSharedData(void)
{
    IMU_Data_t snapshot;

    SAL_CoreCriticalEnter();
    snapshot = s_imuShared;
    SAL_CoreCriticalExit();

    return snapshot;
}

