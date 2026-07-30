#ifndef SPEED_SENSOR_HEADER
#define SPEED_SENSOR_HEADER

#include <sal_internal.h>

extern volatile uint32 handler_call_cnt;

void  SpeedSensor_Init(void);
float Speed_GetMps(void);

#endif /* SPEED_SENSOR_HEADER */