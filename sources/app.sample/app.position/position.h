#ifndef POSITION_H
#define POSITION_H

#include <sal_internal.h>

void Position_Reset(void);

void Position_Update(
    float current_speed_mps,
    int32 heading_x100);

int32 Position_GetXcm(void);
int32 Position_GetYcm(void);
int32 Position_GetTotalDistanceCm(void);
#endif