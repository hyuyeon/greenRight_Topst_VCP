#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>

/* Implemented by the RTOS time-synchronization module. */
uint64_t TimeSync_GetCurrentTimeMs(void);

#endif /* TIME_SYNC_H */
