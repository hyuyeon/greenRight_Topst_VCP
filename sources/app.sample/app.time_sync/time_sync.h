#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <stdint.h>


/* =========================================================
 * CAN Time Sync Protocol
 * ========================================================= */

/*
 * Time Sync Message ID
 *
 * 0x2 = 0010b
 */
#define TIME_SYNC_MSG_ID                 (0x02U)


/*
 * D3-G NTP Sync Status
 */
#define TIME_SYNC_STATUS_OK              (0x00U)
#define TIME_SYNC_STATUS_UNSYNCED        (0x01U)
#define TIME_SYNC_STATUS_ERROR           (0x80U)


/* =========================================================
 * Public API
 * ========================================================= */

/*
 * Time Sync App / Task 생성
 */
void TimeSync_AppCreate(void);


/*
 * D3-G에서 수신한 Sync Frame의 시간정보를 전달한다.
 *
 * ullSyncEpochMs :
 *      프로젝트 Epoch 기준 현재 시각(ms)
 *
 * ucSyncStatus :
 *      D3-G NTP 동기화 상태
 *
 * return:
 *      TRUE  : 정상 처리
 *      FALSE : 잘못된 상태 또는 Tick 획득 실패
 */
uint8_t TimeSync_OnSyncFrame(
    uint64_t ullSyncEpochMs,
    uint8_t ucSyncStatus);


/*
 * 현재 VCP-G가 기준시각을 가지고 있는지 확인
 */
uint8_t TimeSync_IsSynced(void);


/*
 * 현재 VCP-G 논리시각 반환
 *
 * logical time
 * = SAL monotonic tick + synchronized offset
 *
 * return:
 *      TRUE  : 정상
 *      FALSE : 아직 동기화되지 않음
 */
uint8_t TimeSync_GetCurrentMs(
    uint64_t *pullCurrentMs);


/*
 * CAN Header용 하위 12bit Timestamp 반환
 *
 * timestamp = current_time_ms & 0x0FFF
 */
uint8_t TimeSync_GetTimestamp12(
    uint16_t *pusTimestamp12);


/*
 * 마지막 D3-G ↔ VCP-G 시각 오차
 *
 * +값 : VCP 시계가 느림
 * -값 : VCP 시계가 빠름
 */
int64_t TimeSync_GetLastDiffMs(void);

uint64 TimeSync_GetCurrentTimeMs
(
    void
);

uint8 TimeSync_IsValid
(
    void
);
void TimeSync_PrintCurrentTime
(
    void
);
#endif /* TIME_SYNC_H */