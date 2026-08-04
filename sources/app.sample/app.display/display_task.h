#ifndef APP_DISPLAY_TASK_H
#define APP_DISPLAY_TASK_H

#include <stdint.h>

#include "common.h"



/* 디스플레이 동기화 객체와 태스크 생성 */
void Display_AppCreate(void);

/*
 * CAN 수신부는 tl을 갱신한 후,
 * 표시할 값이 변경되면 Display_TrafficLightNotify()를 호출한다.
 */
/* CAN 수신부에서 신호등 데이터 수신 후 호출 */
void Display_TrafficLightNotify(void);

/* TurnJudgeTask에서 판단 완료 후 호출 */
uint8_t Display_DicisionPost(const Dicision *newDecision);

/* 다른 태스크에서 TrafficLight 전체 값을 일관되게 가져올 때 사용 */
void Display_GetTrafficLightSnapshot(TrafficLight *outTl);

#endif /* APP_DISPLAY_TASK_H */
