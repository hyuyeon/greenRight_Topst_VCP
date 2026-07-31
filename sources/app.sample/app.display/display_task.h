#ifndef APP_DISPLAY_TASK_H
#define APP_DISPLAY_TASK_H

#include <stdint.h>

#include "common.h"

/*
 * 기존 TurnJudgeTask와의 호환을 위해 전역 TrafficLight 상태를 유지한다.
 * CAN 수신부에서는 직접 수정하기보다 Display_TrafficLightUpdate() 사용 권장.
 */
extern TrafficLight tl;

/* 디스플레이 동기화 객체와 태스크 생성 */
void Display_AppCreate(void);

/* CAN 수신부에서 신호등 데이터 수신 후 호출 */
void Display_TrafficLightUpdate(const TrafficLight *newTl);

/* TurnJudgeTask에서 판단 완료 후 호출 */
uint8_t Display_DicisionPost(const Dicision *newDecision);

/* 다른 태스크에서 TrafficLight 전체 값을 일관되게 가져올 때 사용 */
void Display_GetTrafficLightSnapshot(TrafficLight *outTl);

#endif /* APP_DISPLAY_TASK_H */