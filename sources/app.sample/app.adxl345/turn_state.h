#ifndef TURN_STATE_H
#define TURN_STATE_H

typedef enum
{
    IDLE = 0,
    WAIT_STEER_RIGHT,
    WAIT_RETURN_RIGHT,
    WAIT_STEER_LEFT,
    WAIT_RETURN_LEFT
} State_t;

/*
 * 실제 변수는 sensor_app.c에서 한 번만 정의한다.
 * 다른 파일에서는 이 선언을 통해 같은 변수를 사용한다.
 */
extern State_t state;

#endif /* TURN_STATE_H */