#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>

typedef enum {
    SIG_RED = 0,
    SIG_YELLOW,
    SIG_GREEN,
    SIG_COUNT
} SignalColor;

typedef struct {
    uint16_t x;              // 10 bit (0~1023)
    uint16_t y;              // 11 bit (0~2047)
    uint8_t speed;           // 8 bit
    uint16_t heading;        // 9 bit (0~511)
    uint16_t timestamp;      // 12bit
    uint8_t turn_signal;
} EgoVehicle;

typedef struct {
	uint8_t type; //if its 0, no candidate vehicle.
	//1: ego vehicle right turn vs opposite straight
  //2: ego vehicle right turn vs opposite left turn[protected]
  //4: ego vehicle left turn[unprotected] vs opposite straight
  //8: ego vehicle left turn[unprotected] vs opposite right turn
	uint16_t cz_x; //conflict zone x coordinate
  uint16_t cz_y; //conflict zone y coordinate
  uint16_t x; //candidate vehicle x coordinate
  uint16_t y; //candidate vehicle y coordinate
  uint8_t speed; //candidate vehicle speed
	uint64_t timestamp_ms; //current timestamp in milliseconds on frame
	uint64_t received_timestamp; //timestamp when the message received
} CandidateVehicle;


typedef struct {
    uint8_t  type; // msgId 0110의 tl_type_mask 원본값: 0=신호등 없음, 0x80=MQTT 통신 에러
    uint8_t  color; //if 255, no traffic.
    uint8_t  time_left; //no time_left could be possible
    //(even if candidate vehicle is not in the conflict zone, 
    //he traffic light may be in the conflict zone)
    uint16_t cz_x;  //conflict zone x coordinate where the traffic light is located
    uint16_t cz_y;  //conflict zone y coordinate where the traffic light is located
} TrafficLight;


typedef struct{
  uint8_t turnState; //0: straight, 1: right turn, 2: left turn,255: mqtt comm fail
  //on right turn situation
	uint8_t pedestrianFlag; //0: no pedestrian, 1: pedestrian, 2 : AI error
  //flags : 0 = false, 1 = true
  //on right turn situation
	uint8_t LStraightFlag; //is left side straight vehicle exist?
	uint8_t OppLeftFlag;   //is opposite left turn vehicle exist?
  //on left turn situation
	uint8_t tlWarningFlag; //is there enough time to cross the intersection?
	uint8_t OppStraightFlag; //is opposite straight vehicle exist?
  uint8_t OppRightFlag;    //is opposite right turn vehicle exist?
} Dicision;

typedef struct
{
    uint8_t msg_id;      // 4bit
    uint16_t timestamp;	 // 12bit
} CAN_Header_t;

// uint8_t maneuver; //0: straight, 1: right turn, 2: unprotected left turn, 3: protected left turn
// uint8_t pedFlag; ///* 00 None / 01 (As Decimal, 1) Exists / 10(As Decimal, 2) AI Error 
// EgoVehicle ego;
// CandidateVehicle candidateVehicle;
// TrafficLight tl; //Displayed on LCD and used for decision making
// QueueHandle_t dicisionQueue; //for task synchronization, task synch message queue
// SemaphoreHandle_t turnJudgeSem; //judgement task wake up semaphore
// SemaphoreHandle_t tlDisplaySem; //tlDisplay task wake up semaphore
// QueueHandle_t buzzerQueue; //buzzer task wake up queue

/* maneuver value definitions*/
#define MANEUVER_STRAIGHT          0U
#define MANEUVER_RIGHT_TURN        1U
#define MANEUVER_LEFT_TURN_UNPROT  2U
#define MANEUVER_LEFT_TURN_PROT    3U

/* candidateVehicle.type */
#define CAND_NONE                  0x00U  /* no candidate vehicle */
#define CAND_COMM_ERROR            0x80U  /* MQTT communication error: candidate unavailable */

#define CAND_RT_LEFT_STRAIGHT      1U  /* right turn vs left-side straight vehicle */
#define CAND_RT_OPP_LEFT           2U  /* right turn vs opposite protected/general left-turn vehicle */

#define CAND_LT_OPP_STRAIGHT       4U  /* unprotected left turn vs opposite straight vehicle */
#define CAND_LT_OPP_RIGHT          8U  /* unprotected left turn vs opposite right-turn vehicle */

#define TL_NONE                    0U     /* no referenced traffic light */
#define TL_COMM_ERROR              0x80U  /* MQTT communication error: traffic light unavailable */

/* dicision queue??湲몄씠 overwrite 諛⑹떇?대씪 1*/
#define DICISION_QUEUE_LEN 1

extern EgoVehicle ego;

#endif /* COMMON_H */