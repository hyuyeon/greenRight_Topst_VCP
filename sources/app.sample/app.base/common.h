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
  uint16_t heading; //candidate vehicle heading (9 bit)
	uint64_t timestamp_ms; //source timestamp from the CAN header (12 bit)
	uint64_t received_timestamp; //timestamp when the message received
} CandidateVehicle;


typedef struct {
    uint8_t  type; // msgId 0110의 tl_type_mask 원본값: 0=신호등 없음, 0x80=MQTT 통신 에러
    uint8_t  color; 
    uint8_t  time_left; //no time_left could be possible
    //(even if candidate vehicle is not in the conflict zone, 
    //he traffic light may be in the conflict zone)
    uint16_t cz_x;  //자차가 회전하려는 목표 지점의 x
    uint16_t cz_y;  //자차가 회전하려는 목표 지점의 y
    uint16_t timestamp; //source timestamp from the CAN header (12 bit)
    uint64_t received_timestamp; //local tick when the message was received
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


#define DICISION_QUEUE_LEN 1

extern EgoVehicle ego;
extern CandidateVehicle candidateVehicle;
extern TrafficLight tl;
extern uint8_t maneuver;
extern volatile uint8_t pedFlag;

#endif /* COMMON_H */ 