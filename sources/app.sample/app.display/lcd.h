#ifndef __LCD_H__
#define __LCD_H__

#include <stdint.h>
#include "common.h"
// RGB565 주요 색상 정의
#define COLOR_BLACK       0x0000
#define COLOR_WHITE       0xFFFF
#define COLOR_RED         0xF800
#define COLOR_GREEN       0x07E0
#define COLOR_BLUE        0x001F
#define COLOR_YELLOW      0xFFE0
#define COLOR_ORANGE      0xFD20
#define COLOR_DARKGRAY    0x39E7

// ST7735S 디스플레이 해상도 (128x160 기본, 패널 네이티브 기준)
#define ST7735_WIDTH      128
#define ST7735_HEIGHT     160

// 대시보드 UI는 랜드스케이프(가로 160 x 세로 128) 기준으로 설계되어 있음.
// MADCTL(0x36)에 MV 비트를 세팅해 회전시킨 뒤 실제 주소 지정에 사용하는
// 논리 해상도. lcd_RT.c(Nucleo)의 LCD_W/LCD_H와 동일한 의미.
#define LCD_W             ST7735_HEIGHT
#define LCD_H             ST7735_WIDTH

// 대시보드 UI 전용 색상 (RGB565)
#define COLOR_DIVIDER     0x39C7
#define COLOR_AMBER       0xFD20
#define COLOR_ICON        0xC618
#define COLOR_DIM_RED     0x4000
#define COLOR_DIM_AMBER   0x4200
#define COLOR_DIM_GREEN   0x0320


typedef enum {
    DIR_STRAIGHT = 0,
    DIR_RIGHT = 1,
    DIR_LEFT = 2,
    DIR_ERROR = 255
} TurnDirection;

#define WARN_NONE              0x00
#define WARN_PEDESTRIAN        0x01
#define WARN_STRAIGHT_VEHICLE  0x02
#define WARN_OPPOSITE_TURN     0x04
#define WARN_TL                0x08
#define WARN_COMM_ERROR        0x10

// 주요 API 함수 선언
void ST7735S_Init(void);
void ST7735S_WriteCommand(uint8_t cmd);
void ST7735S_WriteData(uint8_t data);
void ST7735S_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ST7735S_FillScreen(uint16_t color);
void ST7735S_DrawPixel(uint8_t x, uint8_t y, uint16_t color);
void ST7735S_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);

// 대시보드 그리기 UI API (nucleo lcd_RT.c와 동일한 레이아웃/시그니처)
void Dashboard_DrawStatic(TurnDirection dir, uint8_t warnMask, SignalColor sig, uint8_t countdown, uint8_t pedFlag);
void Dashboard_DrawSignalDots(SignalColor active);
void Dashboard_UpdateCountdown(uint8_t sec);
void Dashboard_DrawDirection(TurnDirection dir, uint8_t dataStatus);
void Dashboard_DrawWarnings(TurnDirection dir, uint8_t mask, uint8_t pedFlag);

// 디버깅용: SPI 전송 성공/실패 누적 통계를 UART로 출력
void LCD_PrintXferStats(void);

#endif // __LCD_H__