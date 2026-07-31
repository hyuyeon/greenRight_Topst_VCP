#include <stddef.h>

#include <sal_api.h>

#include "../../dev.drivers/gpio/gpio.h"
#include "../../dev.drivers/gpsb/gpsb.h"
#include "lcd.h"

#define LCD_DC_PIN             GPIO_GPB(1UL)
#define LCD_RST_PIN            GPIO_GPA(13UL)
#define LCD_CS_PIN             GPIO_GPB(5UL)
#define LCD_BLK_PIN            GPIO_GPA(10UL)

#define LCD_GPSB_CH            0UL
#define LCD_SPI_SPEED_HZ       (5UL * 1000UL * 1000UL)
#define LCD_XFER_MODE          (GPSB_XFER_MODE_PIO | GPSB_XFER_MODE_WITHOUT_INTERRUPT)
static uint8_t g_rxDummy[2];

static void LCD_Write(uint8_t dc, const uint8_t *data, uint32_t size)
{
    (void)GPIO_Set(LCD_DC_PIN, dc);
    (void)GPSB_CsActivate(LCD_GPSB_CH, LCD_CS_PIN, FALSE);
    (void)GPSB_Xfer(LCD_GPSB_CH, data, NULL, size, LCD_XFER_MODE);
    (void)GPSB_CsDeactivate(LCD_GPSB_CH, LCD_CS_PIN, FALSE);
}

void ST7735S_WriteCommand(uint8_t cmd)
{
    LCD_Write(0UL, &cmd, 1UL);
}

void ST7735S_WriteData(uint8_t data)
{
    LCD_Write(1UL, &data, 1UL);
}

void ST7735S_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    ST7735S_WriteCommand(0x2A);
    ST7735S_WriteData(0x00);
    ST7735S_WriteData(x0);
    ST7735S_WriteData(0x00);
    ST7735S_WriteData(x1);

    ST7735S_WriteCommand(0x2B);
    ST7735S_WriteData(0x00);
    ST7735S_WriteData(y0);
    ST7735S_WriteData(0x00);
    ST7735S_WriteData(y1);

    ST7735S_WriteCommand(0x2C);
}

void ST7735S_DrawPixel(uint8_t x, uint8_t y, uint16_t color)
{
    uint8_t data[2];

    if ((x >= LCD_W) || (y >= LCD_H))
    {
        return;
    }

    data[0] = (uint8_t)(color >> 8);
    data[1] = (uint8_t)color;

    ST7735S_SetAddressWindow(x, y, x, y);
    LCD_Write(1UL, data, sizeof(data));
}

void ST7735S_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color)
{
    uint8_t colorData[2];
    uint32_t pixelCount;
    uint32_t i;

    if ((w == 0U) || (h == 0U) || (x >= LCD_W) || (y >= LCD_H))
    {
        return;
    }

    if (((uint16_t)x + w) > LCD_W)
    {
        w = (uint8_t)(LCD_W - x);
    }

    if (((uint16_t)y + h) > LCD_H)
    {
        h = (uint8_t)(LCD_H - y);
    }

    colorData[0] = (uint8_t)(color >> 8);
    colorData[1] = (uint8_t)color;

    ST7735S_SetAddressWindow(x, y,
                             (uint8_t)(x + w - 1U),
                             (uint8_t)(y + h - 1U));

    (void)GPIO_Set(LCD_DC_PIN, 1UL);
    (void)GPSB_CsActivate(LCD_GPSB_CH, LCD_CS_PIN, FALSE);

    pixelCount = (uint32_t)w * (uint32_t)h;

    for (i = 0UL; i < pixelCount; i++)
    {
        (void)GPSB_Xfer(LCD_GPSB_CH, colorData, g_rxDummy,
                        sizeof(colorData), LCD_XFER_MODE);
    }

    (void)GPSB_CsDeactivate(LCD_GPSB_CH, LCD_CS_PIN, FALSE);
}

void ST7735S_FillScreen(uint16_t color)
{
    ST7735S_FillRect(0U, 0U, (uint8_t)LCD_W, (uint8_t)LCD_H, color);
}

void ST7735S_Init(void)
{
    GPSBOpenParam_t openParam = {0};
    static const uint8_t gammaPositive[16] = {
        0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D,
        0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10
    };
    static const uint8_t gammaNegative[16] = {
        0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
        0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10
    };

    openParam.uiSdo = GPIO_GPB(6UL);
    openParam.uiSdi = GPIO_GPB(7UL);
    openParam.uiSclk = GPIO_GPB(4UL);
    openParam.uiIsSlave = GPSB_MASTER_MODE;

    if (GPSB_Open(LCD_GPSB_CH, openParam) != SAL_RET_SUCCESS)
    {
        return;
    }

    if (GPSB_SetSpeed(LCD_GPSB_CH, LCD_SPI_SPEED_HZ) != SAL_RET_SUCCESS)
    {
        return;
    }

    if (GPSB_SetBpw(LCD_GPSB_CH, 8U) != SAL_RET_SUCCESS)
    {
        return;
    }

    (void)GPIO_Config(LCD_DC_PIN, GPIO_FUNC(0UL) | GPIO_OUTPUT);
    (void)GPIO_Config(LCD_RST_PIN, GPIO_FUNC(0UL) | GPIO_OUTPUT);
    (void)GPIO_Config(LCD_BLK_PIN, GPIO_FUNC(0UL) | GPIO_OUTPUT);
    (void)GPSB_CsInit(LCD_GPSB_CH, LCD_CS_PIN, FALSE);

    (void)GPIO_Set(LCD_BLK_PIN, 1UL);

    (void)GPIO_Set(LCD_RST_PIN, 1UL);
    (void)SAL_TaskSleep(10UL);
    (void)GPIO_Set(LCD_RST_PIN, 0UL);
    (void)SAL_TaskSleep(50UL);
    (void)GPIO_Set(LCD_RST_PIN, 1UL);
    (void)SAL_TaskSleep(120UL);

    ST7735S_WriteCommand(0x01);
    (void)SAL_TaskSleep(150UL);

    ST7735S_WriteCommand(0x11);
    (void)SAL_TaskSleep(120UL);

    ST7735S_WriteCommand(0xB1);
    ST7735S_WriteData(0x01);
    ST7735S_WriteData(0x2C);
    ST7735S_WriteData(0x2D);

    ST7735S_WriteCommand(0xB2);
    ST7735S_WriteData(0x01);
    ST7735S_WriteData(0x2C);
    ST7735S_WriteData(0x2D);

    ST7735S_WriteCommand(0xB3);
    ST7735S_WriteData(0x01);
    ST7735S_WriteData(0x2C);
    ST7735S_WriteData(0x2D);
    ST7735S_WriteData(0x01);
    ST7735S_WriteData(0x2C);
    ST7735S_WriteData(0x2D);

    ST7735S_WriteCommand(0xB4);
    ST7735S_WriteData(0x07);

    ST7735S_WriteCommand(0xC0);
    ST7735S_WriteData(0xA2);
    ST7735S_WriteData(0x02);
    ST7735S_WriteData(0x84);

    ST7735S_WriteCommand(0xC1);
    ST7735S_WriteData(0xC5);

    ST7735S_WriteCommand(0xC2);
    ST7735S_WriteData(0x0A);
    ST7735S_WriteData(0x00);

    ST7735S_WriteCommand(0xC3);
    ST7735S_WriteData(0x8A);
    ST7735S_WriteData(0x2A);

    ST7735S_WriteCommand(0xC4);
    ST7735S_WriteData(0x8A);
    ST7735S_WriteData(0xEE);

    ST7735S_WriteCommand(0xC5);
    ST7735S_WriteData(0x0E);

    ST7735S_WriteCommand(0x20);

    /* 대시보드 UI는 가로(160x128) 레이아웃으로 설계되어 있어 MV 비트로 회전.
     * MV(0x20) | MX(0x40) = 0x60 (RGB 순서, lcd_RT.c와 동일).
     * RED가 파랑, YELLOW가 시안으로 나오면 R/B가 뒤바뀐 것이므로
     * BGR 비트(0x08)를 빼야 함 (패널 배선에 따라 실측 필요). */
    ST7735S_WriteCommand(0x36);
    ST7735S_WriteData(0x60);

    ST7735S_WriteCommand(0x3A);
    ST7735S_WriteData(0x05);

    ST7735S_WriteCommand(0xE0);
    LCD_Write(1UL, gammaPositive, sizeof(gammaPositive));

    ST7735S_WriteCommand(0xE1);
    LCD_Write(1UL, gammaNegative, sizeof(gammaNegative));

    ST7735S_WriteCommand(0x13);
    (void)SAL_TaskSleep(10UL);

    ST7735S_WriteCommand(0x29);
    (void)SAL_TaskSleep(100UL);

}

/* ===========================================================================
 * 아래부터는 lcd_RT.c(Nucleo)에서 검증된 도형/폰트/아이콘/대시보드 레이어를
 * TOPST VCP-G 저수준(GPSB/GPIO 기반 ST7735S_FillRect/DrawPixel) 위로 그대로
 * 포팅한 코드입니다. 하드웨어 SPI 전송 방식만 다르고, 좌표/로직은 동일합니다.
 * ========================================================================= */

static int16_t abs16(int16_t v)
{
    int16_t result;

    if (v < 0)
    {
        result = (int16_t)(-v);
    }
    else
    {
        result = v;
    }

    return result;
}

static void swap_i16(int16_t *a, int16_t *b)
{
    int16_t t;

    t = *a;
    *a = *b;
    *b = t;
}

/* 브레젠험 직선 (보행자 아이콘의 팔다리처럼 각진 선을 그릴 때 사용) */
static void ST7735S_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t dx;
    int16_t sx;
    int16_t dy;
    int16_t sy;
    int16_t err;
    int16_t e2;

    dx = abs16((int16_t)(x1 - x0));
    sx = (x0 < x1) ? 1 : -1;
    dy = (int16_t)(-abs16((int16_t)(y1 - y0)));
    sy = (y0 < y1) ? 1 : -1;
    err = (int16_t)(dx + dy);

    for (;;)
    {
        ST7735S_DrawPixel((uint8_t)x0, (uint8_t)y0, color);

        if ((x0 == x1) && (y0 == y1))
        {
            break;
        }

        e2 = (int16_t)(2 * err);

        if (e2 >= dy)
        {
            err = (int16_t)(err + dy);
            x0 = (int16_t)(x0 + sx);
        }

        if (e2 <= dx)
        {
            err = (int16_t)(err + dx);
            y0 = (int16_t)(y0 + sy);
        }
    }
}

/* 채워진 원 (신호등 점, 사람 머리 등) - 미드포인트 원 알고리즘을
 * 가로 스팬(FillRect) 단위로 채워서 그림. 반지름이 작을 때 DrawCircle(윤곽선)로
 * 그리면 점이 몇 개 안 찍혀서 다이아몬드처럼 보이므로 항상 FillCircle 사용. */
static void ST7735S_FillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t x;
    int16_t y;
    int16_t err;

    x = r;
    y = 0;
    err = 0;

    while (x >= y)
    {
        ST7735S_FillRect((uint8_t)(x0 - x), (uint8_t)(y0 + y), (uint8_t)(2 * x + 1), 1U, color);
        ST7735S_FillRect((uint8_t)(x0 - x), (uint8_t)(y0 - y), (uint8_t)(2 * x + 1), 1U, color);
        ST7735S_FillRect((uint8_t)(x0 - y), (uint8_t)(y0 + x), (uint8_t)(2 * y + 1), 1U, color);
        ST7735S_FillRect((uint8_t)(x0 - y), (uint8_t)(y0 - x), (uint8_t)(2 * y + 1), 1U, color);

        y++;

        if (err <= 0)
        {
            err += (int16_t)(2 * y + 1);
        }

        if (err > 0)
        {
            x--;
            err -= (int16_t)(2 * x + 1);
        }
    }
}

/* 화살촉(삼각형) 채우기 - 표준 스캔라인 방식: y를 위에서 아래로 훑으면서
 * 그 y줄에서 삼각형의 왼쪽/오른쪽 경계 x를 계산해 한 줄씩 FillRect로 채움 */
static void ST7735S_FillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                                  int16_t x2, int16_t y2, uint16_t color)
{
    int16_t a;
    int16_t b;
    int16_t y;
    int16_t last;
    int16_t dx01;
    int16_t dy01;
    int16_t dx02;
    int16_t dy02;
    int16_t dx12;
    int16_t dy12;
    int32_t sa;
    int32_t sb;

    if (y0 > y1) { swap_i16(&y0, &y1); swap_i16(&x0, &x1); }
    if (y1 > y2) { swap_i16(&y1, &y2); swap_i16(&x1, &x2); }
    if (y0 > y1) { swap_i16(&y0, &y1); swap_i16(&x0, &x1); }

    if (y0 == y2)
    {
        a = x0;
        b = x0;

        if (x1 < a) { a = x1; } else if (x1 > b) { b = x1; }
        if (x2 < a) { a = x2; } else if (x2 > b) { b = x2; }

        ST7735S_FillRect((uint8_t)a, (uint8_t)y0, (uint8_t)(b - a + 1), 1U, color);
        return;
    }

    dx01 = (int16_t)(x1 - x0);
    dy01 = (int16_t)(y1 - y0);
    dx02 = (int16_t)(x2 - x0);
    dy02 = (int16_t)(y2 - y0);
    dx12 = (int16_t)(x2 - x1);
    dy12 = (int16_t)(y2 - y1);
    sa = 0;
    sb = 0;

    last = (y1 == y2) ? y1 : (int16_t)(y1 - 1);

    for (y = y0; y <= last; y++)
    {
        a = (int16_t)(x0 + sa / dy01);
        b = (int16_t)(x0 + sb / dy02);
        sa += dx01;
        sb += dx02;

        if (a > b) { swap_i16(&a, &b); }

        ST7735S_FillRect((uint8_t)a, (uint8_t)y, (uint8_t)(b - a + 1), 1U, color);
    }

    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);

    for (; y <= y2; y++)
    {
        a = (int16_t)(x1 + sa / dy12);
        b = (int16_t)(x0 + sb / dy02);
        sa += dx12;
        sb += dx02;

        if (a > b) { swap_i16(&a, &b); }

        ST7735S_FillRect((uint8_t)a, (uint8_t)y, (uint8_t)(b - a + 1), 1U, color);
    }
}

/* ------------------------------- 5x7 폰트 --------------------------------
 * 실제 사용하는 문구(RIGHT TURN / LEFT TURN / STRAIGHT / CAUTION /
 * OPP. TURN / PEDESTRIAN / AI N/A / SEC / 0-9)에 필요한 문자만 최소 구성.
 * 각 문자는 5개의 컬럼(byte)로 구성, bit0=윗줄 ~ bit6=아랫줄(7행). */
typedef struct { char ch; uint8_t col[5]; } FontGlyph;

static const FontGlyph font_table[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'.', {0x00,0x00,0x60,0x60,0x00}},
    {'/', {0x40,0x20,0x10,0x08,0x04}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x62,0x51,0x49,0x49,0x46}},
    {'3', {0x22,0x41,0x49,0x49,0x36}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}},
};
#define FONT_TABLE_LEN (sizeof(font_table) / sizeof(font_table[0]))

static const FontGlyph *Font_Find(char c)
{
    uint8_t i;
    const FontGlyph *result;

    result = &font_table[0]; /* 미지원 문자는 공백 처리 */

    for (i = 0U; i < FONT_TABLE_LEN; i++)
    {
        if (font_table[i].ch == c)
        {
            result = &font_table[i];
            break;
        }
    }

    return result;
}

/* size: 1이면 5x7 픽셀 그대로, 2면 픽셀 하나를 2x2 블록으로 확대(카운트다운
 * 숫자처럼 크게 보여주고 싶을 때 사용) */
static void ST7735S_DrawChar(int16_t x, int16_t y, char c, uint16_t color, uint8_t size)
{
    const FontGlyph *g;
    uint8_t col;
    uint8_t row;
    uint8_t line;

    g = Font_Find(c);

    for (col = 0U; col < 5U; col++)
    {
        line = g->col[col];

        for (row = 0U; row < 7U; row++)
        {
            if ((line & (uint8_t)(1U << row)) != 0U)
            {
                ST7735S_FillRect((uint8_t)(x + (col * size)), (uint8_t)(y + (row * size)),
                                 size, size, color);
            }
        }
    }
}

static void ST7735S_DrawString(int16_t x, int16_t y, const char *s, uint16_t color, uint8_t size)
{
    int16_t cx;

    cx = x;

    while (*s != '\0')
    {
        ST7735S_DrawChar(cx, y, *s, color, size);
        cx = (int16_t)(cx + ((5 + 1) * size)); /* 글자 폭 5 + 자간 1 */
        s++;
    }
}

/* --------------------------------- 아이콘 --------------------------------
 * 전부 위의 도형 프리미티브(FillRect/FillCircle/DrawLine/FillTriangle) 조합으로만
 * 그려서, 이미지 리소스나 별도 비트맵 없이 컴파일 타임에 고정된 벡터 아이콘처럼 동작. */

static void Icon_Car(int16_t cx, int16_t cy, uint16_t color)
{
    ST7735S_FillRect((uint8_t)(cx - 8), (uint8_t)(cy - 2), 16U, 7U, color); /* 차체 */
    ST7735S_FillRect((uint8_t)(cx - 5), (uint8_t)(cy - 7), 10U, 5U, color); /* 지붕 */
}

static void Icon_Pedestrian(int16_t cx, int16_t cy, uint16_t color)
{
    ST7735S_FillCircle(cx, (int16_t)(cy - 8), 3, color);                                       /* 머리 */
    ST7735S_DrawLine(cx, (int16_t)(cy - 5), cx, (int16_t)(cy + 3), color);                      /* 몸통 */
    ST7735S_DrawLine(cx, (int16_t)(cy + 3), (int16_t)(cx - 4), (int16_t)(cy + 8), color);        /* 왼다리 */
    ST7735S_DrawLine(cx, (int16_t)(cy + 3), (int16_t)(cx + 4), (int16_t)(cy + 8), color);        /* 오른다리 */
    ST7735S_DrawLine(cx, (int16_t)(cy - 3), (int16_t)(cx - 4), cy, color);                       /* 왼팔 */
    ST7735S_DrawLine(cx, (int16_t)(cy - 3), (int16_t)(cx + 4), cy, color);                       /* 오른팔 */
}

static void Icon_TrafficLight(int16_t cx, int16_t cy, uint16_t bodyColor, uint16_t alertColor)
{
    /* 몸체 (각진 사각형, 세로로 긴 형태) */
    ST7735S_FillRect((uint8_t)(cx - 4), (uint8_t)(cy - 9), 8U, 18U, bodyColor);

    /* 점 3개: 위(강조/빨강) - 중간 - 아래, 배경색으로 파서 대비 */
    ST7735S_FillCircle(cx, (int16_t)(cy - 5), 2, alertColor);
    ST7735S_FillCircle(cx, cy, 2, COLOR_BLACK);
    ST7735S_FillCircle(cx, (int16_t)(cy + 5), 2, COLOR_BLACK);
}

static void Icon_Warning(int16_t cx, int16_t cy, uint16_t color)
{
    ST7735S_FillRect((uint8_t)(cx - 2), (uint8_t)(cy - 9), 4U, 12U, color); /* 막대 */
    ST7735S_FillCircle(cx, (int16_t)(cy + 7), 2, color);                    /* 점 */
}

/* 좌측 60x99 영역(x:0~60, y:29~128)에 그리는 회전/직진 화살표 3종.
 * 셋 다 같은 박스 안에서 x=30을 기준으로 서로 대칭/변형된 좌표를 씁니다.
 * 우회전: 세로 막대 -> 둥근 모서리 -> 가로 막대 -> 화살촉(오른쪽) 순으로 겹침 */
static void Icon_ArrowRightTurn(uint16_t color)
{
    ST7735S_FillRect(12U, 65U, 8U, 40U, color);
    ST7735S_FillCircle(16, 65, 5, color);
    ST7735S_FillRect(16U, 61U, 24U, 8U, color);
    ST7735S_FillTriangle(46, 65, 36, 55, 36, 75, color);
}

/* 좌회전: 우회전 화살표를 x=30 기준으로 좌우 반전한 좌표 (x' = 60 - x) */
static void Icon_ArrowLeftTurn(uint16_t color)
{
    ST7735S_FillRect(40U, 65U, 8U, 40U, color);
    ST7735S_FillCircle(44, 65, 5, color);
    ST7735S_FillRect(20U, 61U, 24U, 8U, color);
    ST7735S_FillTriangle(14, 65, 24, 55, 24, 75, color);
}

/* 직진: 세로 막대 + 위쪽을 가리키는 화살촉 */
static void Icon_ArrowStraight(uint16_t color)
{
    ST7735S_FillRect(26U, 65U, 8U, 40U, color);
    ST7735S_FillTriangle(30, 50, 18, 65, 42, 65, color);
}

/* -------------------------------- 대시보드 --------------------------------
 * 이 아래부터가 실제로 화면 레이아웃(좌표)을 정하는 "애플리케이션" 계층입니다.
 * 위쪽은 전부 이 계층을 위한 도구 상자입니다. */

#define COLOR_TEXT COLOR_WHITE

/* 상단 바(y:0~28) 왼쪽에 신호등 3점을 가로로 배치.
 * 비활성 신호도 DrawCircle(윤곽선) 대신 FillCircle(어두운 색)로 그려서
 * 작은 반지름에서 윤곽선이 다이아몬드처럼 보이는 문제를 없앰. */
void Dashboard_DrawSignalDots(SignalColor active)
{
    const int16_t cy = 14;

    ST7735S_FillRect(0U, 0U, 52U, 28U, COLOR_BLACK);

    ST7735S_FillCircle(10, cy, 5, (active == SIG_RED) ? COLOR_RED : COLOR_DIM_RED);
    ST7735S_FillCircle(26, cy, 5, (active == SIG_YELLOW) ? COLOR_YELLOW : COLOR_DIM_AMBER);
    ST7735S_FillCircle(42, cy, 5, (active == SIG_GREEN) ? COLOR_GREEN : COLOR_DIM_GREEN);
}

static void Dashboard_DrawCountdown(uint8_t sec)
{
    char buf[3];

    buf[0] = (char)('0' + (sec / 10U));
    buf[1] = (char)('0' + (sec % 10U));
    buf[2] = '\0';

    ST7735S_DrawString(55, 3, buf, COLOR_TEXT, 2U);
}

void Dashboard_UpdateCountdown(uint8_t sec)
{
    ST7735S_FillRect(55U, 2U, 26U, 16U, COLOR_BLACK);
    Dashboard_DrawCountdown(sec);
}

/* 상단 바 오른쪽의 주행상태 라벨("RIGHT TURN"/"LEFT TURN"/"STRAIGHT")과
 * 좌측 화살표 아이콘을 dir 하나로 같이 결정해서 그림. */
void Dashboard_DrawDirection(TurnDirection dir)
{
    const char *label;

    /* 화살표 영역(좌측 60x99, 상단 바 아래) 초기화 */
    ST7735S_FillRect(0U, 29U, 60U, (uint8_t)(LCD_H - 29U), COLOR_BLACK);
    /* 라벨 영역(상단 바 오른쪽) 초기화 */
    ST7735S_FillRect(80U, 1U, (uint8_t)(LCD_W - 80U), 26U, COLOR_BLACK);

    if (dir == DIR_ERROR)
    {
        return;
    }

    switch (dir)
    {
        case DIR_LEFT:
            label = "LEFT TURN";
            Icon_ArrowLeftTurn(COLOR_BLUE);
            break;
        case DIR_RIGHT:
            label = "RIGHT TURN";
            Icon_ArrowRightTurn(COLOR_BLUE);
            break;
        case DIR_STRAIGHT:
        default:
            label = "STRAIGHT";
            Icon_ArrowStraight(COLOR_BLUE);
            break;
    }

    ST7735S_DrawString(90, 10, label, COLOR_TEXT, 1U);
}

/* 우측 경고 한 항목의 표시 내용 (사이드바 색, 아이콘, 라벨/상태 문구, 상태 색) */
typedef struct {
    uint8_t     flag;
    uint16_t    bar_color;
    const char *label;
    const char *status;
    uint16_t    status_color;
    uint8_t     icon; /* 0 = 차량, 1 = 보행자, 2 = 신호등, 4 = 경고 */
} WarningInfo;

/* 우회전 시 경고: 보행자 / 좌측 직진 차량 / 대향 비보호좌회전 */
static const WarningInfo warning_table_right[] = {
    { WARN_STRAIGHT_VEHICLE, COLOR_AMBER,  "L STRAIGHT", "CAUTION", COLOR_AMBER,  0U },
    { WARN_OPPOSITE_TURN,    COLOR_AMBER,  "OPP.LEFT",   "CAUTION", COLOR_AMBER,  0U },
    { WARN_PEDESTRIAN,       COLOR_RED,    "PEDESTRIAN", NULL,      COLOR_RED,    1U },
};
#define WARNING_TABLE_RIGHT_LEN (sizeof(warning_table_right) / sizeof(warning_table_right[0]))

/* 좌회전 시 경고: 대향 직진 / 대향 우회전 / 신호(적신호 임박) */
static const WarningInfo warning_table_left[] = {
    { WARN_STRAIGHT_VEHICLE, COLOR_AMBER, "OPP.STRAIGHT", "CAUTION",  COLOR_AMBER, 0U },
    { WARN_OPPOSITE_TURN,    COLOR_AMBER, "OPP. RIGHT",   "CAUTION",  COLOR_AMBER, 0U },
    { WARN_TL,               COLOR_RED,   "SIGNAL",       "RED SOON", COLOR_RED,   2U },
};
#define WARNING_TABLE_LEFT_LEN (sizeof(warning_table_left) / sizeof(warning_table_left[0]))

/* 통신 에러 */
static const WarningInfo warning_table_error[] = {
    { WARN_COMM_ERROR, COLOR_RED, "COMM ERROR", "NO DATA", COLOR_RED, 4U },
    { WARN_PEDESTRIAN, COLOR_RED, "PEDESTRIAN", NULL,      COLOR_RED, 1U },
};
#define WARNING_TABLE_ERROR_LEN (sizeof(warning_table_error) / sizeof(warning_table_error[0]))

void Dashboard_DrawWarnings(TurnDirection dir, uint8_t mask, uint8_t pedFlag)
{
    const WarningInfo *table;
    uint8_t table_len;
    const WarningInfo *active[3];
    uint8_t count;
    uint8_t i;
    uint16_t area_y;
    uint16_t area_h;
    uint16_t row_h;
    uint16_t y0;
    uint16_t cy;
    const char *status;
    uint16_t status_color;

    switch (dir)
    {
        case DIR_RIGHT:
            table = warning_table_right;
            table_len = (uint8_t)WARNING_TABLE_RIGHT_LEN;
            break;
        case DIR_LEFT:
            table = warning_table_left;
            table_len = (uint8_t)WARNING_TABLE_LEFT_LEN;
            break;
        case DIR_ERROR:
            table = warning_table_error;
            table_len = (uint8_t)WARNING_TABLE_ERROR_LEN;
            break;
        case DIR_STRAIGHT:
        default:
            table = NULL;
            table_len = 0U;
            break;
    }

    count = 0U;

    for (i = 0U; i < table_len; i++)
    {
        if ((mask & table[i].flag) != 0U)
        {
            active[count] = &table[i];
            count++;
        }
    }

    ST7735S_FillRect(61U, 29U, (uint8_t)(LCD_W - 61U), (uint8_t)(LCD_H - 29U), COLOR_BLACK);

    if (count == 0U)
    {
        return;
    }

    area_y = 29U;
    area_h = (uint16_t)(LCD_H - area_y);
    row_h = (uint16_t)(area_h / count);

    for (i = 0U; i < count; i++)
    {
        y0 = (uint16_t)(area_y + (i * row_h));
        cy = (uint16_t)(y0 + (row_h / 2U));

        ST7735S_FillRect(60U, (uint8_t)(y0 + 2U), 3U, (uint8_t)(row_h - 4U), active[i]->bar_color);

        if (active[i]->icon == 0U)
        {
            Icon_Car(74, (int16_t)cy, COLOR_ICON);
        }
        else if (active[i]->icon == 1U)
        {
            Icon_Pedestrian(74, (int16_t)cy, COLOR_ICON);
        }
        else if (active[i]->icon == 2U)
        {
            Icon_TrafficLight(74, (int16_t)cy, COLOR_ICON, COLOR_RED);
        }
        else if (active[i]->icon == 4U)
        {
            Icon_Warning(74, (int16_t)cy, COLOR_RED);
        }
        else
        {
            /* 정의되지 않은 아이콘 값: 아무 것도 그리지 않음 */
        }

        ST7735S_DrawString(86, (int16_t)(cy - 9U), active[i]->label, COLOR_TEXT, 1U);

        /* 보행자 항목만 status가 NULL -> pedFlag로 문구/색 결정 */
        if (active[i]->status != NULL)
        {
            status = active[i]->status;
            status_color = active[i]->status_color;
        }
        else if (pedFlag == 2U)
        {
            status = "AI N/A";   /* 2: 에러(AI 인식 불가) */
            status_color = COLOR_RED;
        }
        else
        {
            status = "DETECTED"; /* 1: 보행자 있음 */
            status_color = COLOR_RED;
        }

        ST7735S_DrawString(86, (int16_t)(cy + 3U), status, status_color, 1U);
    }
}

void Dashboard_DrawStatic(TurnDirection dir, uint8_t warnMask,
                          SignalColor sig, uint8_t countdown, uint8_t pedFlag)
{
    ST7735S_FillScreen(COLOR_BLACK);
    ST7735S_FillRect(0U, 28U, (uint8_t)LCD_W, 1U, COLOR_DIVIDER);
    ST7735S_FillRect(60U, 28U, 1U, (uint8_t)(LCD_H - 28U), COLOR_DIVIDER);

    Dashboard_DrawSignalDots(sig);
    Dashboard_DrawCountdown(countdown);
    ST7735S_DrawString(58, 20, "SEC", 0x7BEF, 1U);

    Dashboard_DrawDirection(dir);
    Dashboard_DrawWarnings(dir, warnMask, pedFlag);
}