/*
 * lcd.h
 *
 *  Created on: 2020. 3. 27.
 *      Author: Baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_LCD_H_
#define SRC_COMMON_HW_INCLUDE_LCD_H_





#include "hw_def.h"


#ifdef _USE_HW_LCD



#define LCD_WIDTH     HW_LCD_WIDTH
#define LCD_HEIGHT    HW_LCD_HEIGHT


#if HW_LCD_COLOR_SWAP == 1
enum class_color {
 white     = 0xFFFF,
 gray      = 0x1084,
 darkgray  = 0x55AD,
 black     = 0x0000,
 purple    = 0x1080,
 pink      = 0x19FE,
 red       = 0x00F8,
 orange    = 0x20FD,
 brown     = 0x45A1,
 beige     = 0xBBF7,
 yellow    = 0xE0FF,
 lightgreen= 0x7297,
 green     = 0x0004,
 darkblue  = 0x1100,
 blue      = 0x1F00,
 lightblue = 0xDCAE,
};
#else
enum class_color {
 white     = 0xFFFF,
 gray      = 0x8410,
 darkgray  = 0xAD55,
 black     = 0x0000,
 purple    = 0x8010,
 pink      = 0xFE19,
 red       = 0xF800,
 orange    = 0xFD20,
 brown     = 0xA145,
 beige     = 0xF7BB,
 yellow    = 0xFFE0,
 lightgreen= 0x9772,
 green     = 0x0400,
 darkblue  = 0x0011,
 blue      = 0x001F,
 lightblue = 0xAEDC,
};
#endif

typedef struct lcd_driver_t_ lcd_driver_t;

typedef struct lcd_driver_t_
{
  bool     (*init)(void);
  void     (*setWindow)(int32_t x, int32_t y, int32_t w, int32_t h);
  uint16_t (*getWidth)(void);
  uint16_t (*getHeight)(void);
  bool     (*setCallBack)(void (*p_func)(void));
  bool     (*sendBuffer)(uint8_t *p_data, uint32_t length, uint32_t timeout_ms);

} lcd_driver_t;



bool lcdInit(void);
bool lcdIsInit(void);
void lcdReset(void);

uint8_t lcdGetBackLight(void);
void    lcdSetBackLight(uint8_t value);

uint32_t lcdReadPixel(uint16_t x_pos, uint16_t y_pos);
void lcdClear(uint32_t rgb_code);
void lcdClearBuffer(uint32_t rgb_code);

bool lcdDrawAvailable(void);
bool lcdRequestDraw(void);
void lcdUpdateDraw(void);
void lcdSetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void lcdDisplayOff(void);
void lcdDisplayOn(void);

uint32_t lcdGetFps(void);
uint32_t lcdGetFpsTime(void);

int32_t lcdGetWidth(void);
int32_t lcdGetHeight(void);

uint16_t *lcdGetFrameBuffer(void);
uint16_t *lcdGetCurrentFrameBuffer(void);
void lcdSetDoubleBuffer(bool enable);

void lcdDrawPixel(uint16_t x_pos, uint16_t y_pos, uint32_t rgb_code);
void lcdDrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint16_t color);
void lcdDrawVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void lcdDrawHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void lcdDrawFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void lcdDrawFillScreen(uint16_t color);
void lcdDrawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void lcdPrintf(int x, int y, uint16_t color,  const char *fmt, ...);
uint32_t lcdGetStrWidth(const char *fmt, ...);

static inline
uint16_t lcdSwap16(uint16_t data) { return (data<<8) | (data>>8); }



#endif /* _USE_HW_LCD */



#endif /* SRC_COMMON_HW_INCLUDE_LCD_H_ */
