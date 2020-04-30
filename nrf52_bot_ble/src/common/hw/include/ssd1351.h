/*
 * ssd1351.h
 *
 *  Created on: 2020. 3. 26.
 *      Author: HanCheol Cho
 */

#ifndef SRC_COMMON_HW_INCLUDE_SSD1351_H_
#define SRC_COMMON_HW_INCLUDE_SSD1351_H_



#ifdef __cplusplus
 extern "C" {
#endif



#include "hw_def.h"


#ifdef _USE_HW_SSD1351

#include "ssd1351_regs.h"



#define SSD1351_LCD_WIDTH      128
#define SSD1351_LCD_HEIGHT     96




bool ssd1351Init(void);
bool ssd1351DrawAvailable(void);
bool ssd1351RequestDraw(void);
void ssd1351SetFrameBuffer(uint16_t *p_buf);
void ssd1351SetWindow(int32_t x, int32_t y, int32_t w, int32_t h);

uint32_t ssd1351GetFps(void);
uint32_t ssd1351GetFpsTime(void);

uint16_t ssd1351GetWidth(void);
uint16_t ssd1351GetHeight(void);


#endif

#ifdef __cplusplus
}
#endif



#endif
