/*
 * ssd1351.c
 *
 *  Created on: 2020. 3. 26.
 *      Author: Baram
 */




#include "ssd1351.h"

#ifdef _USE_HW_SSD1351
#include "gpio.h"
#include "spi.h"



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




#define _PIN_DEF_RST    0

static uint8_t spi_ch = _DEF_SPI1;

static int32_t _width  = SSD1351_LCD_WIDTH;
static int32_t _height = SSD1351_LCD_HEIGHT;

static void ssd1351InitRegs(void);
static void writecommand(uint8_t c);
static void writedata(uint8_t d);
static void ssd1351FillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color);
static void ssd1351SetRotation(uint8_t m);

bool ssd1351Init(void)
{
  spiBegin(spi_ch);
  spiSetClockDivider(spi_ch, SPI_DIV_2); // 8Mhz

  //-- Reset Lcd
  //
  gpioPinWrite(_PIN_DEF_RST, _DEF_HIGH);
  delay(10);
  gpioPinWrite(_PIN_DEF_RST, _DEF_LOW);
  delay(50);
  gpioPinWrite(_PIN_DEF_RST, _DEF_HIGH);
  delay(10);


  ssd1351InitRegs();
  ssd1351SetRotation(0);

  uint32_t pre_time = millis();
  uint32_t y = 0;
  ssd1351FillRect(0,  y, _width,  _height/5, black); y += _height/5;
  ssd1351FillRect(0,  y, _width,  _height/5, red);   y += _height/5;
  ssd1351FillRect(0,  y, _width,  _height/5, green); y += _height/5;
  ssd1351FillRect(0,  y, _width,  _height/5, blue);  y += _height/5;
  ssd1351FillRect(0,  y, _width,  _height/5, white); y += _height/5;
  logPrintf("%d ms\n", (int)(millis()-pre_time));
  return true;
}



void writecommand(uint8_t c)
{
  spiSetDCX(spi_ch, 1);
  spiDmaTransfer(spi_ch, &c, 1, 50);
}

void writedata(uint8_t d)
{
  spiDmaTransfer(spi_ch, &d, 1, 50);
}

void ssd1351InitRegs(void)
{
  writecommand(SSD1351_CMD_COMMANDLOCK);
  writedata(0x12);
  writecommand(SSD1351_CMD_COMMANDLOCK);
  writedata(0xB1);

  writecommand(SSD1351_CMD_DISPLAYOFF);

  writecommand(SSD1351_CMD_CLOCKDIV);
  writedata(0xF1);
  writecommand(SSD1351_CMD_MUXRATIO);
  writedata(127);
  writecommand(SSD1351_CMD_DISPLAYOFFSET);
  writedata(0x00);
  writecommand(SSD1351_CMD_SETGPIO);
  writedata(0x00);
  writecommand(SSD1351_CMD_FUNCTIONSELECT);
  writedata(0x01);
  writecommand(SSD1351_CMD_PRECHARGE);
  writedata(0x32);
  writecommand(SSD1351_CMD_VCOMH);
  writedata(0x05);
  writecommand(SSD1351_CMD_NORMALDISPLAY);
  writecommand(SSD1351_CMD_CONTRASTABC);
  writedata(0xC8);
  writedata(0x80);
  writedata(0xC8);
  writecommand(SSD1351_CMD_CONTRASTMASTER);
  writedata(0x0F);

  writecommand(SSD1351_CMD_SETVSL);
  writedata(0xA0);
  writedata(0xB5);
  writedata(0x55);

  writecommand(SSD1351_CMD_PRECHARGE2);
  writedata(0x01);


  writecommand(SSD1351_CMD_NORMALDISPLAY);
  writecommand(SSD1351_CMD_DISPLAYON);

  delay(50);
}


void ssd1351SetRotation(uint8_t m)
{
  uint8_t rotation;

  // madctl bits:
  // 6,7 Color depth (01 = 64K)
  // 5   Odd/even split COM (0: disable, 1: enable)
  // 4   Scan direction (0: top-down, 1: bottom-up)
  // 3   Reserved
  // 2   Color remap (0: A->B->C, 1: C->B->A)
  // 1   Column remap (0: 0-127, 1: 127-0)
  // 0   Address increment (0: horizontal, 1: vertical)
  uint8_t madctl = 0b01100100; // 64K, enable split, CBA

  rotation = m & 3; // Clip input to valid range


  switch (rotation)
  {
    case 0:
      madctl |= 0b00010000; // Scan bottom-up
      //_width  = _init_width;
      //_height = _init_height;
      break;
    case 1:
      madctl |= 0b00010011; // Scan bottom-up, column remap 127-0, vertical
      //_width  = _init_height;
      //_height = _init_width;
      break;
    case 2:
      madctl |= 0b00000010; // Column remap 127-0
      //_width  = _init_width;
      //_height = _init_height;
      break;
    case 3:
      madctl |= 0b00000001; // Vertical
      //_width  = _init_height;
      //_height = _init_width;
      break;
  }

  writecommand(SSD1351_CMD_SETREMAP);
  writedata(madctl);
  uint8_t startline = (rotation < 2) ? SSD1351_LCD_HEIGHT : 0;

  writecommand(SSD1351_CMD_STARTLINE);
  writedata(startline);
}

void ssd1351SetWindow(int32_t x0, int32_t y0, int32_t x1, int32_t y1)
{
  uint8_t buf[8];


  buf[0] = SSD1351_CMD_SETCOLUMN;
  buf[1] = x0;
  buf[2] = x1;
  spiSetDCX(spi_ch, 1);
  spiDmaTransfer(spi_ch, buf, 3, 50);

  buf[0] = SSD1351_CMD_SETROW;
  buf[1] = y0;
  buf[2] = y1;
  spiSetDCX(spi_ch, 1);
  spiDmaTransfer(spi_ch, buf, 3, 50);

  buf[0] = SSD1351_CMD_WRITERAM;
  spiSetDCX(spi_ch, 1);
  spiDmaTransfer(spi_ch, buf, 1, 50);
}


void ssd1351FillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint32_t color)
{
  uint16_t buf[w];

  // Clipping
  if ((x >= _width) || (y >= _height)) return;

  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }

  if ((x + w) > _width)  w = _width  - x;
  if ((y + h) > _height) h = _height - y;

  if ((w < 1) || (h < 1)) return;


  for (int i=0; i<w; i++)
  {
    buf[i] = (color<<8) | (color>>8);
  }

  ssd1351SetWindow(x, y, x + w - 1, y + h - 1);

  for (int i=0; i<h; i++)
  {
    spiDmaTransfer(spi_ch, buf, w*2, 50);
  }

}


#endif
