/*
 * ap.cpp
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */




#include "ap.h"



void bootCmdif(void);


void apInit(void)
{
  hwInit();

  cmdifOpen(_DEF_UART1, 57600);
  cmdifAdd("boot", bootCmdif);

}

void apMain(void)
{
  uint32_t pre_time;
  uint16_t x = 0;
  uint16_t y = 0;
  bool update = false;


  while(1)
  {
    if (millis()-pre_time >= 500)
    {
      pre_time = millis();

      ledToggle(_DEF_LED1);
      update = true;
    }
    cmdifMain();

    if ( tusb_inited() )
    {
      tud_task();
    }

#if 1
    static uint32_t fps_time = 0;
    static uint32_t fps = 0;

    if (lcdDrawAvailable() > 0)
    {
      lcdClearBuffer(black);

      if (update == true)
      {
        update = false;
        fps_time = lcdGetFpsTime();
        fps = lcdGetFps();
      }

      lcdPrintf(0,  0, white, "%d ms", fps_time);
      lcdPrintf(0, 16, white, "%d fps", fps);

      lcdDrawFillRect(x, 32, 20, 20, red);
      lcdDrawFillRect(lcdGetWidth()-x, 52, 20, 20, green);
      lcdDrawFillRect(x + 30, 72, 20, 20, blue);

      x += 2;

      x %= lcdGetWidth();
      y %= lcdGetHeight();

      lcdRequestDraw();
    }
#endif
  }
}





void bootCmdif(void)
{
  bool ret = true;


  if (cmdifGetParamCnt() == 1 && cmdifHasString("reset", 0) == true)
  {
    bspDeInit();
    NVIC_SystemReset();
  }
  else if (cmdifGetParamCnt() == 1 && cmdifHasString("spi", 0) == true)
  {
    uint8_t data[2] = {1, 2};
    uint32_t pre_time;

    pre_time = millis();
    if (spiDmaTransfer(_DEF_SPI1, (void *)data, 2, 100) == true)
    {
      cmdifPrintf("spi tx ok, %d ms\n", millis()-pre_time);
    }
    else
    {
      cmdifPrintf("spi tx fail\n");
    }
  }
  else
  {
    ret = false;
  }

  if (ret == false)
  {
    cmdifPrintf( "boot reset \n");
  }
}
