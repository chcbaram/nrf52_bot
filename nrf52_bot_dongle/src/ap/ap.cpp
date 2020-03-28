/*
 * ap.cpp
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */




#include "ap.h"



void bootCmdif(void);
static void threadUsb(void const *argument);


void apInit(void)
{
  hwInit();

  cmdifOpen(_DEF_UART2, 57600);
  cmdifAdd("boot", bootCmdif);

  osThreadDef(threadUsb, threadUsb, _HW_DEF_RTOS_THREAD_PRI_USB, 0, _HW_DEF_RTOS_THREAD_MEM_USB);
  osThreadCreate(osThread(threadUsb), NULL);
}

void apMain(void)
{
  uint32_t pre_time;


  while(1)
  {
    if (millis()-pre_time >= 500)
    {
      pre_time = millis();

      ledToggle(_DEF_LED1);
    }
    cmdifMain();
  }
}


static void threadUsb(void const *argument)
{

  while(1)
  {
    if ( tusb_inited() )
    {
      tud_task();
    }
    osThreadYield();
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
