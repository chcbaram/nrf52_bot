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

uint8_t buf[255];

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

    bleUartUpdate();

#ifdef _USE_HW_BLEUART
    static int i = 0;
    static uint32_t diff_time;
    static uint32_t pre_time_uart;

    pre_time_uart = millis();
    bleUartPrintf("ble12345678901234567890 %d %d\n", i++, diff_time);
    diff_time = millis()-pre_time_uart;
#else

    static uint32_t pre_time_ble;


    if (millis()-pre_time_ble >= 100)
    {
      pre_time_ble = millis();
      bleUartPrintf("a");
    }
    while (bleUartAvailable() > 0)
    {
      uartPutch(_DEF_UART2, bleUartRead());
    }
#endif
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
