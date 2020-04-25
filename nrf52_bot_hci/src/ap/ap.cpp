/*
 * ap.cpp
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */




#include "ap.h"
#include "ArduinoBLE/ArduinoBLE.h"



void bootCmdif(void);
static void threadUsb(void const *argument);
static void threadLED(void const *argument);
static void threadLCD(void const *argument);

extern void HCIUpdate(void);


BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic switchCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);



void apInit(void)
{
  hwInit();

  cmdifOpen(_DEF_UART1, 57600);
  cmdifAdd("boot", bootCmdif);

  osThreadDef(threadUsb, threadUsb, _HW_DEF_RTOS_THREAD_PRI_USB, 0, _HW_DEF_RTOS_THREAD_MEM_USB);
  osThreadCreate(osThread(threadUsb), NULL);

  osThreadDef(threadLED, threadLED, _HW_DEF_RTOS_THREAD_PRI_LED, 0, _HW_DEF_RTOS_THREAD_MEM_LED);
  osThreadCreate(osThread(threadLED), NULL);

  osThreadDef(threadLCD, threadLCD, _HW_DEF_RTOS_THREAD_PRI_LCD, 0, _HW_DEF_RTOS_THREAD_MEM_LCD);
  osThreadCreate(osThread(threadLCD), NULL);

}

void apMain(void)
{
  // begin initialization
  if (!BLE.begin())
  {
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName("LED");
  BLE.setAdvertisedService(ledService);

  // add the characteristic to the service
  ledService.addCharacteristic(switchCharacteristic);

  // add service
  BLE.addService(ledService);

  // set the initial value for the characeristic:
  switchCharacteristic.writeValue(0);

  // start advertising
  int ret = BLE.advertise();


  while(1)
  {
    BLEDevice central = BLE.central();

    cmdifMain();
  }
}


static void threadUsb(void const *argument)
{

  for(;;)
  {
    if (tusb_inited()){
      tud_task();
    }
    osThreadYield();

    HCIUpdate();
  }
}


static void threadLED(void const *argument)
{
  for(;;)
  {
    ledToggle(_DEF_LED1);
    osDelay(500);
  }
}


static void threadLCD(void const *argument)
{
  bool update = false;
  uint32_t pre_time;
  uint32_t fps_time = 0;
  uint32_t fps = 0;
  uint16_t x = 0;
  uint16_t y = 0;

  for(;;)
  {
    if (lcdDrawAvailable() > 0){
      lcdClearBuffer(black);

      if (update == true) {
        update = false;
        fps_time = lcdGetFpsTime();
        fps = lcdGetFps();
      }

      lcdPrintf(0, 0, white, "%d ms", fps_time);
      lcdPrintf(0, 16, white, "%d fps", fps);

      lcdDrawFillRect(x, 32, 20, 20, red);
      lcdDrawFillRect(lcdGetWidth() - x, 52, 20, 20, green);
      lcdDrawFillRect(x + 30, 72, 20, 20, blue);

      x += 2;

      x %= lcdGetWidth();
      y %= lcdGetHeight();

      lcdRequestDraw();
    }

    if (millis()-pre_time >= 500){
      pre_time = millis();
      update = true;
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
