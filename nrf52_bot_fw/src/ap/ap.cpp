/*
 * ap.cpp
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */




#include "ap.h"
#include "nrf52botBLE.h"


void bootCmdif(void);
static void threadUsb(void const *argument);
static void threadLED(void const *argument);
static void threadLCD(void const *argument);
static void threadBLE(void const *argument);


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

  // Initialize callback task
  ble_callback_init(_HW_DEF_RTOS_THREAD_MEM_BLE_CALLBACK);

  osThreadDef(threadBLE, threadBLE, _HW_DEF_RTOS_THREAD_PRI_BLE, 0, _HW_DEF_RTOS_THREAD_MEM_BLE);
  osThreadCreate(osThread(threadBLE), NULL);
}

void apMain(void)
{
  while(1)
  {
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


#include "services/BLEDis.h"

void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);

static void threadBLE(void const *argument)
{
  BLEDis  bledis;  // device information

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  nrf52bot_ble.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  nrf52bot_ble.configPrphBandwidth(BANDWIDTH_MAX);
  nrf52bot_ble.begin();
  nrf52bot_ble.setTxPower(4);    // Check bluefruit.h for supported values
  nrf52bot_ble.setName("nrf52bot");
  nrf52bot_ble.Periph.setConnectCallback(connect_callback);
  nrf52bot_ble.Periph.setDisconnectCallback(disconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("Adafruit Industries");
  bledis.setModel("Bluefruit Feather52");
  bledis.begin();

  // Advertising packet
  nrf52bot_ble.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  nrf52bot_ble.Advertising.addTxPower();

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  nrf52bot_ble.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   *
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html
   */
  nrf52bot_ble.Advertising.restartOnDisconnect(true);
  nrf52bot_ble.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  nrf52bot_ble.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  nrf52bot_ble.Advertising.start(0);                // 0 = Don't stop advertising after n seconds

  for(;;)
  {

  }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = nrf52bot_ble.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  logPrintf("Connected to ");
  logPrintf(central_name);
  logPrintf("\r\n");
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  logPrintf("\r\n");
  logPrintf("Disconnected, reason = 0x"); logPrintf("%X", reason);
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
