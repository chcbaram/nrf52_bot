/*
 * hw.c
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */




#include "hw.h"



extern uint32_t __isr_vector_addr;
extern uint32_t _image_start;
extern uint32_t _image_size;



__attribute__((section(".tag"))) const flash_tag_t fw_tag =
   {
    // fw info
    //
    .magic_number = 0xAAAA5555,
    .version_str  = "V200326R1",
    .board_str    = "NRF52_BOT_B/D",
    .name_str     = "Firmware",
    .date_str     = __DATE__,
    .time_str     = __TIME__,
    .addr_tag     = (uint32_t)&fw_tag,
    .addr_fw      = (uint32_t)&__isr_vector_addr,

    .load_start   = (uint32_t)&_image_start,  // load_addr
    .load_size    = (uint32_t)&_image_size,   // load_size
   };


void blectlr_assertion_handler(const char *const file, const uint32_t line)
{
  while(1)
  {

  }
}

static void m_assert_handler(const char *const file, const uint32_t line)
{
  while(1)
  {

  }
}

void host_signal(void)
{
  /* Wake up the RX event/data thread */
  return;
}


static uint8_t ble_controller_mempool[32*1024];

void hwInit(void)
{
  bspInit();

  swtimerInit();
  cmdifInit();

  ledInit();
  buttonInit();
  vcpInit();
  uartInit();
  uartOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);

  logPrintf("\n\n[ Firmware Begin... ]\r\n");
  logPrintf("Addr Tag   \t\t: 0x%X\r\n", (int)fw_tag.addr_tag);
  logPrintf("Addr Fw    \t\t: 0x%X\r\n", (int)fw_tag.addr_fw);
  logPrintf("Addr Hw    \t\t: 0x%X\r\n", (int)hwInit);

  gpioInit();
  spiInit();
  //flashInit();

  //usbInit();

  lcdInit();  




  // After usbInit()
  //radioInit();


  mpsl_clock_lfclk_cfg_t clock_cfg;

#if 1
  clock_cfg.source = MPSL_CLOCK_LF_SRC_RC;
  clock_cfg.accuracy_ppm = MPSL_DEFAULT_CLOCK_ACCURACY_PPM;

  clock_cfg.rc_ctiv = MPSL_RECOMMENDED_RC_CTIV;
  clock_cfg.rc_temp_ctiv = MPSL_RECOMMENDED_RC_TEMP_CTIV;
#else
  clock_cfg.source = MPSL_CLOCK_LF_SRC_XTAL;
  clock_cfg.accuracy_ppm = MPSL_DEFAULT_CLOCK_ACCURACY_PPM;

  clock_cfg.rc_ctiv = 0;
  clock_cfg.rc_temp_ctiv = 0;
#endif

  int err = 0;







  err = mpsl_init(&clock_cfg, SWI5_EGU5_IRQn, m_assert_handler);
  if (err)
  {
    return;
  }



  err = ble_controller_init(blectlr_assertion_handler);
  if (err)
  {
    return;
  }




  int required_memory;
  ble_controller_cfg_t cfg;
  cfg.master_count.count = BLE_CONTROLLER_DEFAULT_MASTER_COUNT;

#if 1
  /* NOTE: ble_controller_cfg_set() returns a negative errno on error. */
  required_memory =
    ble_controller_cfg_set(BLE_CONTROLLER_DEFAULT_RESOURCE_CFG_TAG,
               BLE_CONTROLLER_CFG_TYPE_MASTER_COUNT,
               &cfg);
  if (required_memory < 0)
  {
    return;
  }

  cfg.slave_count.count = BLE_CONTROLLER_DEFAULT_SLAVE_COUNT;

  required_memory =
    ble_controller_cfg_set(BLE_CONTROLLER_DEFAULT_RESOURCE_CFG_TAG,
               BLE_CONTROLLER_CFG_TYPE_SLAVE_COUNT,
               &cfg);
  if (required_memory < 0)
  {
    return;
  }

  cfg.buffer_cfg.rx_packet_size = BLE_CONTROLLER_DEFAULT_RX_PACKET_SIZE;
  cfg.buffer_cfg.tx_packet_size = BLE_CONTROLLER_DEFAULT_TX_PACKET_SIZE;
  cfg.buffer_cfg.rx_packet_count = BLE_CONTROLLER_DEFAULT_RX_PACKET_COUNT;
  cfg.buffer_cfg.tx_packet_count = BLE_CONTROLLER_DEFAULT_TX_PACKET_COUNT;

  required_memory =
    ble_controller_cfg_set(BLE_CONTROLLER_DEFAULT_RESOURCE_CFG_TAG,
               BLE_CONTROLLER_CFG_TYPE_BUFFER_CFG,
               &cfg);
  if (required_memory < 0)
  {
    return;
  }

  cfg.event_length.event_length_us = 7500;
  required_memory =
    ble_controller_cfg_set(BLE_CONTROLLER_DEFAULT_RESOURCE_CFG_TAG,
               BLE_CONTROLLER_CFG_TYPE_EVENT_LENGTH,
               &cfg);
  if (required_memory < 0)
  {
    return;
  }



  err = ble_controller_enable(host_signal,
            ble_controller_mempool);

#endif


  NVIC_SetPriority(RTC0_IRQn, MPSL_HIGH_IRQ_PRIORITY);
  NVIC_EnableIRQ(RTC0_IRQn);
  NVIC_SetPriority(TIMER0_IRQn, MPSL_HIGH_IRQ_PRIORITY);
  NVIC_EnableIRQ(TIMER0_IRQn);
  NVIC_SetPriority(RADIO_IRQn, MPSL_HIGH_IRQ_PRIORITY);
  NVIC_EnableIRQ(RADIO_IRQn);

  NVIC_SetPriority(SWI5_EGU5_IRQn, 4);
  NVIC_EnableIRQ(SWI5_EGU5_IRQn);

  NVIC_SetPriority(RNG_IRQn, 5);
  NVIC_EnableIRQ(RNG_IRQn);


  uint8_t cmd[8];
  uint8_t evt[257];

  memset(evt, 0x00, 257);

  err = hci_evt_get(evt);

  cmd[0] = 0x0A;
  cmd[1] = 0x20;
  //cmd[0] = 0x03;
  //cmd[1] = 0x0C;
  cmd[2] = 1;
  cmd[3] = 1;
  err = hci_cmd_put(cmd);

  delay(100);



  err = hci_evt_get(evt);



  return;
}




void RNG_IRQHandler(void)
{
  ble_controller_RNG_IRQHandler();
}

void RTC0_IRQHandler(void)
{
  MPSL_IRQ_RTC0_Handler();
}

void TIMER0_IRQHandler(void)
{
  MPSL_IRQ_TIMER0_Handler();
}

void RADIO_IRQHandler(void)
{
  MPSL_IRQ_RADIO_Handler();
}

void SWI5_EGU5_IRQHandler(void)
{
  mpsl_low_priority_process();
}
