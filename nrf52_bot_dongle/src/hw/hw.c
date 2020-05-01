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
    .board_str    = "NRF52_BOT_DONGLE_B/D",
    .name_str     = "Firmware",
    .date_str     = __DATE__,
    .time_str     = __TIME__,
    .addr_tag     = (uint32_t)&fw_tag,
    .addr_fw      = (uint32_t)&__isr_vector_addr,

    .load_start   = (uint32_t)&_image_start,  // load_addr
    .load_size    = (uint32_t)&_image_size,   // load_size
   };


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




  //gpioInit();
  spiInit();
  flashInit();

  usbInit();

  // After usbInit()
  //radioInit();
  bleUartInit();
}
