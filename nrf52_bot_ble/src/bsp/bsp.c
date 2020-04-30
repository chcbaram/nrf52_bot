/*
 * bsp.c
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */




#include "bsp.h"
#include "uart.h"
#include "usb.h"
#include "rtos.h"
#include "nrf_sdm.h"
#include "nrf_mbr.h"
#include "nrf_clock.h"



volatile NRF_TIMER_Type *p_timer_us = NRF_TIMER4;

extern uint32_t __isr_vector_addr;
static volatile uint32_t systick_counter = 0;

extern void swtimerISR(void);


void SysTick_Handler(void)
{
  systick_counter++;
  swtimerISR();
  osSystickHandler();
}


void bspInit(void)
{

  uint32_t sd_version;

  sd_version = SD_VERSION_GET(MBR_SIZE);

  if (sd_version == SD_VERSION)
  {
    sd_softdevice_disable();
    sd_softdevice_vector_table_base_set((uint32_t)&__isr_vector_addr);

    /*
    sd_mbr_command_t command;

    command.command = SD_MBR_COMMAND_INIT_SD;
    sd_mbr_command(&command);

    command.command = SD_MBR_COMMAND_IRQ_FORWARD_ADDRESS_SET;
    command.params.irq_forward_address_set.address = (uint32_t)&__isr_vector_addr;
    sd_mbr_command(&command);
    */
  }

  nrf_systick_load_set(SystemCoreClock / (1000UL / (uint32_t)1)); // 1Khz
  nrf_systick_csr_set(
      NRF_SYSTICK_CSR_CLKSOURCE_CPU |
      NRF_SYSTICK_CSR_TICKINT_ENABLE |
      NRF_SYSTICK_CSR_ENABLE);


  p_timer_us->TASKS_STOP = (TIMER_TASKS_STOP_TASKS_STOP_Trigger << TIMER_TASKS_STOP_TASKS_STOP_Pos);
  p_timer_us->PRESCALER   = 4; // 16Mhz / (2^4) = 1Mhz
  p_timer_us->BITMODE     = (TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos);
  p_timer_us->MODE        = (TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos);
  p_timer_us->TASKS_START = (TIMER_TASKS_START_TASKS_START_Trigger << TIMER_TASKS_START_TASKS_START_Pos);
}

void bspDeInit(void)
{
  usbDeInit();


  // Disable Interrupts
  //
  for (int i=0; i<8; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFF;
    __DSB();
    __ISB();
  }
  SysTick->CTRL = 0;
}

int __io_putchar(int ch)
{
  uartWrite(_DEF_UART1, (uint8_t *)&ch, 1);
  return 1;
}

void delay(uint32_t ms)
{
  uint32_t pre_time = millis();

#ifdef _USE_HW_RTOS
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    osDelay(ms);
  }
  else
  {
    while(millis()-pre_time < ms);
  }
#else
  while(millis()-pre_time < ms);
#endif
}


uint32_t millis(void)
{
  static uint32_t pre_us_time = 0;
  static uint32_t ms_time_h = 0;
  static uint32_t ms_time_l = 0;
  uint32_t cur_us_time;

  cur_us_time = micros();
  ms_time_l = cur_us_time / 1000;


  if ((cur_us_time ^ pre_us_time) & (1<<31))
  {
    ms_time_h += 4294967; // Overflow 처리.
  }

  pre_us_time = cur_us_time;

  //return systick_counter;
  return (ms_time_h + ms_time_l);
}

uint32_t micros(void)
{
  volatile uint32_t cur_time;

  p_timer_us->TASKS_CAPTURE[0] = (TIMER_TASKS_CAPTURE_TASKS_CAPTURE_Trigger << TIMER_TASKS_CAPTURE_TASKS_CAPTURE_Pos);
  cur_time = (uint32_t)p_timer_us->CC[0];

  return cur_time;
}
