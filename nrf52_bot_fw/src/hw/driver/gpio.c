/*
 * gpio.c
 *
 *  Created on: 2020. 3. 26.
 *      Author: HanCheol Cho
 */






#include "gpio.h"
#include "cmdif.h"




typedef struct
{
  uint32_t      pin;
  uint8_t       mode;
  uint8_t       on_state;
  uint8_t       init_state;
} gpio_tbl_t;


gpio_tbl_t gpio_tbl[GPIO_MAX_CH] =
{
  {NRF_GPIO_PIN_MAP(1, 7), _DEF_OUTPUT,  _DEF_HIGH, _DEF_HIGH},  // 0. LCD_RESET
};



void gpioCmdif(void);



bool gpioInit(void)
{
  uint32_t i;


  for (i=0; i<GPIO_MAX_CH; i++)
  {
    if (gpio_tbl[i].mode == _DEF_OUTPUT)
    {
      gpioPinWrite(i, gpio_tbl[i].init_state);
    }

    gpioPinMode(i, gpio_tbl[i].mode);
  }


  cmdifAdd("gpio", gpioCmdif);

  return true;
}

void gpioPinMode(uint8_t channel, uint8_t mode)
{
  if (channel >= GPIO_MAX_CH)
  {
    return;
  }

  switch (mode)
  {
    case _DEF_INPUT:
      nrf_gpio_cfg(
          gpio_tbl[channel].pin,
          NRF_GPIO_PIN_DIR_INPUT,
          NRF_GPIO_PIN_INPUT_CONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_S0S1,
          NRF_GPIO_PIN_NOSENSE);
      break;

    case _DEF_INPUT_PULLUP:
      nrf_gpio_cfg(
          gpio_tbl[channel].pin,
          NRF_GPIO_PIN_DIR_INPUT,
          NRF_GPIO_PIN_INPUT_CONNECT,
          NRF_GPIO_PIN_PULLUP,
          NRF_GPIO_PIN_S0S1,
          NRF_GPIO_PIN_NOSENSE);
      break;

    case _DEF_INPUT_PULLDOWN:
      nrf_gpio_cfg(
          gpio_tbl[channel].pin,
          NRF_GPIO_PIN_DIR_INPUT,
          NRF_GPIO_PIN_INPUT_CONNECT,
          NRF_GPIO_PIN_PULLDOWN,
          NRF_GPIO_PIN_S0S1,
          NRF_GPIO_PIN_NOSENSE);
      break;

    case _DEF_OUTPUT:
      nrf_gpio_cfg(
          gpio_tbl[channel].pin,
          NRF_GPIO_PIN_DIR_OUTPUT,
          NRF_GPIO_PIN_INPUT_DISCONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_S0S1,
          NRF_GPIO_PIN_NOSENSE);
      break;

    case _DEF_OUTPUT_PULLUP:
      nrf_gpio_cfg(
          gpio_tbl[channel].pin,
          NRF_GPIO_PIN_DIR_OUTPUT,
          NRF_GPIO_PIN_INPUT_DISCONNECT,
          NRF_GPIO_PIN_PULLUP,
          NRF_GPIO_PIN_S0S1,
          NRF_GPIO_PIN_NOSENSE);
      break;

    case _DEF_OUTPUT_PULLDOWN:
      nrf_gpio_cfg(
          gpio_tbl[channel].pin,
          NRF_GPIO_PIN_DIR_OUTPUT,
          NRF_GPIO_PIN_INPUT_DISCONNECT,
          NRF_GPIO_PIN_PULLDOWN,
          NRF_GPIO_PIN_S0S1,
          NRF_GPIO_PIN_NOSENSE);
      break;
  }
}

void gpioPinWrite(uint8_t channel, uint8_t value)
{
  uint8_t pin_value;

  if (channel >= GPIO_MAX_CH)
  {
    return;
  }

  if (value > 0)
  {
    pin_value = gpio_tbl[channel].on_state;
  }
  else
  {
    pin_value = !gpio_tbl[channel].on_state;
  }

  nrf_gpio_pin_write(gpio_tbl[channel].pin, pin_value);
}

uint8_t gpioPinRead(uint8_t channel)
{
  if (nrf_gpio_pin_read(gpio_tbl[channel].pin) == gpio_tbl[channel].on_state)
  {
    return _DEF_HIGH;
  }
  else
  {
    return _DEF_LOW;
  }
}

void gpioPinToggle(uint8_t channel)
{
  if (channel >= GPIO_MAX_CH)
  {
    return;
  }

  nrf_gpio_pin_toggle(gpio_tbl[channel].pin);
}





//-- gpioCmdif
//
void gpioCmdif(void)
{
  bool ret = true;
  uint8_t number;
  uint8_t state;


  if (cmdifGetParamCnt() == 1)
  {
    if (cmdifHasString("show", 0) == true)
    {
      while(cmdifRxAvailable() == 0)
      {
        for (int i=0; i<GPIO_MAX_CH; i++)
        {
          cmdifPrintf("%d ", gpioPinRead(i));
        }
        cmdifPrintf("\r\n");
        delay(50);
      }
    }
    else
    {
      ret = false;
    }
  }
  else if (cmdifGetParamCnt() == 2)
  {
    if (cmdifHasString("read", 0) == true)
    {
      number = (uint8_t)cmdifGetParam(1);

      while(cmdifRxAvailable() == 0)
      {
        cmdifPrintf("gpio %d : %d\r\n", number, gpioPinRead(number));
        delay(50);
      }
    }
    else
    {
      ret = false;
    }
  }
  else if (cmdifGetParamCnt() == 3)
  {
    if (cmdifHasString("write", 0) == true)
    {
      number = (uint8_t)cmdifGetParam(1);
      state = (uint8_t)cmdifGetParam(2);

      gpioPinWrite(number, state);

      cmdifPrintf("gpio write %d : %d\r\n", number, gpioPinRead(number));
    }
    else if (cmdifHasString("on", 0) == true)
    {
      uint32_t on_time;

      number = (uint8_t)cmdifGetParam(1);
      on_time = (uint32_t)cmdifGetParam(2);

      gpioPinWrite(number, 1);
      delay(on_time);
      gpioPinWrite(number, 0);

      cmdifPrintf("gpio on %d : %d ms\r\n", number, on_time);
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }


  if (ret == false)
  {
    cmdifPrintf( "gpio show\n");
    cmdifPrintf( "gpio read  0~%d\n", GPIO_MAX_CH-1);
    cmdifPrintf( "gpio write 0~%d [0:1]\n", GPIO_MAX_CH-1);
    cmdifPrintf( "gpio on 0~%d ms[0~3000]\n", GPIO_MAX_CH-1);
  }
}
