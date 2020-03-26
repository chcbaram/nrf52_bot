/*
 * hw.h
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */

#ifndef SRC_HW_HW_H_
#define SRC_HW_HW_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "hw_def.h"


#include "led.h"
#include "uart.h"
#include "cmdif.h"
#include "swtimer.h"
#include "button.h"
#include "flash.h"
#include "usb.h"
#include "vcp.h"
#include "spi.h"
#include "gpio.h"
#include "ili9341.h"


void hwInit(void);


#ifdef __cplusplus
}
#endif


#endif /* SRC_HW_HW_H_ */
