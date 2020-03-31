/*
 * hw_def.h
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */

#ifndef SRC_HW_HW_DEF_H_
#define SRC_HW_HW_DEF_H_


#include "def.h"
#include "bsp.h"



#define _HW_DEF_RTOS_MEM_SIZE(x)              ((x)/4)

#define _HW_DEF_RTOS_THREAD_PRI_MAIN          osPriorityNormal
#define _HW_DEF_RTOS_THREAD_PRI_USB           osPriorityNormal


#define _HW_DEF_RTOS_THREAD_MEM_MAIN          _HW_DEF_RTOS_MEM_SIZE( 6*1024)
#define _HW_DEF_RTOS_THREAD_MEM_USB           _HW_DEF_RTOS_MEM_SIZE( 2*1024)



#define _USE_HW_FLASH
#define _USE_HW_VCP
#define _USE_HW_RTOS


#define _USE_HW_LED
#define      HW_LED_MAX_CH          3

#define _USE_HW_UART
#define      HW_UART_MAX_CH         2

#define _USE_HW_CMDIF
#define      HW_CMDIF_LIST_MAX              32
#define      HW_CMDIF_CMD_STR_MAX           16
#define      HW_CMDIF_CMD_BUF_LENGTH        128

#define _USE_HW_SWTIMER
#define      HW_SWTIMER_MAX_CH      8

#define _USE_HW_BUTTON
#define      HW_BUTTON_MAX_CH       1

#define _USE_HW_SPI
#define      HW_SPI_MAX_CH          1

#define _USE_HW_RADIO
#define      HW_RADIO_BASE_ADDR     0x12345678
#define      HW_RADIO_MAX_BUF_LEN   8



#endif /* SRC_HW_HW_DEF_H_ */
