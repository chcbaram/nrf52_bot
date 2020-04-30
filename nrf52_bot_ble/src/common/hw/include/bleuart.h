/*
 * bleuart.h
 *
 *  Created on: 2020. 4. 30.
 *      Author: Baram
 */

#ifndef SRC_HW_DRIVER_BLEUART_BLEUART_H_
#define SRC_HW_DRIVER_BLEUART_BLEUART_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"

#ifdef _USE_HW_BLEUART



bool bleUartInit(void);
bool bleUartIsConnected(void);

uint32_t bleUartAvailable(void);
int32_t  bleUartWrite(uint8_t *p_data, uint32_t length);
uint8_t  bleUartRead(void);
int32_t  bleUartPrintf(const char *fmt, ...);


#endif

#ifdef __cplusplus
}
#endif


#endif /* SRC_HW_DRIVER_BLEUART_BLEUART_H_ */
