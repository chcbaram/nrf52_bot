/*
 * bleuart_c.h
 *
 *  Created on: 2020. 5. 2.
 *      Author: Baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_BLEUART_C_H_
#define SRC_COMMON_HW_INCLUDE_BLEUART_C_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "hw_def.h"

#ifdef _USE_HW_BLEUART_C



bool bleUartInit(void);
bool bleUartIsConnected(void);
bool bleUartUpdate(void);


uint32_t bleUartAvailable(void);
int32_t  bleUartWrite(uint8_t *p_data, uint32_t length);
uint8_t  bleUartRead(void);
int32_t  bleUartPrintf(const char *fmt, ...);


#endif

#ifdef __cplusplus
}
#endif



#endif /* SRC_COMMON_HW_INCLUDE_BLEUART_C_H_ */
