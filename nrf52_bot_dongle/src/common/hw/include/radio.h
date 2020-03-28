/*
 * radio.h
 *
 *  Created on: 2020. 3. 28.
 *      Author: Baram
 */

#ifndef SRC_COMMON_HW_INCLUDE_RADIO_H_
#define SRC_COMMON_HW_INCLUDE_RADIO_H_



#ifdef __cplusplus
extern "C" {
#endif


#include "hw_def.h"

#ifdef _USE_HW_RADIO

#define RADIO_MAX_CH       HW_RADIO_MAX_CH
#define RADIO_BASE_ADDR    HW_RADIO_BASE_ADDR



bool radioInit(void);



#endif


#ifdef __cplusplus
}
#endif


#endif /* SRC_COMMON_HW_INCLUDE_RADIO_H_ */
