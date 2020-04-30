/*
 * rtos.h
 *
 *  Created on: 2020. 1. 29.
 *      Author: Baram
 */

#ifndef SRC_BSP_RTOS_H_
#define SRC_BSP_RTOS_H_


#ifdef __cplusplus
 extern "C" {
#endif

#include "bsp.h"

#define ms2tick              pdMS_TO_TICKS
#define tick2ms(tck)         ( ( ((uint64_t)(tck)) * 1000) / configTICK_RATE_HZ )
#define tick2us(tck)         ( ( ((uint64_t)(tck)) * 1000000) / configTICK_RATE_HZ )

#define malloc_type(type)    rtos_malloc( sizeof(type) )
#define rtos_malloc_type(_type)   (_type*) rtos_malloc(sizeof(_type))

static inline void* rtos_malloc(size_t _size)
{
  return (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? malloc(_size) : pvPortMalloc(_size);
}

static inline void rtos_free( void *pv )
{
  return (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) ? free(pv) : vPortFree(pv);
}

void rtosInit(void);


#ifdef __cplusplus
 }
#endif


#endif /* SRC_BSP_RTOS_H_ */
