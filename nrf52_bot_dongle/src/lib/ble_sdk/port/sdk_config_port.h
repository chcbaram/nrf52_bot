/*
 * sdk_config_port.h
 *
 *  Created on: 2020. 5. 2.
 *      Author: Baram
 */

#ifndef SRC_LIB_BLE_SDK_PORT_SDK_CONFIG_PORT_H_
#define SRC_LIB_BLE_SDK_PORT_SDK_CONFIG_PORT_H_


#include "hw_def.h"

#ifdef _USE_HW_BLEUART_C
#include "sdk_config_nus_c.h"
#else
#include "sdk_config_nus_p.h"
#endif

#endif /* SRC_LIB_BLE_SDK_PORT_SDK_CONFIG_PORT_H_ */
