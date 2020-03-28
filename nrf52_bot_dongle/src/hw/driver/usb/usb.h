/*
 * usb.h
 *
 *  Created on: 2020. 3. 22.
 *      Author: Baram
 */

#ifndef SRC_HW_DRIVER_USB_USB_H_
#define SRC_HW_DRIVER_USB_USB_H_


#ifdef __cplusplus
extern "C" {
#endif


#include "def.h"
#include "bsp.h"

#include "tusb.h"


bool usbInit(void);
void usbDeInit(void);



#ifdef __cplusplus
}
#endif

#endif /* SRC_HW_DRIVER_USB_USB_H_ */
