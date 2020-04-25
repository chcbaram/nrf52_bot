/*
 * HCINrfTransport.cpp
 *
 *  Created on: 2020. 4. 25.
 *      Author: Baram
 */




#include "HCINrfTransport.h"



#define HCI_COMMAND_PKT 0x01
#define HCI_ACLDATA_PKT 0x02
#define HCI_EVENT_PKT   0x04



#ifdef _USE_HW_RTOS
static osMutexId mutex_id;
#endif

static uint8_t  msg_buf[HCI_BUF_MAX];
static uint8_t  rx_buf[HCI_BUF_MAX];
static qbuffer_t qbuffer_rx;


HCINrfTransportClass::HCINrfTransportClass()
{
#ifdef _USE_HW_RTOS
  osMutexDef(mutex_id);
  mutex_id = osMutexCreate (osMutex(mutex_id));
#endif

  is_begin = false;
}

HCINrfTransportClass::~HCINrfTransportClass()
{
}

int HCINrfTransportClass::begin()
{
  qbufferCreate(&qbuffer_rx, rx_buf, HCI_BUF_MAX);

  is_begin = true;
  return 1;
}

void HCINrfTransportClass::end()
{
  is_begin = false;
}

void HCINrfTransportClass::wait(unsigned long timeout)
{
  for (unsigned long start = millis(); (millis() - start) < timeout;) {
    if (available()) {
      break;
    }
  }
}

int HCINrfTransportClass::available()
{
  return (int)qbufferAvailable(&qbuffer_rx);;
}

int HCINrfTransportClass::peek()
{
  return qbuffer_rx.ptr_out;
}

int HCINrfTransportClass::read()
{
  uint8_t data = 0;
  int ret = 0;

  qbufferRead(&qbuffer_rx, &data, 1);

  ret = data & 0xFF;

  return ret;
}

size_t HCINrfTransportClass::write(const uint8_t* data, size_t length)
{
#ifdef _USE_HW_RTOS
  osMutexWait(mutex_id, osWaitForever);
#endif

  if (data[0] == HCI_COMMAND_PKT)
  {
    hci_cmd_put(&data[1]);
    logPrintf("cmd\n");
  }
  if (data[0] == HCI_ACLDATA_PKT)
  {
    hci_data_put(&data[1]);
    logPrintf("acl\n");
  }

  logPrintf("write %X, %02X%02X, %d, %d %d\n", data[0], data[2], data[1], data[3], data[4], data[5]);

#ifdef _USE_HW_RTOS
  osMutexRelease(mutex_id);
#endif
  return length;
}


HCINrfTransportClass HCINrfTransport;
HCITransportInterface& HCITransport = HCINrfTransport;




void HCIUpdate(void)
{
  uint8_t  data;
  uint16_t len;

#ifdef _USE_HW_RTOS
  osMutexWait(mutex_id, osWaitForever);
#endif

  if (hci_evt_get(msg_buf) == 0)
  {
    data = HCI_EVENT_PKT;
    qbufferWrite(&qbuffer_rx, &data, 1);

    len = msg_buf[1] + 2;

    qbufferWrite(&qbuffer_rx, msg_buf, len);
    //logPrintf("EVT 0x%X, %d\n", msg_buf[0], len);
  }
  else if (hci_data_get(msg_buf) == 0)
  {
    data = HCI_ACLDATA_PKT;
    qbufferWrite(&qbuffer_rx, &data, 1);

    len = ((msg_buf[2]<<8) | (msg_buf[2]<<0)) + 4;

    qbufferWrite(&qbuffer_rx, msg_buf, len);
    //logPrintf("ASC 0x%X\n", msg_buf[0]);
  }

#ifdef _USE_HW_RTOS
  osMutexRelease(mutex_id);
#endif
}

