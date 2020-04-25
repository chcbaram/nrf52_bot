/*
 * HCINrfTransport.h
 *
 *  Created on: 2020. 4. 25.
 *      Author: Baram
 */

#ifndef SRC_LIB_ARDUINOBLE_UTILITY_HCINRFTRANSPORT_H_
#define SRC_LIB_ARDUINOBLE_UTILITY_HCINRFTRANSPORT_H_


#include "HCITransport.h"
#include "qbuffer.h"


#define HCI_BUF_MAX     (HCI_MSG_BUFFER_MAX_SIZE+10)


class HCINrfTransportClass : public HCITransportInterface {
public:
  HCINrfTransportClass();
  virtual ~HCINrfTransportClass();

  virtual int begin();
  virtual void end();

  virtual void wait(unsigned long timeout);

  virtual int available();
  virtual int peek();
  virtual int read();

  virtual size_t write(const uint8_t* data, size_t length);

private:
  bool is_begin;
};



#endif /* SRC_LIB_ARDUINOBLE_UTILITY_HCINRFTRANSPORT_H_ */
