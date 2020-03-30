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

#define RADIO_MAX_CH        8
#define RADIO_BASE_ADDR     HW_RADIO_BASE_ADDR


#define RADIO_PAYLOAD_LEN   128


typedef enum
{
  RADIO_CH_BROAD = 0,
  RADIO_CH_1 = 1,
  RADIO_CH_2 = 2,
  RADIO_CH_3 = 3,
  RADIO_CH_4 = 4,
  RADIO_CH_5 = 5,
  RADIO_CH_6 = 6,
  RADIO_CH_7 = 7,
} radio_ch_t;


typedef enum
{
  RADIO_BAUD_1M = 0,
  RADIO_BAUD_2M = 1,
} radio_baud_t;

typedef enum
{
  RADIO_TYPE_BIN = 0,
  RADIO_TYPE_STR = 1,
} radio_type_t;

typedef struct
{
  uint8_t ID;
  uint8_t LENGTH;
  uint8_t TYPE;
  uint8_t PAYLOAD[RADIO_PAYLOAD_LEN+1];
} radio_packet_t;


bool     radioInit(void);
bool     radioOpen(radio_baud_t baud, uint32_t addr);

bool     radioEnable(radio_ch_t ch);
int32_t  radioAvailable(radio_ch_t ch);
bool     radioRead(radio_ch_t ch, radio_packet_t *p_packet);
bool     radioWrite(radio_ch_t ch, radio_packet_t *p_packet, uint32_t timeout_ms);
bool     radioPrintf(radio_ch_t ch, const char *fmt, ...);

#endif


#ifdef __cplusplus
}
#endif


#endif /* SRC_COMMON_HW_INCLUDE_RADIO_H_ */
