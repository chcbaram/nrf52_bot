/*
 * radio.c
 *
 *  Created on: 2020. 3. 28.
 *      Author: Baram
 */





#include "radio.h"
#include "swtimer.h"
#include "cmdif.h"

#include "nrf_radio.h"


#ifdef _USE_HW_CMDIF
void radioCmdif(void);
#endif




// S0     - 1B
// LENGTH - 1B
// S1     - 1B
// PAYLOAD- 200B

const nrf_radio_packet_conf_t nrf_packet_config =
    {
        .lflen  = 8,    /**< Length on air of LENGTH field in number of bits. */
        .s0len  = 1,    /**< Length on air of S0 field in number of bytes. */
        .s1len  = 8,    /**< Length on air of S1 field in number of bits. */
        .s1incl = true, /**< Include or exclude S1 field in RAM. */
        .cilen  = 0,    /**< Length of code indicator - long range. */
        .plen   = NRF_RADIO_PREAMBLE_LENGTH_8BIT,
        .crcinc = false,
        .termlen = 0,
        .maxlen = RADIO_PAYLOAD_LEN+1,
        .statlen = 0,
        .balen   = 4, // Base address length
        .big_endian = false,
        .whiteen = false,
    };


static bool is_init = false;
static bool is_open = false;


static radio_packet_t rx_packet;
static radio_packet_t tx_packet;



static char *radioInfo_MODE(char *str);
static char *radioInfo_STATE(char *str);

uint32_t radioGetPrefixAddr(uint8_t index);
uint32_t radioGetBaseAddr(uint8_t index);
void     radioSetPrefixAddr(uint8_t index, uint8_t data);
void     radioSetBaseAddr(uint8_t index, uint32_t data);
void     radioSetRxAddrEnable(uint8_t index, bool enable);
bool     radioGetRxAddrEnable(uint8_t index);
bool     radioSetReadyForRx(uint32_t timeout_ms);
bool     radioSetReadyForTx(uint32_t timeout_ms);
bool     radioSetStart(void);
bool     radioSetStop(void);



bool radioInit(void)
{
  uint32_t i;


  for (i=0; i<8; i++)
  {
    radioSetBaseAddr(i, RADIO_BASE_ADDR);
    radioSetPrefixAddr(i, i);
  }

  nrf_radio_packet_configure(NRF_RADIO, &nrf_packet_config);

  nrf_radio_crcinit_set(NRF_RADIO, 0);
  nrf_radio_crc_configure(NRF_RADIO, 2, 0, 0x11021);


  radioSetRxAddrEnable(0, true);



  nrf_radio_packetptr_set(NRF_RADIO, &rx_packet);


#ifdef _USE_HW_CMDIF
  cmdifAdd("radio", radioCmdif);
#endif

  is_init = true;

  return true;
}

bool radioOpen(radio_baud_t baud, uint32_t addr)
{
  uint32_t i;


  for (i=0; i<8; i++)
  {
    radioSetBaseAddr(i, addr);
  }


  switch(baud)
  {
    case RADIO_BAUD_1M:
      nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_NRF_1MBIT);
      break;

    case RADIO_BAUD_2M:
      nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_NRF_2MBIT);
      break;

    default:
      nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_NRF_1MBIT);
      break;
  }

  is_open = true;

  return true;
}

bool radioEnable(radio_ch_t ch)
{
  // TODO
  return true;
}

int32_t radioAvailable(radio_ch_t ch)
{
  // TODO
  return 0;
}

bool radioRead(radio_ch_t ch, radio_packet_t *p_packet)
{
  // TODO
  return true;
}

bool radioWrite(radio_ch_t ch, radio_packet_t *p_packet, uint32_t timeout_ms)
{
  bool ret = true;


  radioSetReadyForTx(100);

  p_packet->ID = ch;
  p_packet->TYPE = RADIO_TYPE_BIN;

  memcpy(&tx_packet, p_packet, sizeof(radio_packet_t));

  nrf_radio_packetptr_set(NRF_RADIO, &tx_packet);
  radioSetStart();

  return ret;
}

bool radioPrintf(uint8_t channel, const char *fmt, ...)
{
  bool ret = true;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;
  radio_packet_t *p_packet = &tx_packet;


  len = vsnprintf((char *)p_packet->PAYLOAD, RADIO_PAYLOAD_LEN + 1, fmt, arg);
  va_end (arg);

  p_packet->ID =  channel;
  p_packet->TYPE = RADIO_TYPE_STR;
  p_packet->LENGTH = len;

  nrf_radio_packetptr_set(NRF_RADIO, p_packet);
  radioSetReadyForTx(100);
  radioSetStart();

  return ret;
}






//-- Internal Function
//
uint32_t radioGetPrefixAddr(uint8_t index)
{
  uint32_t ret = 0;


  if (index >= 0 && index <= 3)
  {
    ret = nrf_radio_prefix0_get(NRF_RADIO);
    ret = (ret >> (8*index)) & (0xFF);
  }
  else
  {
    ret = nrf_radio_prefix1_get(NRF_RADIO);

    index = index - 4;
    ret = (ret >> (8*index)) & (0xFF);
  }

  return ret;
}

void radioSetPrefixAddr(uint8_t index, uint8_t data)
{
  uint32_t reg;


  if (index >= 0 && index <= 3)
  {
    reg = nrf_radio_prefix0_get(NRF_RADIO);
    reg &= ~(0xFF<<(8*index));
    reg |=  (data<<(8*index));
    nrf_radio_prefix0_set(NRF_RADIO, reg);
  }
  else
  {
    index = index - 4;
    reg = nrf_radio_prefix1_get(NRF_RADIO);
    reg &= ~(0xFF<<(8*index));
    reg |=  (data<<(8*index));
    nrf_radio_prefix1_set(NRF_RADIO, reg);
  }
}

uint32_t radioGetBaseAddr(uint8_t index)
{
  uint32_t ret = 0;


  if (index == 0)
  {
    ret = nrf_radio_base0_get(NRF_RADIO);
  }
  else
  {
    ret = nrf_radio_base1_get(NRF_RADIO);
  }

  return ret;
}

void radioSetBaseAddr(uint8_t index, uint32_t data)
{
  if (index == 0)
  {
    nrf_radio_base0_set(NRF_RADIO, data);
  }
  else
  {
    nrf_radio_base1_set(NRF_RADIO, data);
  }
}

void radioSetRxAddrEnable(uint8_t index, bool enable)
{
  uint32_t reg;

  if (index >= 8) return;


  reg = nrf_radio_rxaddresses_get(NRF_RADIO);

  reg &= ~(1<<index);
  reg |= (enable<<index);

  nrf_radio_rxaddresses_set(NRF_RADIO, reg);
}

bool radioGetRxAddrEnable(uint8_t index)
{
  uint32_t reg;

  if (index >= 8) return false;


  reg = nrf_radio_rxaddresses_get(NRF_RADIO);

  if ((reg & (1<<index)) > 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool radioSetReadyForRx(uint32_t timeout_ms)
{
  bool ret = true;
  uint32_t pre_time;
  nrf_radio_state_t goal_state = NRF_RADIO_STATE_RXIDLE;
  nrf_radio_state_t cur_state;

  pre_time = millis();
  while(1)
  {
    cur_state = nrf_radio_state_get(NRF_RADIO);

    switch(cur_state)
    {
      case NRF_RADIO_STATE_DISABLED:
        NRF_RADIO->TASKS_RXEN = 1;
        break;

      case NRF_RADIO_STATE_RXIDLE:
        break;

      case NRF_RADIO_STATE_RX:
        break;

      case NRF_RADIO_STATE_TXIDLE:
        NRF_RADIO->TASKS_DISABLE = 1;
        break;

      case NRF_RADIO_STATE_TX:
        NRF_RADIO->TASKS_DISABLE = 1;
        break;

      default:
        break;
    }

    if (cur_state == goal_state)
    {
      break;
    }

    if (millis()-pre_time >= timeout_ms)
    {
      ret = false;
      break;
    }
  }

  return ret;
}

bool radioSetReadyForTx(uint32_t timeout_ms)
{
  bool ret = true;
  uint32_t pre_time;
  nrf_radio_state_t goal_state = NRF_RADIO_STATE_TXIDLE;
  nrf_radio_state_t cur_state;

  pre_time = millis();
  while(1)
  {
    cur_state = nrf_radio_state_get(NRF_RADIO);

    switch(cur_state)
    {
      case NRF_RADIO_STATE_DISABLED:
        NRF_RADIO->TASKS_TXEN = 1;
        break;

      case NRF_RADIO_STATE_RXIDLE:
        NRF_RADIO->TASKS_DISABLE = 1;
        break;

      case NRF_RADIO_STATE_RX:
        NRF_RADIO->TASKS_DISABLE = 1;
        break;

      case NRF_RADIO_STATE_TXIDLE:
        break;

      case NRF_RADIO_STATE_TX:
        break;

      default:
        break;
    }

    if (cur_state == goal_state)
    {
      break;
    }

    if (millis()-pre_time >= timeout_ms)
    {
      ret = false;
      break;
    }
  }

  return ret;
}

bool radioSetStart(void)
{
  NRF_RADIO->TASKS_START = 1;
  return true;
}

bool radioSetStop(void)
{
  NRF_RADIO->TASKS_STOP = 1;
  return true;
}


bool radioInfo(void)
{
  char str_buf[128];

  NRF_RADIO->TASKS_START = 1;

  logPrintf("TXPOWER    : %d dBm\n", nrf_radio_txpower_get(NRF_RADIO));
  logPrintf("FREQ       : %d Mhz\n", nrf_radio_frequency_get(NRF_RADIO));
  logPrintf("MODE       : %s\n", radioInfo_MODE(str_buf));
  logPrintf("STATE      : %s\n", radioInfo_STATE(str_buf));

  for (int i=0; i<8; i++)
  {
    logPrintf("ADDR       : 0x%02X 0x%08X \n", radioGetPrefixAddr(i), radioGetBaseAddr(i));
  }
  logPrintf("TX ADDR    : %d\n", nrf_radio_txaddress_get(NRF_RADIO));

  for (int i=0; i<8; i++)
  {
    logPrintf("RX ADDR%d   : %d\n", i, radioGetRxAddrEnable(i));
  }

  return true;
}

char *radioInfo_MODE(char *str)
{
  nrf_radio_mode_t mode = nrf_radio_mode_get(NRF_RADIO);

  str[0] = 0;

  if (mode == NRF_RADIO_MODE_NRF_1MBIT) strcpy(str, "NRF_RADIO_MODE_NRF_1MBIT");
  if (mode == NRF_RADIO_MODE_NRF_2MBIT) strcpy(str, "NRF_RADIO_MODE_NRF_2MBIT");
  if (mode == NRF_RADIO_MODE_BLE_1MBIT) strcpy(str, "NRF_RADIO_MODE_BLE_1MBIT");
  if (mode == NRF_RADIO_MODE_BLE_2MBIT) strcpy(str, "NRF_RADIO_MODE_BLE_2MBIT");
  if (mode == NRF_RADIO_MODE_BLE_LR125KBIT) strcpy(str, "NRF_RADIO_MODE_BLE_LR125KBIT");
  if (mode == NRF_RADIO_MODE_BLE_LR500KBIT) strcpy(str, "NRF_RADIO_MODE_BLE_LR500KBIT");
  if (mode == NRF_RADIO_MODE_IEEE802154_250KBIT) strcpy(str, "NRF_RADIO_MODE_IEEE802154_250KBIT");

  return str;
}

char *radioInfo_STATE(char *str)
{
  nrf_radio_state_t mode = nrf_radio_state_get(NRF_RADIO);

  str[0] = 0;

  str[0] = 0;

  if (mode == NRF_RADIO_STATE_DISABLED) strcpy(str, "NRF_RADIO_STATE_DISABLED");
  if (mode == NRF_RADIO_STATE_RXRU) strcpy(str, "NRF_RADIO_STATE_RXRU");
  if (mode == NRF_RADIO_STATE_RXIDLE) strcpy(str, "NRF_RADIO_STATE_RXIDLE");
  if (mode == NRF_RADIO_STATE_RX) strcpy(str, "NRF_RADIO_STATE_RX");
  if (mode == NRF_RADIO_STATE_RXDISABLE) strcpy(str, "NRF_RADIO_STATE_RXDISABLE");
  if (mode == NRF_RADIO_STATE_TXRU) strcpy(str, "NRF_RADIO_STATE_TXRU");
  if (mode == NRF_RADIO_STATE_TXIDLE) strcpy(str, "NRF_RADIO_STATE_TXIDLE");
  if (mode == NRF_RADIO_STATE_TX) strcpy(str, "NRF_RADIO_STATE_TX");
  if (mode == NRF_RADIO_STATE_TXDISABLE) strcpy(str, "NRF_RADIO_STATE_TXDISABLE");


  return str;
}

#ifdef _USE_HW_CMDIF
void radioCmdif(void)
{
  bool ret = true;


  if (cmdifGetParamCnt() == 1)
  {
    if(cmdifHasString("info", 0) == true)
    {
      radioInfo();
    }
    else if (cmdifHasString("rx_ready", 0) == true)
    {
      uint32_t pre_time;

      pre_time = micros();
      radioSetReadyForRx(1000);
      cmdifPrintf("%d us\n", micros()-pre_time);
    }
    else if (cmdifHasString("tx_ready", 0) == true)
    {
      uint32_t pre_time;

      pre_time = micros();
      radioSetReadyForTx(1000);
      cmdifPrintf("%d us\n", micros()-pre_time);
    }
    else if (cmdifHasString("rx", 0) == true)
    {
      NRF_RADIO->EVENTS_READY = 0;
      NRF_RADIO->EVENTS_END = 0;
      NRF_RADIO->EVENTS_PAYLOAD = 0;
      nrf_radio_packetptr_set(NRF_RADIO, &rx_packet);
      radioSetReadyForRx(100);
      radioSetStart();

      uint32_t pre_time;
      uint32_t cnt_sum = 0;

      pre_time = millis();
      cnt_sum = 0;
      while(cmdifRxAvailable() == 0)
      {
        //cmdifPrintf("R %d E %d P %d\n", NRF_RADIO->EVENTS_READY, NRF_RADIO->EVENTS_END, NRF_RADIO->EVENTS_PAYLOAD);

        if (NRF_RADIO->EVENTS_END)
        {
          NRF_RADIO->EVENTS_READY   = 0;
          NRF_RADIO->EVENTS_END     = 0;
          NRF_RADIO->EVENTS_PAYLOAD = 0;

          if (NRF_RADIO->CRCSTATUS)
          {
            /*
            cmdifPrintf("S0 %X, S1 %X, L %X, PAYLOAD %d, %d, %d\n",
                        rx_packet.S0,
                        rx_packet.S1,
                        rx_packet.LENGTH,
                        rx_packet.PAYLOAD[0],
                        rx_packet.PAYLOAD[1],
                        NRF_RADIO->CRCSTATUS);
                        */
            cnt_sum += rx_packet.LENGTH;
          }
          radioSetStart();
        }

        if (millis()-pre_time >= 1000)
        {
          pre_time = millis();

          cmdifPrintf("%d bytes\n", cnt_sum);

          cnt_sum = 0;
        }
      }
    }
    else if (cmdifHasString("tx", 0) == true)
    {
      uint8_t cnt = 0;

      radioSetReadyForTx(100);
      nrf_radio_packetptr_set(NRF_RADIO, &tx_packet);

      while(1)
      {
        uint8_t ch;

        if (cmdifRxAvailable() > 0)
        {
          ch = cmdifGetch();

          if (ch == '1')
          {
            tx_packet.ID =  0x55;
            tx_packet.TYPE = ~0x55;
            tx_packet.LENGTH = 128;
            tx_packet.PAYLOAD[0] = cnt++;
            tx_packet.PAYLOAD[1] = cnt++;

            radioSetStart();

            cmdifPrintf("R %d E %d\n", NRF_RADIO->EVENTS_READY, NRF_RADIO->EVENTS_END);
          }
          else
          {
            break;
          }
        }
      }
    }
    else if (cmdifHasString("txs", 0) == true)
    {
      uint8_t cnt = 0;

      radioSetReadyForTx(100);
      nrf_radio_packetptr_set(NRF_RADIO, &tx_packet);

      while(cmdifRxAvailable() == 0)
      {
        tx_packet.ID =  0x55;
        tx_packet.TYPE = ~0x55;
        tx_packet.LENGTH = 200;
        tx_packet.PAYLOAD[0] = cnt++;
        tx_packet.PAYLOAD[1] = cnt++;

        radioSetReadyForTx(100);
        radioSetStart();
      }
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }


  if (ret == false)
  {
    cmdifPrintf( "radio info\n");
    cmdifPrintf( "radio rx_ready\n");
    cmdifPrintf( "radio tx_ready\n");
  }
}
#endif
