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






typedef struct
{
  uint32_t length;
  uint32_t in;
  uint32_t out;

  radio_packet_t buf[RADIO_MAX_BUF_LEN];
} radio_buf_t;



// S0     - 1B
// LENGTH - 1B
// S1     - 2B
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

volatile bool is_tx_done = true;

static radio_packet_t rx_packet;
static radio_packet_t tx_packet;

static radio_buf_t *rx_buf[RADIO_MAX_CH];



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

    rx_buf[i] = NULL;
  }

  nrf_radio_packet_configure(NRF_RADIO, &nrf_packet_config);
  nrf_radio_crcinit_set(NRF_RADIO, 0);
  nrf_radio_crc_configure(NRF_RADIO, 2, 0, 0x11021);

  nrf_radio_int_enable(NRF_RADIO, RADIO_INTENSET_END_Msk);

  radioEnable(RADIO_CH_0);


  nrf_radio_packetptr_set(NRF_RADIO, &rx_packet);
  radioSetReadyForRx(10);
  radioSetStart();


  NVIC_SetPriority(RADIO_IRQn, 6);
  NVIC_EnableIRQ(RADIO_IRQn);


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
  if (rx_buf[ch] == NULL)
  {
    rx_buf[ch] = malloc(sizeof(radio_buf_t));

    rx_buf[ch]->length = RADIO_MAX_BUF_LEN;
    rx_buf[ch]->in = 0;
    rx_buf[ch]->out = 0;

    radioSetRxAddrEnable(ch, true);

    nrf_radio_packetptr_set(NRF_RADIO, &rx_packet);
    radioSetReadyForRx(10);
    radioSetStart();
  }
  return true;
}

int32_t radioAvailable(radio_ch_t ch)
{
  int32_t ret = 0;
  radio_buf_t *p_node = rx_buf[ch];

  if (p_node != NULL)
  {
    ret = (p_node->length + p_node->in - p_node->out) % p_node->length;
  }
  return ret;
}

bool radioRead(radio_ch_t ch, radio_packet_t *p_packet)
{
  bool ret = true;
  radio_buf_t *p_node = rx_buf[ch];

  if (p_node != NULL && p_node->out != p_node->in)
  {
    memcpy(p_packet, &p_node->buf[p_node->out], sizeof(radio_packet_t));
    p_node->out = (p_node->out + 1) % p_node->length;
  }
  else
  {
    ret = false;
  }

  return ret;
}

bool radioWrite(radio_ch_t ch, radio_packet_t *p_packet, uint32_t timeout_ms)
{
  bool ret = false;
  uint32_t pre_time;

  radioSetReadyForTx(10);

  is_tx_done = false;
  nrf_radio_txaddress_set(NRF_RADIO, ch);
  nrf_radio_packetptr_set(NRF_RADIO, p_packet);
  radioSetStart();

  pre_time = millis();
  while(millis()-pre_time < timeout_ms)
  {
    if (is_tx_done == true)
    {
      ret = true;
      is_tx_done = false;
      break;
    }
  }

  nrf_radio_packetptr_set(NRF_RADIO, &rx_packet);
  radioSetReadyForRx(10);
  radioSetStart();

  return ret;
}

bool radioPrintf(radio_ch_t ch, const char *fmt, ...)
{
  bool ret = true;
  va_list arg;
  va_start (arg, fmt);
  int32_t len;
  radio_packet_t *p_packet = &tx_packet;


  len = vsnprintf((char *)p_packet->PAYLOAD, RADIO_PAYLOAD_LEN + 1, fmt, arg);
  va_end (arg);

  p_packet->ID     = ch;
  p_packet->TYPE   = RADIO_TYPE_STR;
  p_packet->LENGTH = len;


  ret = radioWrite(ch, p_packet, 100);

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
        NRF_RADIO->TASKS_STOP = 1;
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
        NRF_RADIO->TASKS_STOP = 1;
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




void RADIO_IRQHandler(void)
{
  radio_buf_t *p_node;
  uint32_t next_i;
  nrf_radio_state_t state = nrf_radio_state_get(NRF_RADIO);


  if (NRF_RADIO->EVENTS_END)
  {
    NRF_RADIO->EVENTS_END = 0;

    if (state == NRF_RADIO_STATE_TX || state == NRF_RADIO_STATE_TXIDLE)
    {
      is_tx_done = true;
    }
    else
    {
      if (NRF_RADIO->CRCSTATUS)
      {
        p_node = rx_buf[NRF_RADIO->RXMATCH];

        if (p_node != NULL)
        {
          next_i = (p_node->in + 1) % p_node->length;

          if (next_i != p_node->out)
          {
            memcpy(&p_node->buf[p_node->in], &rx_packet, sizeof(radio_packet_t));
            p_node->in = next_i;
          }
        }
      }

      nrf_radio_packetptr_set(NRF_RADIO, &rx_packet);
      NRF_RADIO->TASKS_START = 1;
    }
  }
}


#ifdef _USE_HW_CMDIF
void radioCmdif(void)
{
  bool ret = true;
  radio_packet_t packet;


  if (cmdifGetParamCnt() == 1)
  {
    if(cmdifHasString("info", 0) == true)
    {
      radioInfo();
    }
    else if (cmdifHasString("echo", 0) == true)
    {
      uint32_t cnt = 0;
      uint8_t ch;

      radioEnable(RADIO_CH_1);

      while(1)
      {
        if (cmdifRxAvailable() > 0)
        {
          ch = cmdifGetch();

          if (ch == '1')
          {
            radioPrintf(RADIO_CH_0, "This is Radio %d\n", cnt);
            cmdifPrintf("CH0 TX -> This is Radio %d\n", cnt);
            cnt++;

            uint32_t pre_time = millis();
            while(millis()-pre_time < 100)
            {
              if (radioAvailable(RADIO_CH_0) > 0)
              {
                radioRead(RADIO_CH_0, &packet);
                if (packet.TYPE == RADIO_TYPE_STR)
                {
                  cmdifPrintf("CH0 RX -> %s", packet.PAYLOAD);
                }
                break;
              }
            }
          }
          else if (ch == '2')
          {
            radioPrintf(RADIO_CH_1, "This is Radio %d\n", cnt);
            cmdifPrintf("CH1 TX -> This is Radio %d\n", cnt);
            cnt++;
          }
          else
          {
            break;
          }
        }
        if (radioAvailable(RADIO_CH_0) > 0)
        {
          radioRead(RADIO_CH_0, &packet);
          if (packet.TYPE == RADIO_TYPE_STR)
          {
            cmdifPrintf("CH0 RX -> %s", packet.PAYLOAD);
            radioPrintf(RADIO_CH_0, "%s", packet.PAYLOAD);
          }
        }
        if (radioAvailable(RADIO_CH_1) > 0)
        {
          radioRead(RADIO_CH_1, &packet);
          if (packet.TYPE == RADIO_TYPE_STR)
          {
            cmdifPrintf("CH1 RX -> %s", packet.PAYLOAD);
          }
        }
      }
    }
    else if (cmdifHasString("rxd", 0) == true)
    {
      while(cmdifRxAvailable() == 0)
      {
        if (radioAvailable(RADIO_CH_0) > 0)
        {
          radioRead(RADIO_CH_0, &packet);
          cmdifPrintf("I%X,T%X, L %X, PAYLOAD %d, %d\n",
                      packet.ID,
                      packet.TYPE,
                      packet.LENGTH,
                      packet.PAYLOAD[0],
                      packet.PAYLOAD[1]);

        }
      }
    }
    else if (cmdifHasString("txd", 0) == true)
    {
      uint8_t cnt = 0;

      while(1)
      {
        uint8_t ch;

        if (cmdifRxAvailable() > 0)
        {
          ch = cmdifGetch();

          if (ch == '1')
          {
            tx_packet.ID =  0;
            tx_packet.TYPE = 1;
            tx_packet.LENGTH = 1;
            tx_packet.PAYLOAD[0] = cnt++;
            tx_packet.PAYLOAD[1] = cnt;

            if (radioWrite(RADIO_CH_0, &tx_packet, 50) == true)
            {
              cmdifPrintf("Tx OK\n");
            }
            else
            {
              cmdifPrintf("Tx Fail\n");
            }
          }
          else
          {
            break;
          }
        }
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
