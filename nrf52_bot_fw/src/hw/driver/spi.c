/*
 * spi.c
 *
 *  Created on: 2020. 1. 30.
 *      Author: Baram
 */





#include "spi.h"




#define SPI_TX_DMA_MAX_LENGTH   0xEFFF
#define SPI_DATA_8BIT           0
#define SPI_DATA_16BIT          1


typedef struct
{
  bool tx_done;
  uint8_t *p_tx_buf;
  uint8_t *p_tx_buf_next;
  uint32_t tx_length_next;
} spi_dma_buf_t;

typedef struct
{
  bool               is_open;
  bool               is_dma_init;
  bool               is_refresh;

  uint8_t            data_size;
  NRF_SPIM_Type     *h_spi;
  spi_dma_buf_t      dma_tx_buf;

  void              (*func_tx)(void);
} spi_t;





static spi_t spi_tbl[SPI_MAX_CH];



bool spiInit(void)
{
  uint8_t i;


  for(i=0; i<SPI_MAX_CH; i++)
  {
    spi_tbl[i].is_open = false;
    spi_tbl[i].is_dma_init = false;
    spi_tbl[i].is_refresh = false;
    spi_tbl[i].func_tx    = NULL;
    spi_tbl[i].data_size  = SPI_DATA_8BIT;

    spi_tbl[i].dma_tx_buf.p_tx_buf         = NULL;
    spi_tbl[i].dma_tx_buf.p_tx_buf_next    = NULL;
    spi_tbl[i].dma_tx_buf.tx_done          = false;
    spi_tbl[i].dma_tx_buf.tx_length_next   = 0;
  }

  return true;
}


void spiBegin(uint8_t spi_ch)
{
  spi_t  *p_spi;


  switch(spi_ch)
  {
    case _DEF_SPI1:
      p_spi = &spi_tbl[spi_ch];

      p_spi->h_spi = NRF_SPIM3;


      p_spi->h_spi->ENABLE = SPIM_ENABLE_ENABLE_Disabled;
      p_spi->h_spi->CSNPOL = (0<<SPIM_PSEL_CSN_PIN_Pos);  // Active Low
      p_spi->h_spi->DCXCNT = 0;

      p_spi->h_spi->PSEL.SCK  = NRF_GPIO_PIN_MAP(1,  4);
      p_spi->h_spi->PSEL.MOSI = NRF_GPIO_PIN_MAP(1,  5);
      p_spi->h_spi->PSEL.MISO = 0;
      p_spi->h_spi->PSEL.CSN  = NRF_GPIO_PIN_MAP(1,  8);
      p_spi->h_spi->PSELDCX   = NRF_GPIO_PIN_MAP(1,  6);

      nrf_gpio_cfg(
          p_spi->h_spi->PSEL.SCK,
          NRF_GPIO_PIN_DIR_OUTPUT,
          NRF_GPIO_PIN_INPUT_DISCONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_H0H1,
          NRF_GPIO_PIN_NOSENSE);

      nrf_gpio_cfg(
          p_spi->h_spi->PSEL.MOSI,
          NRF_GPIO_PIN_DIR_OUTPUT,
          NRF_GPIO_PIN_INPUT_DISCONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_H0H1,
          NRF_GPIO_PIN_NOSENSE);

      nrf_gpio_cfg(
          p_spi->h_spi->PSEL.CSN,
          NRF_GPIO_PIN_DIR_OUTPUT,
          NRF_GPIO_PIN_INPUT_DISCONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_H0H1,
          NRF_GPIO_PIN_NOSENSE);

      nrf_gpio_cfg(
          p_spi->h_spi->PSELDCX,
          NRF_GPIO_PIN_DIR_OUTPUT,
          NRF_GPIO_PIN_INPUT_DISCONNECT,
          NRF_GPIO_PIN_NOPULL,
          NRF_GPIO_PIN_H0H1,
          NRF_GPIO_PIN_NOSENSE);

      p_spi->h_spi->IFTIMING.CSNDUR = 0; // n * 15.625ns
      p_spi->h_spi->ORC = 0;

      p_spi->h_spi->INTENCLR =
                SPIM_INTENCLR_STARTED_Msk
              | SPIM_INTENCLR_ENDTX_Msk
              | SPIM_INTENCLR_END_Msk
              | SPIM_INTENCLR_ENDRX_Msk
              | SPIM_INTENCLR_STOPPED_Msk;

      p_spi->h_spi->INTENSET = SPIM_INTENSET_END_Msk;

      spiSetBitOrder(spi_ch, SPI_BIT_MSB);
      spiSetClockDivider(spi_ch, SPI_DIV_1);

      p_spi->is_dma_init = true;
      p_spi->h_spi->ENABLE = SPIM_ENABLE_ENABLE_Enabled;

      NVIC_SetPriority(SPIM3_IRQn, 5);
      NVIC_EnableIRQ(SPIM3_IRQn);

      p_spi->is_open = true;
      break;
  }
}

void spiSetDCX(uint8_t spi_ch, uint32_t length)
{
  spi_tbl[spi_ch].h_spi->DCXCNT = length;
}

void spiTransfer(uint8_t spi_ch, uint8_t *p_tx_data, uint8_t *p_rx_data, uint32_t length)
{

  spiDmaTransfer(spi_ch, p_tx_data, length, 100);

  return;
}


uint8_t spiTransfer8(uint8_t spi_ch, uint8_t data)
{
  uint8_t ret = 0;
  spi_t  *p_spi = &spi_tbl[spi_ch];


  if (p_spi->is_open == false) return 0;


  spiDmaTransfer(spi_ch, &data, 1, 50);

  return ret;
}


uint16_t spiTransfer16(uint8_t spi_ch, uint16_t data)
{
  uint8_t tBuf[2];
  //uint8_t rBuf[2];
  uint16_t ret;
  spi_t  *p_spi = &spi_tbl[spi_ch];


  if (p_spi->is_open == false) return 0;

  if (p_spi->data_size == SPI_DATA_8BIT)
  {
    tBuf[1] = (uint8_t)data;
    tBuf[0] = (uint8_t)(data>>8);

    spiDmaTransfer(spi_ch, tBuf, 2, 50);

    //ret = rBuf[0];
    //ret <<= 8;
    //ret += rBuf[1];
    ret = 0;
  }
  else
  {
    spiDmaTransfer(spi_ch, (uint8_t *)&data, 2, 50);
  }
  return ret;
}


void spiSetBitOrder(uint8_t spi_ch, uint8_t bitOrder)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];


  if (p_spi->is_open == false) return;

  if (bitOrder == SPI_BIT_LSB)
  {
    p_spi->h_spi->CONFIG |=  (1<<SPIM_CONFIG_ORDER_Pos);
  }
  else
  {
    p_spi->h_spi->CONFIG &= ~(1<<SPIM_CONFIG_ORDER_Pos);
  }
}


void spiSetClockDivider(uint8_t spi_ch, uint32_t clockDiv)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];


  switch(clockDiv)
  {
    case SPI_DIV_1:
      p_spi->h_spi->FREQUENCY = 0x14000000;   // 32Mhz
      break;
    case SPI_DIV_2:
      p_spi->h_spi->FREQUENCY = 0x0A000000;   // 16Mhz
      break;
    case SPI_DIV_4:
      p_spi->h_spi->FREQUENCY = 0x80000000;   // 8Mhz
      break;
    case SPI_DIV_8:
      p_spi->h_spi->FREQUENCY = 0x40000000;   // 4Mhz
      break;
    case SPI_DIV_16:
      p_spi->h_spi->FREQUENCY = 0x20000000;   // 2Mhz
      break;
    case SPI_DIV_32:
      p_spi->h_spi->FREQUENCY = 0x10000000;   // 1Mhz
      break;
    case SPI_DIV_64:
      p_spi->h_spi->FREQUENCY = 0x08000000;   // 500Khz
      break;
    case SPI_DIV_128:
      p_spi->h_spi->FREQUENCY = 0x04000000;   // 250Khz
      break;

    default:
      p_spi->h_spi->FREQUENCY = 0x02000000;   // 125Khz
      break;
  }
}


void spiSetDataMode(uint8_t spi_ch, uint8_t dataMode)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];
  uint32_t reg_data;

  if (p_spi->is_open == false) return;


  reg_data = p_spi->h_spi->CONFIG & SPIM_CONFIG_ORDER_Msk;

  switch( dataMode )
  {
    // CPOL=0, CPHA=0
    case SPI_MODE0:
      reg_data |= (0<<SPIM_CONFIG_CPOL_Pos) | (0<<SPIM_CONFIG_CPHA_Pos);
      break;

    // CPOL=0, CPHA=1
    case SPI_MODE1:
      reg_data |= (0<<SPIM_CONFIG_CPOL_Pos) | (1<<SPIM_CONFIG_CPHA_Pos);
      break;

    // CPOL=1, CPHA=0
    case SPI_MODE2:
      reg_data |= (1<<SPIM_CONFIG_CPOL_Pos) | (0<<SPIM_CONFIG_CPHA_Pos);
      break;

    // CPOL=1, CPHA=1
    case SPI_MODE3:
      reg_data |= (1<<SPIM_CONFIG_CPOL_Pos) | (1<<SPIM_CONFIG_CPHA_Pos);
      break;
  }

  p_spi->h_spi->CONFIG = reg_data;

}


void spiDmaStartTx(uint8_t spi_ch, uint8_t *p_buf, uint32_t length)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];


  if (p_spi->is_open == false)     return;
  if (p_spi->is_dma_init == false) return;


  if(length > SPI_TX_DMA_MAX_LENGTH)
  {
    p_spi->dma_tx_buf.tx_done       = false;
    p_spi->dma_tx_buf.tx_length_next= length - SPI_TX_DMA_MAX_LENGTH;
    p_spi->dma_tx_buf.p_tx_buf      =  p_buf;
    p_spi->dma_tx_buf.p_tx_buf_next = &p_buf[SPI_TX_DMA_MAX_LENGTH];

    p_spi->h_spi->TXD.MAXCNT  = SPI_TX_DMA_MAX_LENGTH;
    p_spi->h_spi->TXD.PTR     = (uint32_t)p_spi->dma_tx_buf.p_tx_buf;
    p_spi->h_spi->TASKS_START = SPIM_TASKS_START_TASKS_START_Msk;
  }
  else
  {
    p_spi->dma_tx_buf.tx_done       = false;
    p_spi->dma_tx_buf.tx_length_next= 0;
    p_spi->dma_tx_buf.p_tx_buf      = p_buf;
    p_spi->dma_tx_buf.p_tx_buf_next = NULL;

    p_spi->h_spi->TXD.MAXCNT  = length;
    p_spi->h_spi->TXD.PTR     = (uint32_t)p_spi->dma_tx_buf.p_tx_buf;
    p_spi->h_spi->TASKS_START = SPIM_TASKS_START_TASKS_START_Msk;
  }
}


bool spiDmaIsTxDone(uint8_t spi_ch)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];


  if (p_spi->is_open == false)     return true;
  if (p_spi->is_dma_init == false) return true;


  return p_spi->dma_tx_buf.tx_done;
}


void spiDmaSetRefresh(uint8_t spi_ch, bool enable)
{
  spi_t  *p_spi = &spi_tbl[spi_ch];


  if (p_spi->is_open == false)     return;
  if (p_spi->is_dma_init == false) return;


  p_spi->is_refresh = enable;

}

bool spiDmaTransfer(uint8_t spi_ch, void *buf, uint32_t length, uint32_t timeout)
{
  bool ret = true;
  uint32_t t_time;


  spiDmaStartTx(spi_ch, (uint8_t *)buf, length);

  t_time = millis();

  if (timeout == 0) return true;

  while(1)
  {
    if(spiDmaIsTxDone(spi_ch))
    {
      break;
    }
    if((millis()-t_time) > timeout)
    {
      ret = false;
      break;
    }
  }

  return ret;
}

void spiAttachTxInterrupt(uint8_t spi_ch, void (*func)())
{
  spi_t  *p_spi = &spi_tbl[spi_ch];


  if (p_spi->is_open == false)     return;
  if (p_spi->is_dma_init == false) return;


  p_spi->func_tx = func;

}


void spiTxISR(uint8_t spi_ch)
{
  volatile uint16_t length;
  spi_t  *p_spi = &spi_tbl[spi_ch];



  if(p_spi->dma_tx_buf.tx_length_next > 0)
  {
    p_spi->dma_tx_buf.p_tx_buf = p_spi->dma_tx_buf.p_tx_buf_next;

    if(p_spi->dma_tx_buf.tx_length_next > SPI_TX_DMA_MAX_LENGTH)
    {
      length = SPI_TX_DMA_MAX_LENGTH;
      p_spi->dma_tx_buf.tx_length_next = p_spi->dma_tx_buf.tx_length_next - SPI_TX_DMA_MAX_LENGTH;
      p_spi->dma_tx_buf.p_tx_buf_next = &p_spi->dma_tx_buf.p_tx_buf[SPI_TX_DMA_MAX_LENGTH];
    }
    else
    {
      length = p_spi->dma_tx_buf.tx_length_next;
      p_spi->dma_tx_buf.tx_length_next = 0;
      p_spi->dma_tx_buf.p_tx_buf_next = NULL;
    }
    p_spi->h_spi->TXD.MAXCNT  = length;
    p_spi->h_spi->TXD.PTR     = (uint32_t)p_spi->dma_tx_buf.p_tx_buf;
    p_spi->h_spi->TASKS_START = SPIM_TASKS_START_TASKS_START_Msk;
  }
  else
  {
    p_spi->dma_tx_buf.tx_done = true;
    p_spi->h_spi->DCXCNT = 0;

    if (p_spi->func_tx != NULL)
    {
      (*p_spi->func_tx)();
    }
  }
}



void SPIM3_IRQHandler(void)
{
  if (spi_tbl[_DEF_SPI1].h_spi->EVENTS_END)
  {
    spi_tbl[_DEF_SPI1].h_spi->EVENTS_END = 0;

    if (spi_tbl[_DEF_SPI1].h_spi->EVENTS_ENDTX)
    {
      spiTxISR(_DEF_SPI1);
    }

    spi_tbl[_DEF_SPI1].h_spi->EVENTS_ENDRX = 0;
    spi_tbl[_DEF_SPI1].h_spi->EVENTS_ENDTX = 0;
  }
}


