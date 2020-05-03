/*
 * hw.c
 *
 *  Created on: 2020. 3. 20.
 *      Author: Baram
 */




#include "hw.h"
#include "opus.h"


#define OPUS_TEST


extern uint32_t __isr_vector_addr;
extern uint32_t _image_start;
extern uint32_t _image_size;



__attribute__((section(".tag"))) const flash_tag_t fw_tag =
   {
    // fw info
    //
    .magic_number = 0xAAAA5555,
    .version_str  = "V200326R1",
    .board_str    = "NRF52_BOT_B/D",
    .name_str     = "Firmware",
    .date_str     = __DATE__,
    .time_str     = __TIME__,
    .addr_tag     = (uint32_t)&fw_tag,
    .addr_fw      = (uint32_t)&__isr_vector_addr,

    .load_start   = (uint32_t)&_image_start,  // load_addr
    .load_size    = (uint32_t)&_image_size,   // load_size
   };


#ifdef OPUS_TEST
#include "sound/wav.h"

void test_failed()
{
  while(1);
}
#endif

void hwInit(void)
{
  bspInit();

  swtimerInit();
  cmdifInit();

  ledInit();
  buttonInit();
  vcpInit();
  uartInit();
  uartOpen(_DEF_UART1, 115200);
  uartOpen(_DEF_UART2, 115200);

  logPrintf("\n\n[ Firmware Begin... ]\r\n");
  logPrintf("Addr Tag   \t\t: 0x%X\r\n", (int)fw_tag.addr_tag);
  logPrintf("Addr Fw    \t\t: 0x%X\r\n", (int)fw_tag.addr_fw);
  logPrintf("Addr Hw    \t\t: 0x%X\r\n", (int)hwInit);

  gpioInit();
  spiInit();
  flashInit();

  usbInit();


  lcdInit();  


#ifdef OPUS_TEST
  int sampling_rate = 16000;
  int num_channels = 1;
  int application = OPUS_APPLICATION_VOIP;

  opus_get_version_string();

  OpusEncoder *enc;
  OpusDecoder *dec;
  int rc,err;

  enc = opus_encoder_create(sampling_rate, num_channels, application, &err);
  if(err != OPUS_OK || enc==NULL)
  {
  }
  dec = opus_decoder_create(sampling_rate, num_channels, &err);
  if(err!=OPUS_OK || dec==NULL)
  {
  }

  int bitrate = 16000/2;

  if(opus_encoder_ctl(enc, OPUS_SET_BITRATE(bitrate)) != OPUS_OK) test_failed();
  if(opus_encoder_ctl(enc, OPUS_SET_VBR(1)) != OPUS_OK) test_failed();
  if(opus_encoder_ctl(enc, OPUS_SET_VBR_CONSTRAINT(0)) != OPUS_OK) test_failed();
  if(opus_encoder_ctl(enc, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_20_MS)) != OPUS_OK) test_failed();
  if(opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(0)) != OPUS_OK) test_failed();

  opus_int16 inbuf[640];
  opus_int16 outbuf[640];
  unsigned char packet[15000+257];
  int len;
  int out_samples;

#if 0
  for (int i=0; i<640; i++)
  {
    //inbuf[i] = rand()%0xFFFF;
    inbuf[i] = i % 15;
  }

  for (int j=0; j<1; j++)
  {
    uint32_t dif_time;
    uint32_t pre_time = micros();
    //for (int i=0; i<10; i++)
    {
      len = opus_encode(enc, &inbuf[0], 320, packet, 15000);
    }
    dif_time = micros()-pre_time;

    pre_time = micros();
    out_samples = opus_decode(dec, packet, len, outbuf, 320, 0);

    dif_time = micros()-pre_time;
  }

  for (int i=0; i<320; i++)
  {
    logPrintf("%d %d %d\n", i, inbuf[i], outbuf[i]);
  }
#else
  uint32_t max_enc_time = 0;
  uint32_t max_dec_time = 0;
  uint32_t max_enc_len = 0;
  uint32_t max_enc_size = 0;
  uint32_t max_dec_size = 0;
  int index = 0;

  logPrintf("\n\n");
  logPrintf("int16_t wav_data[%d] = \n", (NUM_ELEMENTS/320)*320);
  logPrintf("{\n");

  for (int j=0; j<NUM_ELEMENTS/320; j++)
  {
    uint32_t dif_time;
    uint32_t pre_time = micros();
    len = opus_encode(enc, (const opus_int16 *)&wav_data[j*320], 320, packet, 15000);
    dif_time = micros()-pre_time;

    if (dif_time > max_enc_time) max_enc_time = dif_time;
    if (len > max_enc_len) max_enc_len = len;

    max_enc_size += len;

    pre_time = micros();
    out_samples = opus_decode(dec, packet, len, outbuf, 320, 0);
    dif_time = micros()-pre_time;
    max_dec_size += out_samples;


    index = 0;
    for (int i=0; i<32; i++)
    {
      logPrintf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, \n",
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5),
                (int)(outbuf[index++]*1.5)
                );
    }
    if (dif_time > max_dec_time) max_dec_time = dif_time;
  }
  logPrintf("max size : %d\n", (NUM_ELEMENTS/320) * 320);
  logPrintf("max enc time : %d.%d ms\n", max_enc_time/1000, (max_enc_time/100)%10);
  logPrintf("max dec time : %d.%d ms\n", max_dec_time/1000, (max_dec_time/100)%10);
  logPrintf("max enc len  : %d \n", max_enc_len);
  logPrintf("max enc size : %d \n", max_enc_size);
  logPrintf("max dec size : %d \n", max_dec_size);
#endif
#endif


  // After usbInit()
  //radioInit();
  bleUartInit();
}
