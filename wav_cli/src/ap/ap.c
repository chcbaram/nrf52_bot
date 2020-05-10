/*
 * ap.c
 *
 *  Created on: 2018. 8. 25.
 *      Author: Baram
 */




#include "ap.h"
#include "util.h"
#include <unistd.h>
#include "wav.h"
#include "opus.h"

#define FLASH_TAG_SIZE      0x400




flash_tag_t    fw_tag;



int32_t getFileSize(char *file_name);
bool addTagToBin(char *src_filename, char *dst_filename);
bool makeWav(void);
bool makeWavToOpus(char *file_name);


void apInit(void)
{

}

void apMain(int argc, char *argv[])
{
  bool ret;
  char *file_name;


  char dst_filename[256];


  setbuf(stdout, NULL);



  if (argc == 2)
  {
    file_name = (char *)argv[ 1 ];

    printf("makeWavToOpus : %s\n", file_name);
    makeWavToOpus(file_name);
    return;
  }

  makeWav();
}




int32_t getFileSize(char *file_name)
{
  int32_t ret = -1;
  static FILE *fp;


  if( ( fp = fopen( file_name, "rb" ) ) == NULL )
  {
    fprintf( stderr, "Unable to open %s\n", file_name );
    exit( 1 );
  }
  else
  {
    fread( (void *)&fw_tag, 1, sizeof(flash_tag_t), fp );

    fseek( fp, 0, SEEK_END );
    ret = ftell( fp );
    fseek( fp, 0, SEEK_SET );
    fclose(fp);
  }

  return ret;
}

bool addTagToBin(char *src_filename, char *dst_filename)
{
  FILE    *p_fd;
  uint8_t *buf;
  size_t   src_len;
  uint16_t t_crc = 0;
  flash_tag_t *p_tag;


  if (!strcmp(src_filename, dst_filename))
  {
    fprintf( stderr, "  src file(%s) and dst file(%s) is same! \n", src_filename, dst_filename );
    exit( 1 );
  }


  /* Open src file */
  if ((p_fd = fopen(src_filename, "rb")) == NULL)
  {
    fprintf( stderr, "  unable to open src file(%s)\n", src_filename );
    exit( 1 );
  }

  fseek( p_fd, 0, SEEK_END );
  src_len = ftell( p_fd );
  fseek( p_fd, 0, SEEK_SET );

  if ((buf = (uint8_t *) calloc(src_len, sizeof(uint8_t))) == NULL)
  {
    fclose(p_fd);
    fprintf( stderr, "  malloc Error \n");
    exit( 1 );
  }


  /* Copy read fp to buf */
  if(fread( &buf[0], 1, src_len, p_fd ) != src_len)
  {
    fclose(p_fd);
    free(buf);
    fprintf( stderr, "  length is wrong! \n" );
    exit( 1 );
  }
  fclose(p_fd);


  /* Calculate CRC16 */
  size_t i;
  for (i = 0; i<src_len - FLASH_TAG_SIZE; i++)
  {
    utilUpdateCrc(&t_crc, buf[FLASH_TAG_SIZE + i]);
  }

  p_tag = (flash_tag_t *)buf;


  if (p_tag->magic_number == 0x5555AAAA)
  {
    free(buf);
    fprintf( stderr, "  already magic number\n");
    return true;
  }
  else if (p_tag->magic_number != 0xAAAA5555)
  {
    free(buf);
    fprintf( stderr, "  wrong magic number 0x%X \n", p_tag->magic_number);
    return false;
  }

  p_tag->magic_number     = 0x5555AAAA;
  p_tag->tag_flash_start  = p_tag->addr_fw;
  p_tag->tag_flash_end    = p_tag->addr_fw + (src_len - FLASH_TAG_SIZE);
  p_tag->tag_flash_length = p_tag->tag_flash_end - p_tag->tag_flash_start;
  p_tag->tag_length       = FLASH_TAG_SIZE;
  strcpy((char *)p_tag->tag_date_str, __DATE__);
  strcpy((char *)p_tag->tag_time_str, __TIME__);
  p_tag->tag_flash_crc = t_crc;


  /* Store data to dst file */
  if ((p_fd = fopen(dst_filename, "wb")) == NULL)
  {
    free(buf);
    fprintf( stderr, "  unable to open dst file(%s)\n", dst_filename );
    exit( 1 );
  }

  if(fwrite(buf, 1, src_len, p_fd) != src_len)
  {
    fclose(p_fd);
    free(buf);
    //_unlink(dst_filename);
    fprintf( stderr, "  total write fail! \n" );
    exit( 1 );
  }

  printf("  created file  : %s (%d KB)\n", dst_filename, (int)((src_len)/1024) );
  printf("  tag fw start  : 0x%08X \n", p_tag->tag_flash_start);
  printf("  tag fw end    : 0x%08X \n", p_tag->tag_flash_end);
  printf("  tag crc       : 0x%04X \n", p_tag->tag_flash_crc);
  printf("  tag date      : %s \n", p_tag->tag_date_str);
  printf("  tag time      : %s \n", p_tag->tag_time_str);

  fclose(p_fd);
  free(buf);

  return true;
}


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>


/*
The header of a wav file Based on:
https://ccrma.stanford.edu/courses/422/projects/WaveFormat/
*/
typedef struct wavfile_header_s
{
    char    ChunkID[4];     /*  4   */
    int32_t ChunkSize;      /*  4   */
    char    Format[4];      /*  4   */

    char    Subchunk1ID[4]; /*  4   */
    int32_t Subchunk1Size;  /*  4   */
    int16_t AudioFormat;    /*  2   */
    int16_t NumChannels;    /*  2   */
    int32_t SampleRate;     /*  4   */
    int32_t ByteRate;       /*  4   */
    int16_t BlockAlign;     /*  2   */
    int16_t BitsPerSample;  /*  2   */

    char    Subchunk2ID[4];
    int32_t Subchunk2Size;
} wavfile_header_t;

/*Standard values for CD-quality audio*/
#define SUBCHUNK1SIZE   (16)
#define AUDIO_FORMAT    (1) /*For PCM*/
#define NUM_CHANNELS    (1)
#define SAMPLE_RATE     (16000/1)
#define BIT_RATE        16




bool makeWav(void)
{
  FILE    *p_fd;
  size_t   src_len;
  wavfile_header_t header;



  memcpy(header.ChunkID, "RIFF", 4);
  header.ChunkSize = sizeof(wav_data) + 36;
  memcpy(header.Format, "WAVE", 4);
  memcpy(header.Subchunk1ID, "fmt ", 4);
  header.Subchunk1Size = 0x10;

  header.AudioFormat = AUDIO_FORMAT;
  header.NumChannels = NUM_CHANNELS;
  header.SampleRate = SAMPLE_RATE;
  header.ByteRate = SAMPLE_RATE * NUM_CHANNELS * BIT_RATE / 8;
  header.BlockAlign = NUM_CHANNELS * BIT_RATE / 8;
  header.BitsPerSample = BIT_RATE;

  memcpy(header.Subchunk2ID, "data", 4);
  header.Subchunk2Size = sizeof(wav_data);

  printf("%d, %d\n", sizeof(wavfile_header_t), sizeof(wav_data));


  /* Store data to dst file */
  if ((p_fd = fopen("wav.wav", "wb")) == NULL)
  {
    fprintf( stderr, "  unable to open dst file\n");
    exit( 1 );
  }

  if(fwrite(&header, 1, sizeof(wavfile_header_t), p_fd) != sizeof(wavfile_header_t))
  {
    fclose(p_fd);
    //_unlink(dst_filename);
    fprintf( stderr, "  total write fail! \n" );
    exit( 1 );
  }

  fwrite(&wav_data[0], 1, sizeof(wav_data), p_fd);



  fclose(p_fd);

  return true;
}



static int err;
static int sampling_rate = 16000;
static int bitrate       = 16000;
static int num_channels  = 1;
static int application   = OPUS_APPLICATION_VOIP;

static OpusEncoder *enc;
static OpusDecoder *dec;

static opus_int16 outbuf[640];
static uint8_t    packet[1024+257];


static bool codecOpusInit()
{
  enc = opus_encoder_create(sampling_rate, num_channels, application, &err);
  if(err != OPUS_OK || enc==NULL)
  {
    return false;
  }
  printf("opus_encoder_create OK, %d\n", err);

  dec = opus_decoder_create(sampling_rate, num_channels, &err);
  if(err != OPUS_OK || enc==NULL)
  {
    return false;
  }
  printf("opus_decoder_create OK, %d\n", err);


  //if(opus_encoder_ctl(enc, OPUS_SET_BITRATE(OPUS_AUTO)) != OPUS_OK) return false;
  if(opus_encoder_ctl(enc, OPUS_SET_BITRATE(sampling_rate)) != OPUS_OK) return false;
  if(opus_encoder_ctl(enc, OPUS_SET_VBR(1)) != OPUS_OK) return false;
  if(opus_encoder_ctl(enc, OPUS_SET_VBR_CONSTRAINT(0)) != OPUS_OK) return false;
  if(opus_encoder_ctl(enc, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_20_MS)) != OPUS_OK) return false;
  if(opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(0)) != OPUS_OK) return false;

  if(opus_decoder_ctl(dec, OPUS_SET_GAIN(3000)) != OPUS_OK) return false;




  return true;
}



typedef struct
{
  uint32_t magic_number;        // 4
  uint32_t file_type;           // 4
  uint16_t frame_len;           // 2
  uint16_t enc_len;             // 2
  uint32_t file_len;            // 4
  uint32_t reserved[4];
} opus_tag_t;


bool makeWavToOpus(char *file_name)
{
  int src_len;
  wavfile_header_t header;
  FILE    *fp_wav;
  FILE    *fp_opus;
  FILE    *fp_wavout;


  codecOpusInit();


  if ((fp_wav = fopen(file_name, "rb")) == NULL)
  {
    fprintf( stderr, "  unable to open src file(%s)\n", file_name );
    exit( 1 );
  }

  fseek( fp_wav, 0, SEEK_END );
  src_len = ftell( fp_wav );
  fseek( fp_wav, 0, SEEK_SET );

  /* Copy read fp to buf */
  if(fread( &header, 1, sizeof(wavfile_header_t), fp_wav ) != sizeof(wavfile_header_t))
  {
    fclose(fp_wav);
    fprintf( stderr, "  length is wrong! \n" );
    exit( 1 );
  }


  header.AudioFormat = AUDIO_FORMAT;
  header.NumChannels = NUM_CHANNELS;
  header.SampleRate = SAMPLE_RATE;
  header.ByteRate = SAMPLE_RATE * NUM_CHANNELS * BIT_RATE / 8;
  header.BlockAlign = NUM_CHANNELS * BIT_RATE / 8;
  header.BitsPerSample = BIT_RATE;

  printf("AudioFormat   %d\n", header.AudioFormat);
  printf("NumChannels   %d\n", header.NumChannels);
  printf("SampleRate    %d\n", header.SampleRate);
  printf("ByteRate      %d\n", header.ByteRate);
  printf("BlockAlign    %d\n", header.BlockAlign);
  printf("BitsPerSample %d\n", header.BitsPerSample);

  printf("DataSize      %d\n", header.Subchunk2Size);

  printf("HeaderSize    %d\n", sizeof(header));

  char opus_file_name[128];
  char wavout_file_name[128];


  sprintf(opus_file_name, "%s.opus", file_name);
  sprintf(wavout_file_name, "%s.out.wav", file_name);


  if ((fp_opus = fopen(opus_file_name, "wb")) == NULL)
  {
    fclose(fp_opus);
    fprintf( stderr, "  unable to open src file(%s)\n", opus_file_name );
    exit( 1 );
  }

  if ((fp_wavout = fopen(wavout_file_name, "wb")) == NULL)
  {
    fclose(fp_wavout);
    fprintf( stderr, "  unable to open src file(%s)\n", wavout_file_name );
    exit( 1 );
  }
  fseek( fp_wavout, sizeof(wavfile_header_t), SEEK_SET );


  uint32_t max_file_len = 0;
  uint32_t max_enc_len = 0;
  uint32_t max_enc_size = 0;
  uint32_t max_dec_size = 0;
  uint32_t max_pkt_size = 0;


  uint8_t *opus_file_buf;
  uint32_t opus_file_len = 0;
  opus_tag_t *p_opus_tag;



  opus_file_buf = (uint8_t *)malloc(src_len);
  if (opus_file_buf == NULL)
  {
    fclose(fp_opus);
    fclose(fp_wav);
    return false;
  }

  p_opus_tag = (opus_tag_t *)opus_file_buf;

  p_opus_tag->magic_number = 0x5555AAAA;
  p_opus_tag->frame_len = 320;
  p_opus_tag->file_len = 0;
  while(1)
  {
    int wav_len;
    int enc_len;
    int dec_len;
    int frame_len = 320;
    int16_t wav_buf[frame_len];

    wav_len = fread( wav_buf, 1, frame_len*2, fp_wav );

    if (wav_len < frame_len*2)
    {
      break;
    }
    enc_len = opus_encode(enc, (const opus_int16 *)wav_buf, frame_len, packet, 1024);


    for (int i=0; i<enc_len; i++)
    {
      opus_file_buf[sizeof(opus_tag_t) + max_enc_size + i] = packet[i];
    }

    if (enc_len > max_enc_len) max_enc_len = enc_len;


    dec_len = opus_decode(dec, packet, enc_len, outbuf, frame_len, 0);

    if (max_pkt_size >= 2)
    {
      fwrite(outbuf, 1, dec_len*2, fp_wavout);
    }
    else
    {
      memset(outbuf, 0x00, 320*2);
      fwrite(outbuf, 1, dec_len*2, fp_wavout);
    }

    max_enc_size += enc_len;
    max_file_len += wav_len;
    max_dec_size += dec_len*2;

    max_pkt_size += 1;
  }

  printf("max size     : %d \n", max_file_len);
  printf("max enc len  : %d \n", max_enc_len);
  printf("max enc size : %d , %d\n", max_enc_size, max_enc_size+max_pkt_size);
  printf("max dec size : %d \n", max_dec_size);


  p_opus_tag->file_len = max_enc_size;
  p_opus_tag->enc_len = max_enc_len;


  fseek( fp_wavout, 0, SEEK_SET );

  memcpy(header.ChunkID, "RIFF", 4);
  memcpy(header.Format, "WAVE", 4);
  memcpy(header.Subchunk1ID, "fmt ", 4);
  header.Subchunk1Size = 0x10;
  header.AudioFormat = AUDIO_FORMAT;
  header.NumChannels = NUM_CHANNELS;
  header.SampleRate = SAMPLE_RATE;
  header.ByteRate = SAMPLE_RATE * NUM_CHANNELS * BIT_RATE / 8;
  header.BlockAlign = NUM_CHANNELS * BIT_RATE / 8;
  header.BitsPerSample = BIT_RATE;
  memcpy(header.Subchunk2ID, "data", 4);


  header.ChunkSize = max_file_len  + 36;
  header.Subchunk2Size = max_file_len;
  fwrite(&header, 1, sizeof(header), fp_wavout);


  fwrite(opus_file_buf, 1, sizeof(opus_tag_t) + max_enc_size, fp_opus);
  free(opus_file_buf);

  fclose(fp_opus);
  fclose(fp_wav);
  fclose(fp_wavout);


  return true;
}
