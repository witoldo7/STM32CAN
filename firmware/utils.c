// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#include <utils.h>
#include "hal.h"
#include "string.h"

//------------------------ADC-BEGIN-------------------------

#define ADC_GRP1_NUM_CHANNELS       1
#define ADC_GRP1_BUF_DEPTH      1

/* Buffers are allocated with size and address aligned to the cache
   line size.*/
#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
adcsample_t samples1[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH)];

/*
 * ADC conversion group 1.
 * Mode:        One shot, 1 channel, SW triggered.
 * Channels:    IN5.
 */
const ADCConversionGroup adcgrpcfg1 = {
  .circular     = false,
  .num_channels = ADC_GRP1_NUM_CHANNELS,
  .end_cb       = NULL,
  .error_cb     = NULL,
  .cfgr         = 0U,
  .cfgr2        = 0U,
  .ccr          = 0U,
  .pcsel        = ADC_SELMASK_IN5,
  .ltr1         = 0x00000000U,
  .htr1         = 0x03FFFFFFU,
  .ltr2         = 0x00000000U,
  .htr2         = 0x03FFFFFFU,
  .ltr3         = 0x00000000U,
  .htr3         = 0x03FFFFFFU,
  .smpr         = {
    ADC_SMPR1_SMP_AN5(ADC_SMPR_SMP_810P5),
    0U
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN5),
    0U,
    0U,
    0U
  }
};

uint16_t getSupplyVoltage(void) {
  uint16_t voltage = 0;
  adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
  voltage = samples1[0] / 3;
  cacheBufferInvalidate(samples1, sizeof (samples1) / sizeof (adcsample_t));
  return voltage;
}

void get_vbat(uint8_t *vbat) {
  uint16_t voltage = getSupplyVoltage();
  memcpy(vbat, &voltage, 2);
}
//------------------------ADC-END-----------------------------

//-------------------------OTHERS-------------------------------
void swab(uint16_t *word) {
  uint16_t tmp;
  if (word != 0) {
    tmp = *word;
    *word = *word << 8;
    *word = tmp >> 8 | *word;
  }
  return;
}

bool prepareReplyPacket(packet_t *reply, packet_t *source, uint8_t *data,
                          uint16_t data_len, uint8_t term) {
  reply->cmd_code = source->cmd_code;
  reply->data_len = data_len;
  if (data_len != 0) {
    memcpy(reply->data, data, data_len);
  }
  reply->term = term;
  return true;
}

uint8_t covertPacketToBuffer(packet_t *packet, uint8_t *buffer) {
  uint8_t size = 0;
  buffer[0] = packet->cmd_code;
  buffer[1] = (uint8_t)(packet->data_len >> 8);
  buffer[2] = (uint8_t)packet->data_len;
  if (packet->data_len != 0) {
    memcpy(buffer + 3, packet->data, packet->data_len);
    size = packet->data_len + 3;
    buffer[size] = packet->term;
    size++;
  }
  else {
    buffer[3] = packet->term;
    size = 4;
  }
  return size;
}


