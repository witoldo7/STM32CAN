// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "hal.h"

typedef struct {
  uint8_t cmd_code;  // command code
  uint16_t data_len; // data block length
  uint8_t *data;     // optional data block
  uint8_t term;      // terminator
} packet_t;

enum term_command_t {
  cmd_term_ack = 0x00,
  cmd_term_nack = 0xFF
};

static inline uint16_t min(uint16_t a, uint16_t b) {
    return (a < b) ? a : b;
}

void swab(uint16_t *word);
uint16_t getSupplyVoltage(void);
void get_vbat(uint8_t *vbat);
bool prepareReplyPacket(packet_t *reply, packet_t *source, uint8_t *data, uint16_t data_len, uint8_t term);
uint8_t covertPacketToBuffer(packet_t *packet, uint8_t *buffer);
#endif /* UTILS_H_ */
