// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#ifndef __COMBI_H__
#define __COMBI_H__

#include "ch.h"
#include "utils.h"

enum combi_command_t {
  //BRD
  cmd_brd_fwversion  = 0x20,
  cmd_brd_adcfilter  = 0x21,
  cmd_brd_adc        = 0x22,
  cmd_brd_egt        = 0x23,
  cmd_brd_vbat       = 0x24,
  //CAN
  cmd_can_open       = 0x80,
  cmd_can_bitrate    = 0x81,
  cmd_can_rxframe    = 0x82,
  cmd_can_txframe    = 0x83,
  cmd_can_ecuconnect = 0x89,
  cmd_can_readflash  = 0x8A,
  cmd_can_writeflash = 0x8B,
};

enum open_command_t {
   combi_close       = 0x00,
   combi_open        = 0x01
};

extern bool exec_cmd_board(packet_t *rx_packet, packet_t *tx_packet);
extern bool exec_cmd_can(packet_t *rx_packet, packet_t *tx_packet);
#endif
