// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#ifndef __COMBI_H__
#define __COMBI_H__

#include "ch.h"

typedef struct packet_t packet_t;

enum combi_command_t {
  //BRD
  cmd_brd_fwversion = 0x20,
  cmd_brd_adcfilter = 0x21,
  cmd_brd_adc = 0x22,
  cmd_brd_egt = 0x23,
  //BDM
  cmd_bdm_stop_chip = 0x40,
  cmd_bdm_reset_chip = 0x41,
  cmd_bdm_run_chip = 0x42,
  cmd_bdm_step_chip = 0x43,
  cmd_bdm_restart_chip = 0x44,
  cmd_bdm_mem_read = 0x45,
  cmd_bdm_mem_write = 0x46,
  cmd_bdm_sysreg_read = 0x47,
  cmd_bdm_sysreg_write = 0x48,
  cmd_bdm_adreg_read = 0x49,
  cmd_bdm_adreg_write = 0x4A,
  cmd_bdm_read_flash = 0x4B,
  cmd_bdm_erase_flash = 0x4C,
  cmd_bdm_write_flash = 0x4D,
  cmd_bdm_pinstate = 0x4E,
  //SWCAN
  cmd_swcan_open = 0x60,
  cmd_swcan_bitrate = 0x61,
  cmd_swcan_rxframe = 0x62,
  cmd_swcan_txframe = 0x63,
  cmd_swcan_filter = 0x64,
  //CAN
  cmd_can_open = 0x80,
  cmd_can_bitrate = 0x81,
  cmd_can_rxframe = 0x82,
  cmd_can_txframe = 0x83,
  cmd_can_filter = 0x84,
  cmd_can_rxframe_fdcan = 0x85,
  cmd_can_txframe_fdcan = 0x86,
  cmd_can_ecuconnect = 0x89,
  cmd_can_readflash = 0x8A,
  cmd_can_writeflash = 0x8B,
  //MISC
  cmd_term_ack = 0x00,
  cmd_term_nack = 0xFF
};

enum open_command_t {
   combi_close = 0x00,
   combi_open = 0x01,
   fdcan_open = 0x02
};

struct packet_t {
  uint8_t cmd_code;	// command code
  uint16_t data_len;	// data block length
  uint8_t *data;		// optional data block
  uint8_t term;		// terminator
};

extern bool CombiSendReplyPacket(packet_t *reply, packet_t *source,
                                 uint8_t *data, uint16_t data_len,
                                 uint8_t term);
extern uint8_t CombiSendPacket(packet_t *packet, uint8_t *buffer);
extern void rx_can_msg(CANRxFrame *rxmsg, packet_t *packet);
extern bool exec_cmd_board(packet_t *rx_packet, packet_t *tx_packet);
extern bool exec_cmd_bdm(packet_t *rx_packet, packet_t *tx_packet);
extern bool exec_cmd_swcan(packet_t *rx_packet, packet_t *tx_packet);
extern bool exec_cmd_can(packet_t *rx_packet, packet_t *tx_packet);
extern void initCAN1(void);
#endif
