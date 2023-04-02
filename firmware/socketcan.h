// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#ifndef SOCKETCAN_H_
#define SOCKETCAN_H_

#include "utils.h"

enum socketcan_command_t {
  cmd_socketcan_connect_hs = 0x60,
  cmd_socketcan_bitrate_hs = 0x61,
  cmd_socketcan_filter_hs  = 0x62,
  cmd_socketcan_rx_hscan   = 0x63,
  cmd_socketcan_tx_hscan   = 0x64,
  cmd_socketcan_connect_sw = 0x68,
  cmd_socketcan_bitrate_sw = 0x69,
  cmd_socketcan_filter_sw  = 0x6A,
  cmd_socketcan_rx_swcan   = 0x6B,
  cmd_socketcan_tx_swcan   = 0x6C,
};

bool exec_cmd_socketcan(packet_t *rx_packet, packet_t *tx_packet);

#endif /* SOCKETCAN_H_ */
