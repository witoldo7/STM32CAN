// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#ifndef J2534_H_
#define J2534_H_

#include "utils.h"
#include "canutils.h"
#include "j2534def.h"

#define FILTER_NBR 16

typedef struct {
  CAN_Filter filter[FILTER_NBR];
  CAN_RamAddress *msgRam;
  uint8_t index;
} CAN_FILTER;

typedef struct {
  CANDriver* canp;
  CANConfig* canCfg;
  CANRamConfig* ramCfg;
  CAN_RamAddress* ramAdr;
  CAN_FILTER* canFilter;
  bool (*cb)(void*, packet_t*);
  uint32_t protocol;
  uint32_t flags;
  uint32_t bitRate;
  uint32_t bitSamplePoint;
  uint32_t syncJumpWidth;
  uint32_t iso15766bs;
  uint32_t iso15766bsTx;
  uint32_t iso15766stmin;
  uint32_t iso15766stminTx;
  uint32_t iso15766wtfMax;
  uint32_t canMixedFormat;
  uint32_t J1962Pins;
  uint32_t swCanHsDataRate;
  uint32_t swCanSpeedChangeEnable;
  uint32_t swCanResSwitch;
  bool loopback;
  bool isConnected;
} j2534_conn;

typedef struct {

} j2534_can_cfg;

extern bool exec_cmd_j2534(packet_t *rx_packet, packet_t *tx_packet);

#endif /* J2534_H_ */
