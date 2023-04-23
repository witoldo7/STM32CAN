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
  uint32_t Loopback;
  uint32_t NodeAddress;
  uint32_t NetworkLine;
  uint32_t P1Min;
  uint32_t P1Max;
  uint32_t P2Min;
  uint32_t P2Max;
  uint32_t P3Min;
  uint32_t P3Max;
  uint32_t P4Min;
  uint32_t P4Max;
  uint32_t W0a;
  uint32_t W1a;
  uint32_t W2a;
  uint32_t W3a;
  uint32_t W4a;
  uint32_t W5a;
  uint32_t Tidle;
  uint32_t Tinil;
  uint32_t Twup;
  uint32_t Parity;
  uint32_t BitSamplePoint;
  uint32_t SyncJumpWidth;
  uint32_t T1Max;
  uint32_t T2Max;
  uint32_t T3Max;
  uint32_t T4Max;
  uint32_t T5Max;
  uint32_t Iso15765Bs;
  uint32_t Iso15765Stmin;
  uint32_t Iso15765BsTx;
  uint32_t Iso15765StminTx;
  uint32_t DataBits;
  uint32_t FiveBaudMod;
  uint32_t Iso15765WftMax;
  uint32_t CanMixedFormat;
  uint32_t J1962Pins;
  uint32_t SwCanHsDataRate;
  uint32_t SwCanSpeedchangeEnable;
  uint32_t SwCanResSwitch;
  uint32_t ActiveChannels;
  uint32_t SampleRate;
  uint32_t SamplesPerReading;
  uint32_t ReadingsPerMsg;
  uint32_t AveragingMethod;
  uint32_t SampleResolution;
  uint32_t InputRangeLow;
  uint32_t InputRangeHigh;
} j2534_protocol_cfg;

typedef struct {
  CANDriver* canp;
  CANConfig* canCfg;
  CANRamConfig* ramCfg;
  CAN_RamAddress* ramAdr;
  CAN_FILTER* canFilter;
} j2534_can_cfg;

typedef struct {
  uint32_t protocol;
  uint32_t flags;
  uint32_t DataRate;
  j2534_protocol_cfg* pcfg;
  void* cfg;
  uint32_t (*connect)(void*);
  uint32_t (*disconnect)(void*);
  uint32_t (*write)(void*, uint32_t, uint16_t, uint8_t*);
  uint32_t (*start_filter)(void*, uint8_t*, uint32_t*);
  uint32_t (*stop_filter)(void*, uint32_t);
  uint32_t (*start_periodic_msg)(void*, uint8_t*);
  uint32_t (*stop_periodic_msg)(void*, uint32_t);
  uint32_t (*ioctl_lopback)(void*);
  uint32_t (*ioctl_datarate)(void*);
  uint32_t (*ioctl_clear_filters)(void*);
  uint32_t (*ioctl_five_baud_init)(void*, uint8_t*, uint8_t*);
  uint32_t (*ioctl_fast_init)(void*, uint8_t*, uint8_t*);
  bool isConnected;
} j2534_conn;

extern bool exec_cmd_j2534(packet_t *rx_packet, packet_t *tx_packet);

#endif /* J2534_H_ */
