// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#include "hal.h"
#include <stdio.h>
#include "combi.h"
#include "string.h"
#include "utils.h"
#include "canutils.h"

uint8_t version[2] = {0x03, 0x01};
uint8_t egt_temp[5] = {0};

static CANConfig canConfig1 = {
  OPMODE_CAN,//OPMODE_FDCAN,       /* OP MODE */
  0,                               /* NBTP */
  0,                               /* DBTP */
  0,                               /* TDCR */
  0,                               /* CCCR */
  0,                               /* TEST */
  0                                /* GFC */
};

#define FILTER_SIZE 5
static CANFilter filters[FILTER_SIZE] = {
{
  .filter_mode = CAN_FILTER_MODE_CLASSIC,
  .filter_cfg = CAN_FILTER_CFG_FIFO_0,
  .filter_type = CAN_FILTER_TYPE_STD,
  .identifier1 = 0x180,
  .identifier2 = 0xFFFF,
},
{
  .filter_mode = CAN_FILTER_MODE_CLASSIC,
  .filter_cfg = CAN_FILTER_CFG_FIFO_0,
  .filter_type = CAN_FILTER_TYPE_STD,
  .identifier1 = 0x5E8,
  .identifier2 = 0xFFFF,
},
{
  .filter_mode = CAN_FILTER_MODE_CLASSIC,
  .filter_cfg = CAN_FILTER_CFG_FIFO_0,
  .filter_type = CAN_FILTER_TYPE_STD,
  .identifier1 = 0x7E8,
  .identifier2 = 0xFFFF,
},
{
  .filter_mode = CAN_FILTER_MODE_CLASSIC,
  .filter_cfg = CAN_FILTER_CFG_FIFO_0,
  .filter_type = CAN_FILTER_TYPE_STD,
  .identifier1 = 0x664,
  .identifier2 = 0xFFFF,
},
{
  .filter_mode = CAN_FILTER_MODE_CLASSIC,
  .filter_cfg = CAN_FILTER_CFG_FIFO_0,
  .filter_type = CAN_FILTER_TYPE_STD,
  .identifier1 = 0x665,
  .identifier2 = 0xFFFF,
},
};

bool combi_rx_can_cb(void* conn, void *msg, packet_t *packet) {
  (void)conn;
  CANRxFrame *rxmsg = (CANRxFrame*) msg;
  if (rxmsg->common.XTD) {
    packet->data[0] = rxmsg->ext.EID & 0xFF;
    packet->data[1] = (rxmsg->ext.EID >> 8) & 0xFF;
    packet->data[2] = (rxmsg->ext.EID >> 16) & 0xFF;
    packet->data[3] = (rxmsg->ext.EID >> 24) & 0xFF;
  } else {
    packet->data[0] = rxmsg->std.SID & 0xFF;
    packet->data[1] = (rxmsg->std.SID >> 8) & 0xFF;
    packet->data[2] = (rxmsg->std.SID >> 16) & 0xFF;
  }
  packet->data[12] = rxmsg->DLC;
  packet->data[13] = rxmsg->common.XTD;
  packet->data[14] = rxmsg->common.RTR;
  memcpy(packet->data + 4, rxmsg->data8, rxmsg->DLC);
  packet->cmd_code = cmd_can_rxframe;
  packet->data_len = 15;
  return true;
}

bool exec_cmd_can(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_can_ecuconnect:
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_can_open:
    switch (rx_packet->data[0]) {
    case combi_close:
      canStop(&CAND1);
      registerHsCanCallback(NULL, NULL);
      break;
    case combi_open:
      registerHsCanCallback(&combi_rx_can_cb, NULL);
      canGlobalFilter(&canConfig1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
      canStart(&CAND1, &canConfig1);
      canSTM32SetFilters(&CAND1, FILTER_SIZE, filters);
      break;
    default:
      return false;
    }
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    break;

  case cmd_can_bitrate:
      uint32_t bitrate = rx_packet->data[3] | (uint32_t)*rx_packet->data << 0x18 | (uint32_t)rx_packet->data[1] << 0x10
          | (uint32_t)rx_packet->data[2] << 8;
      bool ret = canBaudRate(&canConfig1, bitrate, NULL, NULL);
      return ret && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);

  case cmd_can_txframe:
    if (rx_packet->data_len != 15) {
      return false;
    }
    CANTxFrame txmsg = {};
    if (rx_packet->data[13] == 1) {
      txmsg.ext.EID = (uint32_t)rx_packet->data[0] | (uint32_t)(rx_packet->data[1] << 8)
          | (uint32_t)(rx_packet->data[2] << 16) | (uint32_t)(rx_packet->data[3] << 24);
      txmsg.common.XTD = true;
    } else {
      txmsg.std.SID = ((uint32_t)rx_packet->data[0] | (uint32_t)(rx_packet->data[1] << 8)
          | (uint32_t)(rx_packet->data[2] << 16)) & 0x7FF;
    }
    txmsg.DLC = rx_packet->data[12];
    txmsg.common.RTR = rx_packet->data[14];
    memcpy(txmsg.data8, rx_packet->data + 4, can_fd_dlc2len(rx_packet->data[12]));

    return (canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(200)) == MSG_OK)
        && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);

   break;
  }
  return false;
}

bool exec_cmd_board(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_brd_fwversion:
    return prepareReplyPacket(tx_packet, rx_packet, version, 2, cmd_term_ack);
  case cmd_brd_adcfilter:
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_brd_adc:
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_brd_egt:
    return prepareReplyPacket(tx_packet, rx_packet, egt_temp, 5, cmd_term_ack);
  case cmd_brd_vbat:
    uint8_t vbat[4] = {0};
    get_vbat(vbat);
    return prepareReplyPacket(tx_packet, rx_packet, vbat, 2, cmd_term_ack);
  }
  return false;
}
