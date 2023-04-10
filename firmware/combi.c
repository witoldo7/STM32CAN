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
#include <utils.h>

uint8_t version[2] = {0x03, 0x01};
uint8_t egt_temp[5] = {0};

CAN_RamAddress can1_ram;

CAN_Filter filter0 = {
  .IdType = FDCAN_STANDARD_ID,
  .FilterIndex = 0,
  .FilterType = FDCAN_FILTER_MASK,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO1_HP,
  .FilterID1 = 0x7E8,
  .FilterID2 = 0x7FF,
};

CAN_Filter filter1 = {
  .IdType = FDCAN_STANDARD_ID,
  .FilterIndex = 1,
  .FilterType = FDCAN_FILTER_MASK,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO0_HP,
  .FilterID1 = 0x7E0,
  .FilterID2 = 0x7FF,
};

CAN_Filter filter2 = {
  .IdType = FDCAN_STANDARD_ID,
  .FilterIndex = 2,
  .FilterType = FDCAN_FILTER_DUAL,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
  .FilterID1 = 0x5E8,
  .FilterID2 = 0x311,
};

CAN_Filter filter3 = {
  .IdType = FDCAN_STANDARD_ID,
  .FilterIndex = 3,
  .FilterType = FDCAN_FILTER_MASK,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
  .FilterID1 = 0x011,
  .FilterID2 = 0x7FF,
};

CANRamConfig can1_ram_cfg = {
  .MessageRAMOffset = 0,
  .StdFiltersNbr = 4,
  .ExtFiltersNbr = 0,
  .RxFifo0ElmtsNbr = 16,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
  .RxFifo1ElmtsNbr = 16,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
  .RxBuffersNbr = 16,
  .RxBufferSize = FDCAN_DATA_BYTES_8,
  .TxEventsNbr = 1,
  .TxBuffersNbr = 2,
  .TxFifoQueueElmtsNbr = 1,
  .TxElmtSize = FDCAN_DATA_BYTES_8
};

static CANConfig canConfig1 = {
  .DBTP =  0,
  .CCCR =  0, //FDCAN_CCCR_TEST,
  .TEST =  0, //FDCAN_TEST_LBCK,
};

bool combi_rx_can_cb(CANRxFrame *rxmsg, packet_t *packet) {
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
      registerHsCanCallback(NULL);
      canStop(&CAND1);
      break;
    case combi_open:
      registerHsCanCallback(&combi_rx_can_cb);
      canMemorryConfig(&CAND1, &canConfig1, &can1_ram_cfg, &can1_ram);
      canGlobalFilter(&canConfig1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
      canStart(&CAND1, &canConfig1);
      //Combilib + Trionic 8 filters
      canFilter(&can1_ram, &filter0);
      canFilter(&can1_ram, &filter1);
      canFilter(&can1_ram, &filter2);
      canFilter(&can1_ram, &filter3);
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
