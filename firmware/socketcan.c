// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#include "socketcan.h"
#include "hal.h"
#include "string.h"
#include "utils.h"

CAN_RamAddress ram1, ram2;

CANRamConfig hscan_ram_cfg1 = {
  .MessageRAMOffset = 0,
  .StdFiltersNbr = 8,
  .ExtFiltersNbr = 0,
  .RxFifo0ElmtsNbr = 2,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_64,
  .RxFifo1ElmtsNbr = 2,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_64,
  .RxBuffersNbr = 2,
  .RxBufferSize = FDCAN_DATA_BYTES_64,
  .TxEventsNbr = 2,
  .TxBuffersNbr = 2,
  .TxFifoQueueElmtsNbr = 2,
  .TxElmtSize = FDCAN_DATA_BYTES_64
};

CANRamConfig ram2_cfg = {
  .MessageRAMOffset = 384,
  .StdFiltersNbr = 4,
  .ExtFiltersNbr = 0,
  .RxFifo0ElmtsNbr = 4,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
  .RxFifo1ElmtsNbr = 4,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
  .RxBuffersNbr = 4,
  .RxBufferSize = FDCAN_DATA_BYTES_8,
  .TxEventsNbr = 1,
  .TxBuffersNbr = 4,
  .TxFifoQueueElmtsNbr = 4,
  .TxElmtSize = FDCAN_DATA_BYTES_8
};

static CANConfig canConfig1 = {
  .DBTP =  0,
  .CCCR =  0, //FDCAN_CCCR_TEST,
  .TEST =  0, //FDCAN_TEST_LBCK,
};

static CANConfig canConfig2 = {
  .DBTP =  0,
  .CCCR =  0, //FDCAN_CCCR_TEST,
  .TEST =  0, //FDCAN_TEST_LBCK,
};

/* data[0:3] - SID or EID
 * data[4]
 *    b0 - XTD
 *    b1 - RTRstatic uint8_t canFilterIndex = 0;
 *
 *    b2 - ESI
 *    b3 - BRS
 *    b4 - FDF
 *    b4 - ANMF
 * data[5] - FIDX
 * data[6:7] - RXTS
 * data[8] - DLC
 * data[9:9+DLC] - data
 */
bool socketcan_rx_can_cb(CANRxFrame *rxmsg, packet_t *packet) {
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
  packet->data[4] = (rxmsg->common.XTD) | (rxmsg->common.RTR << 1) | (rxmsg->common.ESI << 2)
                  | (rxmsg->BRS << 3) | (rxmsg->FDF << 4) | (rxmsg->ANMF << 5);
  packet->data[5] = rxmsg->FIDX;
  packet->data[6] = rxmsg->RXTS & 0xFF;
  packet->data[7] = (rxmsg->RXTS >> 8) & 0xFF;
  packet->data[8] = rxmsg->DLC;
  memcpy(packet->data + 9, rxmsg->data8, can_fd_dlc2len(rxmsg->DLC));
  packet->cmd_code = cmd_socketcan_rx_hscan;
  packet->data_len = 73;
  return true;
}

bool socketcan_rx_swcan_cb(CANRxFrame *rxmsg, packet_t *packet) {
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
  packet->cmd_code = cmd_socketcan_rx_hscan;
  packet->data_len = 15;
  return true;
}

bool socketcan_connectHs(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->data[0]) {
  case 0:
    canStop(&CAND1);
    registerHsCanCallback(NULL);
    break;
  case 2:
    registerHsCanCallback(&socketcan_rx_can_cb);
    uint32_t mode = rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
        | (uint32_t)rx_packet->data[3] << 8;
    /* NISO support */
    if (mode & CAN_CTRLMODE_FD_NON_ISO) {
      canConfig1.CCCR |= FDCAN_CCCR_NISO;
    }

    /* FDCAN mode */
    if (mode & CAN_CTRLMODE_FD) {
      canConfig1.CCCR |= (FDCAN_CCCR_BRSE | FDCAN_CCCR_FDOE);
    }

    /* Loopback Mode */
    if (mode & CAN_CTRLMODE_LOOPBACK) {
      canConfig1.CCCR |= FDCAN_CCCR_TEST | FDCAN_CCCR_MON;
      canConfig1.TEST |= FDCAN_TEST_LBCK;
    }

    /* Enable Monitoring */
    if (mode & CAN_CTRLMODE_LISTENONLY) {
      canConfig1.CCCR |= FDCAN_CCCR_MON;
    }

    /* Disable Auto Retransmission */
    if (mode & CAN_CTRLMODE_ONE_SHOT) {
      canConfig1.CCCR |= FDCAN_CCCR_DAR;
    }
    canMemorryConfig(&CAND1, &canConfig1, &hscan_ram_cfg1, &ram1);
    canStart(&CAND1, &canConfig1);
    break;
 default:
   return false;
 }
  return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
}

bool socketcan_connectSw(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->data[0]) {
  case 0:
    canStop(&CAND2);
    registerSwCanCallback(NULL);
    break;
  case 1:
    registerSwCanCallback(&socketcan_rx_swcan_cb);
    uint32_t mode = rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
                 | (uint32_t)rx_packet->data[3] << 8;
     canMemorryConfig(&CAND2, &canConfig2, &ram2_cfg, &ram2);

     /* Loopback Mode */
     if (mode & CAN_CTRLMODE_LOOPBACK) {
       canConfig2.CCCR |= FDCAN_CCCR_TEST | FDCAN_CCCR_MON;
       canConfig2.TEST |= FDCAN_TEST_LBCK;
     }

     /* Enable Monitoring */
     if (mode & CAN_CTRLMODE_LISTENONLY) {
       canConfig2.CCCR |= FDCAN_CCCR_MON;
     }

     /* Disable Auto Retransmission */
     if (mode & CAN_CTRLMODE_ONE_SHOT) {
       canConfig2.CCCR |= FDCAN_CCCR_DAR;
     }
     canStart(&CAND2, &canConfig2);
     ram2.ExtendedFilterSA = 0;
     ram2.StandardFilterSA = 0;
     palSetLine(LINE_SWM0);
     palSetLine(LINE_SWM1);
     break;
  default:
    return false;
  }
   return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
}

bool socketcan_txFrameHs(packet_t *rx_packet, packet_t *tx_packet) {
  registerHsCanCallback(&socketcan_rx_can_cb);
  uint8_t flags = rx_packet->data[4];
  CANTxFrame txmsg = {};
  if (flags & 0b00000001) {
    txmsg.ext.EID = (uint32_t)rx_packet->data[0] | (uint32_t)(rx_packet->data[1] << 8)
        | (uint32_t)(rx_packet->data[2] << 16) | (uint32_t)(rx_packet->data[3] << 24);
    txmsg.common.XTD = true;
  } else {
    txmsg.std.SID = ((uint32_t)rx_packet->data[0] | (uint32_t)(rx_packet->data[1] << 8)
        | (uint32_t)(rx_packet->data[2] << 16)) & 0x7FF;
  }
  txmsg.MM = rx_packet->data[5];
  txmsg.DLC = rx_packet->data[6];
  txmsg.common.RTR = (flags >> 1) & 0x00000001;
  txmsg.common.ESI = (flags >> 2) & 0x00000001;
  txmsg.BRS = (flags >> 3) & 0x00000001;
  txmsg.FDF = (flags >> 4) & 0x00000001;
  txmsg.EFC = (flags >> 5) & 0x00000001;

  memcpy(txmsg.data8, rx_packet->data + 7, can_fd_dlc2len(rx_packet->data[6]));

  return (canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100)) == MSG_OK )
      && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
}

bool socketcan_txFrameSw(packet_t *rx_packet, packet_t *tx_packet) {
  if (rx_packet->data_len != 15) {
    return false;
  }
  CANTxFrame txswmsg = {};
  if (rx_packet->data[13] == 1) {
    txswmsg.ext.EID = (uint32_t)rx_packet->data[0] | (uint32_t)(rx_packet->data[1] << 8)
        | (uint32_t)(rx_packet->data[2] << 16) | (uint32_t)(rx_packet->data[3] << 24);
    txswmsg.common.XTD = true;
  } else {
    txswmsg.std.SID = ((uint32_t)rx_packet->data[0] | (uint32_t)(rx_packet->data[1] << 8)
        | (uint32_t)(rx_packet->data[2] << 16)) & 0x7FF;
  }
  txswmsg.DLC = rx_packet->data[12];
  txswmsg.common.RTR = rx_packet->data[14];
  memcpy(txswmsg.data8, rx_packet->data + 4, can_fd_dlc2len(rx_packet->data[12]));

  return (canTransmit(&CAND2, CAN_ANY_MAILBOX, &txswmsg, TIME_MS2I(100)) == MSG_OK )
      && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
}

bool socketcan_bitrateHs(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->data_len) {
     case 4:
       uint32_t bitrate = rx_packet->data[3] | (uint32_t)*rx_packet->data << 0x18 | (uint32_t)rx_packet->data[1] << 0x10
           | (uint32_t)rx_packet->data[2] << 8;
       bool ret = canBaudRate(&canConfig1, bitrate, NULL, NULL);
       return ret && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
       break;
     case 5:
       canStop(&CAND1);
       if (rx_packet->data[0] == 0) {
         canConfig1.NBTP = rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
             | (uint32_t)rx_packet->data[3] << 8;
       } else {
         canConfig1.DBTP = rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
             | (uint32_t)rx_packet->data[3] << 8;
       }
       break;
     case 9:
         canConfig1.DBTP = rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
             | (uint32_t)rx_packet->data[3] << 8;
         canConfig1.TDCR = rx_packet->data[8] | (uint32_t)rx_packet->data[5] << 24 | (uint32_t)rx_packet->data[6] << 16
             | (uint32_t)rx_packet->data[7] << 8;
       break;
     default:
       return false;
     }
     return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
}

bool socketcan_bitrateSw(packet_t *rx_packet, packet_t *tx_packet) {
  if (rx_packet->data_len == 4) {
    uint32_t bitrate = rx_packet->data[3] | (uint32_t)*rx_packet->data << 0x18 | (uint32_t)rx_packet->data[1] << 0x10
        | (uint32_t)rx_packet->data[2] << 8;

    return canBaudRate(&canConfig2, bitrate, NULL, NULL) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  }
  return false;
}
bool socketcan_filterHs(packet_t *rx_packet, packet_t *tx_packet) {
  if (rx_packet->data_len != 0)  {
    if ((rx_packet->data_len == 1) && (rx_packet->data[0] == 0)) {
      canGlobalFilter(&canConfig1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
      canStop(&CAND1);
      canStart(&CAND1, &canConfig1);;
    }
  }
  return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
}

bool socketcan_filterSw(packet_t *rx_packet, packet_t *tx_packet) {
  return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
}

bool exec_cmd_socketcan(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_socketcan_connect_hs:
    return socketcan_connectHs(rx_packet, tx_packet);
  case cmd_socketcan_bitrate_hs:
    return socketcan_bitrateHs(rx_packet, tx_packet);
  case cmd_socketcan_filter_hs:
    return socketcan_filterHs(rx_packet, tx_packet);
  case cmd_socketcan_tx_hscan:
    return socketcan_txFrameHs(rx_packet, tx_packet);
  case cmd_socketcan_connect_sw:
    return socketcan_connectSw(rx_packet, tx_packet);
  case cmd_socketcan_bitrate_sw:
    return socketcan_bitrateSw(rx_packet, tx_packet);
  case cmd_socketcan_filter_sw:
    return socketcan_filterSw(rx_packet, tx_packet);
  case cmd_socketcan_tx_swcan:
    return socketcan_txFrameSw(rx_packet, tx_packet);
  default:
    break;
  }
  return false;
}
