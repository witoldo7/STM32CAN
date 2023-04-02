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
#include "usbcombi.h"
#include "bdm.h"
#include "bdmcpu32.h"
#include <utils.h>

bool readflash(LONG start_addr, LONG size);
bool writeflash(char *flash_type, LONG start_addr, LONG size);

uint8_t version[2] = {0x03, 0x01};
uint8_t egt_temp[5] = {0};
const uint32_t CvtEltSize[] = {0, 0, 0, 0, 0, 1, 2, 3, 4, 0, 5, 0, 0, 0, 6, 0, 0, 0, 7};
CANTxFrame txmsg = {};

CAN_RamAddress can1_ram, can2_ram;

CAN_Filter filter0 = {
  .IdType = FDCAN_STANDARD_ID,
  .FilterIndex = 0,
  .FilterType = FDCAN_FILTER_DUAL,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
  .FilterID1 = 0x7E8,
  .FilterID2 = 0x7E0,
};

CAN_Filter filter1 = {
  .IdType = FDCAN_STANDARD_ID,
  .FilterIndex = 1,
  .FilterType = FDCAN_FILTER_DUAL,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
  .FilterID1 = 0x5E8,
  .FilterID2 = 0x311,
};

CAN_Filter filter2 = {
  .IdType = FDCAN_STANDARD_ID,
  .FilterIndex = 2,
  .FilterType = FDCAN_FILTER_MASK,
  .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
  .FilterID1 = 0x011,
  .FilterID2 = 0x7FF,
};

CANRamConfig can1_ram_cfg = {
  .MessageRAMOffset = 0,
  .StdFiltersNbr = 4,
  .ExtFiltersNbr = 4,
  .RxFifo0ElmtsNbr = 32,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
  .RxFifo1ElmtsNbr = 0,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
  .RxBuffersNbr = 16,
  .RxBufferSize = FDCAN_DATA_BYTES_8,
  .TxEventsNbr = 0,
  .TxBuffersNbr = 1,
  .TxFifoQueueElmtsNbr = 1,
  .TxElmtSize = FDCAN_DATA_BYTES_8
};

CANRamConfig hscan_ram_cfg1 = {
  .MessageRAMOffset = 0,
  .StdFiltersNbr = 8,
  .ExtFiltersNbr = 0,
  .RxFifo0ElmtsNbr = 8,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
  .RxFifo1ElmtsNbr = 8,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
  .RxBuffersNbr = 8,
  .RxBufferSize = FDCAN_DATA_BYTES_8,
  .TxEventsNbr = 2,
  .TxBuffersNbr = 8,
  .TxFifoQueueElmtsNbr = 8,
  .TxElmtSize = FDCAN_DATA_BYTES_8
};

CANRamConfig can2_ram_cfg = {
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
  .DBTP = 0,
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
  packet->data[4] = (rxmsg->common.XTD) | (rxmsg->common.RTR << 1) | (rxmsg->common.ESI << 2)
                  | (rxmsg->BRS << 3) | (rxmsg->FDF << 4) | (rxmsg->ANMF << 5);
  packet->data[5] = rxmsg->FIDX;
  packet->data[6] = rxmsg->RXTS & 0xFF;
  packet->data[7] = (rxmsg->RXTS >> 8) & 0xFF;
  packet->data[8] = rxmsg->DLC;
  memcpy(packet->data + 9, rxmsg->data8, can_fd_dlc2len(rxmsg->DLC));
  packet->cmd_code = cmd_can_rxframe_fdcan;
  packet->data_len = 73;
  return true;
}

bool exec_cmd_swcan(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_swcan_open:
    switch (rx_packet->data[0]) {
    case combi_close:
      canStop(&CAND2);
      break;
    case combi_open:
      uint32_t mode = rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
          | (uint32_t)rx_packet->data[3] << 8;
      canMemorryConfig(&CAND2, &canConfig2, &can2_ram_cfg, &can2_ram);
      canConfig2.TXESC = CvtEltSize[can2_ram_cfg.TxElmtSize]; // 8 Byte mode only (4 words per message)
      canConfig2.RXESC = CvtEltSize[can2_ram_cfg.RxFifo0ElmtSize] << FDCAN_RXESC_F0DS_Pos | CvtEltSize[can2_ram_cfg.RxFifo1ElmtSize] << FDCAN_RXESC_F1DS_Pos
             | CvtEltSize[can2_ram_cfg.RxBufferSize] << FDCAN_RXESC_RBDS_Pos;// 8 Byte mode only (4 words per message)

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
      can2_ram.ExtendedFilterSA = 0;
      can2_ram.StandardFilterSA = 0;
      palSetLine(LINE_SWM0);
      palSetLine(LINE_SWM1);
      break;
    default:
      return false;
    }
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    break;

  case cmd_swcan_bitrate:
    if (rx_packet->data_len == 4) {
      uint32_t bitrate = rx_packet->data[3] | (uint32_t)*rx_packet->data << 0x18 | (uint32_t)rx_packet->data[1] << 0x10
          | (uint32_t)rx_packet->data[2] << 8;

      return canBaudRate(&canConfig2, bitrate) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
   break;

  case cmd_swcan_txframe:
    if (rx_packet->data_len != 15) {
      return false;
    }
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

    return (canTransmit(&CAND2, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100)) == MSG_OK )
        && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  break;

  case cmd_swcan_filter:
    if (*rx_packet->data == 0x0) {
      return false;
    }
    //set_can_filter(&CAND2, rx_packet->data);
    break;
  }
  return false;
}

bool exec_cmd_can(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_can_ecuconnect:
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);

  case cmd_can_filter:
    if (rx_packet->data_len != 0)  {
      if ((rx_packet->data_len == 1) && (rx_packet->data[0] == 0)) {
        canGlobalFilter(&canConfig1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
        canStop(&CAND1);
        canStart(&CAND1, &canConfig1);;
      }
    }
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);

  case cmd_can_open:
    switch (rx_packet->data[0]) {
    case combi_close:
      canStop(&CAND1);
      break;
    case combi_open:
      registerHsCanCallback(&combi_rx_can_cb);
      canMemorryConfig(&CAND1, &canConfig1, &can1_ram_cfg, &can1_ram);
      canConfig2.TXESC = CvtEltSize[can1_ram_cfg.TxElmtSize]; // 8 Byte mode only (4 words per message)
      canConfig2.RXESC = CvtEltSize[can1_ram_cfg.RxFifo0ElmtSize] << FDCAN_RXESC_F0DS_Pos | CvtEltSize[can2_ram_cfg.RxFifo1ElmtSize] << FDCAN_RXESC_F1DS_Pos
             | CvtEltSize[can2_ram_cfg.RxBufferSize] << FDCAN_RXESC_RBDS_Pos;// 8 Byte mode only (4 words per message)
      canGlobalFilter(&canConfig1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
      canConfig1.CCCR |= FDCAN_CCCR_DAR;
      canStart(&CAND1, &canConfig1);
      //Combilib + Trionic 8 filters
      canFilter(&can1_ram, &filter0);
      canFilter(&can1_ram, &filter1);
      canFilter(&can1_ram, &filter2);
      break;
    case fdcan_open:
      registerHsCanCallback(&socketcan_rx_can_cb);
      canConfig1.TEST = 0;
      canConfig1.TXESC = CvtEltSize[hscan_ram_cfg1.TxElmtSize]; // 8 Byte mode only (4 words per message)
      canConfig1.RXESC = CvtEltSize[hscan_ram_cfg1.RxFifo0ElmtSize] << FDCAN_RXESC_F0DS_Pos | CvtEltSize[hscan_ram_cfg1.RxFifo1ElmtSize] << FDCAN_RXESC_F1DS_Pos
             | CvtEltSize[hscan_ram_cfg1.RxBufferSize] << FDCAN_RXESC_RBDS_Pos;// 8 Byte mode only (4 words per message)
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
      canMemorryConfig(&CAND1, &canConfig1, &hscan_ram_cfg1, &can1_ram);
      canGlobalFilter(&canConfig1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
      canStart(&CAND1, &canConfig1);
      break;
    default:
      return false;
    }
    return prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    break;

  case cmd_can_bitrate:
    switch (rx_packet->data_len) {
    case 4:
      uint32_t bitrate = rx_packet->data[3] | (uint32_t)*rx_packet->data << 0x18 | (uint32_t)rx_packet->data[1] << 0x10
          | (uint32_t)rx_packet->data[2] << 8;
      bool ret = canBaudRate(&canConfig1, bitrate);
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

  case cmd_can_txframe:
    if (rx_packet->data_len != 15) {
      return false;
    }
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

  /* data[0:3] - SID or EID
   * data[4]
   *    b0 - XTD
   *    b1 - RTR
   *    b2 - ESI
   *    b3 - BRS
   *    b4 - FDF
   *    b5 - EFC
   * data[5] - MM
   * data[6] - DLC
   * data[7:7+DLC] - data
   */
  case cmd_can_txframe_fdcan:
    uint8_t flags = rx_packet->data[4];
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

bool exec_cmd_bdm(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_bdm_stop_chip:
    return (stop_chip() == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_reset_chip:
    return (reset_chip() == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_run_chip:
    if (rx_packet->data_len == 4) {
      uint32_t addr = rx_packet->data[3] | (uint32_t)rx_packet->data[0] << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8;
      return (run_chip(&addr) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    break;
  case cmd_bdm_step_chip:
    return (step_chip() == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_restart_chip:
    return (restart_chip() == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_mem_read:
    if (rx_packet->data_len < 2) {
      return false;
    }
    if (rx_packet->data[0] == 1) {
      uint8_t ret[2];
      if (rx_packet->data[1] == 0) {
        return (memdump_byte(ret) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, ret, 1, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_byte(ret, &addr) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, ret, 1, cmd_term_ack);;
      }
      return false;
    }
    if (rx_packet->data[0] == 2) {
      uint8_t ret[2];
      if (rx_packet->data[1] == 0) {
        return (memdump_word((WORD*)ret) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, ret, 2, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_word((WORD*)ret, &addr) == TERM_OK)
            && prepareReplyPacket(tx_packet, rx_packet, ret, 2, cmd_term_ack);;
      }
      return false;
    }
    if (rx_packet->data[0] == 4) {
      uint8_t ret[4];
      if (rx_packet->data[1] == 0) {
        return (memdump_long((LONG*)ret) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_long((LONG*)ret, &addr) == TERM_OK)
            && prepareReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);;
      }
      return false;
    }
    return false;
  case cmd_bdm_mem_write:
    if (rx_packet->data_len == 5) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      return (memwrite_byte(&addr, rx_packet->data[4]) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    if (rx_packet->data_len == 6) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      WORD data = rx_packet->data[4] << 8 | rx_packet->data[5];
      return (memwrite_word(&addr, data) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    if (rx_packet->data_len == 8) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      LONG data = (uint32_t)rx_packet->data[7] | (uint32_t)rx_packet->data[4] << 24 | (uint32_t)rx_packet->data[5] << 16
          | (uint32_t)rx_packet->data[6] << 8;
      return (memwrite_long(&addr, &data) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    return false;
  case cmd_bdm_sysreg_read:
    if (rx_packet->data_len == 1) {
      uint8_t ret[4];
      return (sysreg_read((LONG*)ret, rx_packet->data[0]) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
    }
    return false;
  case cmd_bdm_sysreg_write:
    if (rx_packet->data_len == 5) {
      LONG data = (uint32_t)rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
          | (uint32_t)rx_packet->data[3] << 8;

      return (sysreg_write(rx_packet->data[0], &data) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    return false;
  case cmd_bdm_adreg_read:
    if (rx_packet->data_len == 1) {
      uint8_t ret[4];
      return (adreg_read((LONG*)ret, rx_packet->data[0]) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
    }
    return false;
  case cmd_bdm_adreg_write:
    if (rx_packet->data_len == 5) {
      LONG data = (uint32_t)rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
          | (uint32_t)rx_packet->data[3] << 8;

      return (adreg_write(rx_packet->data[0], &data) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    return false;
  case cmd_bdm_read_flash:
    if (rx_packet->data_len == 8) {
      uint32_t addr = (uint32_t)rx_packet->data[0] << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      uint32_t size = (uint32_t)rx_packet->data[7] | (uint32_t)rx_packet->data[4] << 24
          | (uint32_t)rx_packet->data[5] << 16 | (uint32_t)rx_packet->data[6] << 8;
      return readflash(addr, size);
    }
    return false;
  case cmd_bdm_erase_flash:
    if (rx_packet->data_len == 14) {
      const char flash_type = (char)rx_packet->data[0];
      LONG start_addr = (uint32_t)rx_packet->data[6] << 24 | (uint32_t)rx_packet->data[7] << 16
          | (uint32_t)rx_packet->data[8] << 8 | (uint32_t)rx_packet->data[9];
      LONG end_addr = (uint32_t)rx_packet->data[0xd] | (uint32_t)rx_packet->data[10] << 24
          | (uint32_t)rx_packet->data[0xb] << 16 | (uint32_t)rx_packet->data[0xc] << 8;
      return erase_flash(&flash_type, &start_addr, &end_addr);
    }
    return false;
  case cmd_bdm_write_flash:
    if (rx_packet->data_len == 14) {
      char flash_type = (char)rx_packet->data[0];
      uint32_t start_addr = (uint32_t)rx_packet->data[6] << 24 | (uint32_t)rx_packet->data[7] << 16
          | (uint32_t)rx_packet->data[8] << 8 | (uint32_t)rx_packet->data[9];
      uint32_t end_addr = (uint32_t)rx_packet->data[0xd] | (uint32_t)rx_packet->data[10] << 24
          | (uint32_t)rx_packet->data[0xb] << 16 | (uint32_t)rx_packet->data[0xc] << 8;
      return writeflash(&flash_type, start_addr, end_addr);
    }
    return false;
  case cmd_bdm_pinstate:
    uint8_t pin = 1; //PIN_PWR.read(); TODO
    return (pin == 1) && prepareReplyPacket(tx_packet, rx_packet, &pin, 1, cmd_term_ack);
  }

  return false;
}

bool readflash(LONG start_addr, LONG size) {
  char result;
  WORD curr_word;
  WORD *buf_ptr;
  LONG curr_addr;
  uint8_t flash_buf[256];
  packet_t tx_packet, rx_packet;
  uint8_t buffer[300] = {0};
  uint8_t buffsize = 0;

  tx_packet.cmd_code = cmd_bdm_read_flash;
  tx_packet.data_len = 256;
  tx_packet.data = flash_buf;
  tx_packet.term = cmd_term_ack;
  buf_ptr = (WORD*)flash_buf;
  curr_addr = start_addr;
  while (curr_addr < start_addr + size) {
   // status = CombiReceivePacket(&rx_packet, 0);
    if ((rx_packet.cmd_code == cmd_bdm_read_flash) && (rx_packet.term == cmd_term_nack)) {
      return false;
    }
    if (curr_addr == start_addr) {
      result = memread_word(&curr_word, &curr_addr);
    }
    else {
      result = memdump_word(&curr_word);
    }
    if (result != TERM_OK) {
      return false;
    }
    swab(&curr_word);
    *buf_ptr = curr_word;
    buf_ptr = buf_ptr + 1;
    curr_addr = curr_addr + 2;
    if (((curr_addr - start_addr) & 0xff) == 0) {
      buffsize = covertPacketToBuffer(&tx_packet, buffer);
      usb_send(&USBD1, EP_IN, buffer, buffsize);
      buf_ptr = (WORD*)flash_buf;
    }
  }

  return result == TERM_OK;
}

bool writeflash(char *flash_type, LONG start_addr, LONG size) {
  packet_t tx_packet, rx_packet;
  bool status;
  WORD curr_word;
  WORD *buf_ptr;
  uint8_t flash_buf[32];
  uint32_t bytes_written;
  bool (*reset_func)(void);
  bool (*flash_func)(const uint32_t*, uint16_t);
  uint8_t buffer[IN_PACKETSIZE] = {0};
  uint8_t buffsize = 0;

  if (strncmp(flash_type, "29f010", 6) == 0 || strncmp(flash_type, "29f400", 6) == 0) {
    reset_func = &reset_am29;
    flash_func = &flash_am29;
  }
  else if (strncmp(flash_type, "28f010", 6) == 0) {
    reset_func = &reset_am28;
    flash_func = &flash_am28;
  }
  else {
    return false;
  }

  // reset the flash
  if (!reset_func()) {
    return false;
  }

  uint32_t curr_addr = start_addr;
  if (strncmp(flash_type, "29f010", 6) == 0) {
    curr_addr = 0;
  }

  tx_packet.cmd_code = cmd_bdm_write_flash;
  tx_packet.data_len = 0;
  tx_packet.data = (BYTE*)0x0;
  tx_packet.term = cmd_term_ack;
  buffsize = covertPacketToBuffer(&tx_packet, buffer);
  usb_send(&USBD1, EP_IN, buffer, buffsize);

  rx_packet.data = flash_buf;
  bytes_written = 0;
  do {
    if (size <= bytes_written) {
      status = reset_func();
      if (status == true) {
        return true;
      }
      reset_chip();
      return false;
    }
    //status = CombiReceivePacket(&rx_packet, 1000);
    if ((((rx_packet.cmd_code != cmd_bdm_write_flash)) || (rx_packet.term == cmd_term_nack)) || (rx_packet.data_len != 256)) {
      return false;
    }
    buf_ptr = (WORD*)flash_buf;
    for (uint16_t byte_cnt = 0; byte_cnt < 0x100; byte_cnt = byte_cnt + 2) {
      swab(buf_ptr);
      curr_word = *buf_ptr;
      buf_ptr = buf_ptr + 1;
      status = flash_func(&curr_addr, curr_word);
      if (status != true) {
        reset_chip();
        return false;
      }
      curr_addr = curr_addr + 2;
    }
    bytes_written = bytes_written + 0x100;
    buffsize = covertPacketToBuffer(&tx_packet, buffer);
    usb_send(&USBD1, EP_IN, buffer, buffsize);
  } while (status == true);

  // reset flash
  return (reset_func() && status);
}
