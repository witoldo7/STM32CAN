// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#include <string.h>
#include "hal.h"
#include "j2534.h"

CAN_RamAddress hscan_ram, swcan_ram;

CANRamConfig hscan_ram_cfg = {
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

CANRamConfig swcan_ram_cfg = {
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

static CANConfig hsCanConfig = {
  .DBTP =  0,
  .CCCR =  0, //FDCAN_CCCR_TEST,
  .TEST =  0, //FDCAN_TEST_LBCK,
};

static CANConfig swCanConfig = {
  .DBTP = 0,
  .CCCR =  0, //FDCAN_CCCR_TEST,
  .TEST =  0, //FDCAN_TEST_LBCK,
};

CAN_Filter hsFilter[32] = { 0 };
static uint8_t hscanFilterIdx = 0;

static uint8_t retBuff[32] = {0};

bool rx_can_msg(CANRxFrame *rxmsg, packet_t *packet) {
  packet->cmd_code = cmd_j2534_read_message;
  uint16_t protocol = CAN;
  memcpy(packet->data, &protocol, 2);
  memcpy(packet->data + 2, rxmsg, 8 + rxmsg->DLC);
  packet->data_len = 10 + rxmsg->DLC;
  return true;
}

bool rx_swcan_msg(CANRxFrame *rxmsg, packet_t *packet) {
  packet->cmd_code = cmd_j2534_read_message;
  uint16_t protocol = SW_CAN_PS;
  memcpy(packet->data, &protocol, 2);
  memcpy(packet->data + 2, rxmsg, 10 + rxmsg->DLC);
  packet->data_len = 10 + rxmsg->DLC;
  return true;
}

uint32_t handle_hscan_connect(uint32_t flags, uint32_t baudrate) {
  (void)flags; //TODO
  registerHsCanCallback(&rx_can_msg);
  if (!canBaudRate(&hsCanConfig, baudrate)) {
    return ERR_INVALID_BAUDRATE;
  }
  canMemorryConfig(&CAND1, &hsCanConfig, &hscan_ram_cfg, &hscan_ram);
  hsCanConfig.TXESC = CvtEltSize(hscan_ram_cfg.TxElmtSize); // 8 Byte mode only (4 words per message)
  hsCanConfig.RXESC = CvtEltSize(hscan_ram_cfg.RxFifo0ElmtSize) << FDCAN_RXESC_F0DS_Pos | CvtEltSize(hscan_ram_cfg.RxFifo1ElmtSize) << FDCAN_RXESC_F1DS_Pos
         | CvtEltSize(hscan_ram_cfg.RxBufferSize) << FDCAN_RXESC_RBDS_Pos;// 8 Byte mode only (4 words per message)
  canGlobalFilter(&hsCanConfig, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  canStart(&CAND1, &hsCanConfig);
  return STATUS_NOERROR;
}

uint32_t handle_swcan_connect(uint32_t flags, uint32_t baudrate) {
  (void)flags; //TODO
  registerSwCanCallback(&rx_swcan_msg);
  if (!canBaudRate(&swCanConfig, baudrate)) {
    return ERR_INVALID_BAUDRATE;
  }
  canMemorryConfig(&CAND2, &swCanConfig, &swcan_ram_cfg, &swcan_ram);
  canGlobalFilter(&swCanConfig, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  canStart(&CAND2, &swCanConfig);
  return STATUS_NOERROR;
}

bool j2534_connect(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t protocolID, flags, baud, error = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len != 12) {
    memcpy(retBuff, &error, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
  }
  memcpy(&protocolID, rx_packet->data, 4);
  memcpy(&flags, rx_packet->data + 4, 4);
  memcpy(&baud, rx_packet->data + 8, 4);

  switch (protocolID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    error = handle_hscan_connect(flags, baud);
    break;
  case SW_CAN_PS:
    error = handle_swcan_connect(flags, baud);
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 4);
  memcpy(retBuff + 4, &protocolID, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
}

uint32_t handle_hscan_disconnect(void) {
  canStop(&CAND1);
  return STATUS_NOERROR;
}

uint32_t handle_swcan_disconnect(void) {
  canStop(&CAND2);
  return STATUS_NOERROR;
}

bool j2534_disconnect(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, error = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len != 4) {
    memcpy(retBuff, &error, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_j2534_ack);
  }
  memcpy(&channelID, rx_packet->data, 4);
  //Same mapping channelId == protocolID
  switch (channelID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    error = handle_hscan_disconnect();
    break;
  case SW_CAN_PS:
    error = handle_swcan_disconnect();
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_j2534_ack);
}

uint32_t handle_hscan_filter(uint8_t* data) {
  uint32_t flags;
  uint32_t pattern, mask;
  uint8_t size = data[11], type = data[10];
  memcpy(&flags, data + 4, 4);
  memcpy(&mask, data + 12, size);
  memcpy(&pattern, data + 24, size);
  (void)type;

  hsFilter[hscanFilterIdx].IdType = (flags & CAN_29BIT_ID) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  hsFilter[hscanFilterIdx].FilterIndex = hscanFilterIdx;
  hsFilter[hscanFilterIdx].FilterType = FDCAN_FILTER_MASK;
  hsFilter[hscanFilterIdx].FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  hsFilter[hscanFilterIdx].FilterID1 = pattern;
  hsFilter[hscanFilterIdx].FilterID2 = mask;

  canFilter(&hscan_ram, &hsFilter[hscanFilterIdx]);
  hscanFilterIdx++;
  return STATUS_NOERROR;
}

uint32_t handle_swcan_filter(void) {
  canStop(&CAND2);
  canGlobalFilter(&swCanConfig, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  canStart(&CAND2, &swCanConfig);
  return STATUS_NOERROR;
}

bool j2534_filter(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, error = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 4);
  //Same mapping channelId == protocolID
  switch (channelID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    error = handle_hscan_filter(rx_packet->data);
    break;
  case SW_CAN_PS:
    error = handle_swcan_filter();
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 4);
  memcpy(retBuff + 4, &hscanFilterIdx, 1);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 5, cmd_j2534_ack);
}

bool j2534_ioctl(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, ioctl, error = ERR_NOT_SUPPORTED;
  uint8_t retSize = 4;
  memcpy(&channelID, rx_packet->data, 4);
  memcpy(&ioctl, rx_packet->data + 4, 4);

  switch(ioctl) {
  case GET_CONFIG:
    //const SCONFIG_LIST *inputlist = pInput;
    break;
  case SET_CONFIG:
    //const SCONFIG_LIST *inputlist = pInput;
    break;
  case READ_VBATT:
    uint16_t vBat = getSupplyVoltage();
    memcpy(retBuff + 4, &vBat, 2);
    retSize = 6;
    error = STATUS_NOERROR;
    break;
  case CLEAR_RX_BUFFER:
    error = STATUS_NOERROR;
    break;
  case CLEAR_MSG_FILTERS:
    if (channelID == CAN) {
      memset(hsFilter, 0, sizeof(hsFilter));
      for (uint8_t i = 0; i < hscan_ram_cfg.StdFiltersNbr; i++)
        canFilter(&hscan_ram, &hsFilter[hscanFilterIdx]);
      error = STATUS_NOERROR;
    }
    break;
  case FIVE_BAUD_INIT:
  case FAST_INIT:
  case CLEAR_TX_BUFFER:
  case CLEAR_PERIODIC_MSGS:
  case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
  case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
  case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
  case READ_PROG_VOLTAGE:
  default:
      break;
  }
  memcpy(retBuff, &error, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, retSize, cmd_j2534_ack);;
}

bool j2534_write_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, timeout, error = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 4);
  memcpy(&timeout, rx_packet->data + 4, 4);
  if (timeout == 0)
    timeout = 100;

  switch (channelID) {
  case CAN:
  case CAN_PS:
    CANTxFrame tx = {0};
    memcpy(&tx, rx_packet->data + 8, rx_packet->data_len - 8);
    if (canTransmit(&CAND1, CAN_ANY_MAILBOX, &tx, TIME_MS2I(timeout)) == MSG_OK) {
      error = STATUS_NOERROR;
    } else {
      error = ERR_TIMEOUT;
    }
    break;
  case SW_CAN_PS:
    CANTxFrame swtx = {0};
    memcpy(&swtx, rx_packet->data + 8, rx_packet->data_len - 8);
    if (canTransmit(&CAND2, CAN_ANY_MAILBOX, &swtx, TIME_MS2I(timeout)) == MSG_OK) {
      error = STATUS_NOERROR;
    } else {
      error = ERR_TIMEOUT;
    }
    break;
  case ISO15765:
  default:
    break;
  }
  memcpy(retBuff, &error, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_j2534_ack);
}

bool j2534_periodic_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, error = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 4);
  //Same mapping channelId == protocolID
  switch (channelID) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    //error = handle_hscan_filter();
    break;
  case SW_CAN_PS:
    //error = handle_swcan_filter();
    break;
  default:
    break;
  }
  memcpy(retBuff, &error, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_j2534_ack);
}

bool exec_cmd_j2534(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_j2534_connect:
    return j2534_connect(rx_packet, tx_packet);
  case cmd_j2534_disconnect:
    return j2534_disconnect(rx_packet, tx_packet);
  case cmd_j2534_ioctl:
    return j2534_ioctl(rx_packet, tx_packet);
  case cmd_j2534_filter:
    return j2534_filter(rx_packet, tx_packet);
  case cmd_j2534_write_message:
    return j2534_write_message(rx_packet, tx_packet);
  case cmd_j2534_periodic_message:
    return j2534_periodic_message(rx_packet, tx_packet);
  default:
    break;
  }
  return false;
}
