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
  .StdFiltersNbr = FILTER_NBR,
  .ExtFiltersNbr = FILTER_NBR,
  .RxFifo0ElmtsNbr = 8,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
  .RxFifo1ElmtsNbr = 8,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
  .RxBuffersNbr = 8,
  .RxBufferSize = FDCAN_DATA_BYTES_8,
  .TxEventsNbr = 1,
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

static CAN_FILTER swCf = {.msgRam = &swcan_ram, .index = 0, .filter = {{0}}};
static CAN_FILTER hsCf = {.msgRam = &hscan_ram, .index = 0, .filter = {{0}}};
static uint8_t retBuff[64] = { 0 };

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

j2534_conn connHs = {.canp = &CAND1,
                     .canCfg = &hsCanConfig,
                     .canFilter = &hsCf,
                     .ramCfg = &hscan_ram_cfg,
                     .ramAdr = &hscan_ram,
                     .cb = rx_can_msg
};

j2534_conn connSw = {.canp = &CAND2,
                     .canCfg = &swCanConfig,
                     .canFilter = &swCf,
                     .ramCfg = &swcan_ram_cfg,
                     .ramAdr = &swcan_ram,
                     .cb = rx_swcan_msg
};

j2534_conn* get_connection(uint32_t channel) {
  switch (channel) {
  case CAN:
  case CAN_PS:
  case ISO15765:
    return &connHs;
  case SW_CAN_PS:
    return &connSw;
  default:
    return NULL;
}
}

uint32_t registerCanCallback(j2534_conn* conn) {
  switch (conn->protocol) {
    case CAN:
    case CAN_PS:
    case ISO15765:
      registerHsCanCallback(conn->cb);
      break;
    case SW_CAN_PS:
      registerSwCanCallback(conn->cb);
      break;
    default:
      return ERR_INVALID_PROTOCOL_ID;
  }
  return STATUS_NOERROR;
}

uint32_t handle_connect(j2534_conn* conn, uint32_t protocol, uint32_t flags, uint32_t bitrate) {
  uint32_t err = ERR_NOT_SUPPORTED;
  conn->protocol = protocol;
  conn->flags = flags;
  conn->bitRate = bitrate;
  err = registerCanCallback(conn);

  if (err != STATUS_NOERROR) {
    return err;
  }

  if (!canBaudRate(conn->canCfg, conn->bitRate, &conn->syncJumpWidth, &conn->bitSamplePoint)) {
    return ERR_INVALID_BAUDRATE;
  }

  canMemorryConfig(conn->canp, conn->canCfg, conn->ramCfg, conn->ramAdr);
  canGlobalFilter(conn->canCfg, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  canStart(conn->canp, conn->canCfg);
  return STATUS_NOERROR;
}

bool j2534_connect(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t protocolID, flags, bitrate, error = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len != 12) {
    memcpy(retBuff, &error, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
  }
  memcpy(&protocolID, rx_packet->data, 4);
  memcpy(&flags, rx_packet->data + 4, 4);
  memcpy(&bitrate, rx_packet->data + 8, 4);

  j2534_conn* conn = get_connection(protocolID);
  error = handle_connect(conn, protocolID, flags, bitrate);

  memcpy(retBuff, &error, 4);
  memcpy(retBuff + 4, &protocolID, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 8, cmd_j2534_ack);
}

uint32_t handle_disconnect(j2534_conn* conn) {
  uint32_t err = ERR_NOT_SUPPORTED;
  conn->cb = NULL;
  err = registerCanCallback(conn);
  if (err != STATUS_NOERROR) {
    return err;
  }
  canStop(conn->canp);
  return err;
}

bool j2534_disconnect(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, error = ERR_NOT_SUPPORTED;
  if(rx_packet->data_len != 4) {
    memcpy(retBuff, &error, 4);
    return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_j2534_ack);
  }
  memcpy(&channelID, rx_packet->data, 4);

  j2534_conn* conn = get_connection(channelID);
  error = handle_disconnect(conn);
  memcpy(retBuff, &error, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_j2534_ack);
}

uint32_t handle_can_filter(j2534_conn* conn, uint8_t* data) {
  CAN_FILTER *cf = conn->canFilter;
  uint32_t flags;
  uint32_t pattern, mask;
  uint8_t size = data[11], type = data[10];
  memcpy(&flags, data + 4, 4);
  memcpy(&mask, data + 12, size);
  memcpy(&pattern, data + 24, size);
  (void)type;
  if (cf->index >= FILTER_NBR)
    return ERR_EXCEEDED_LIMIT;

  cf->filter[cf->index].IdType = (flags & CAN_29BIT_ID) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  cf->filter[cf->index].FilterIndex = cf->index;
  cf->filter[cf->index].FilterType = FDCAN_FILTER_MASK;
  cf->filter[cf->index].FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  cf->filter[cf->index].FilterID1 = pattern;
  cf->filter[cf->index].FilterID2 = mask;

  canFilter(cf->msgRam, &cf->filter[cf->index]);
  cf->index++;
  return STATUS_NOERROR;
}

uint32_t clear_can_filters(j2534_conn* conn) {
  CAN_FILTER *cf = conn->canFilter;
  memset(cf->filter, 0, FILTER_NBR-1);
  for(uint8_t i = 0; i < FILTER_NBR; i++) {
    cf->filter[i].FilterIndex = i;
    canFilter(cf->msgRam, &cf->filter[i]);
  }
  return STATUS_NOERROR;
}

uint32_t clear_can_filter(j2534_conn* conn, uint8_t idx) {
  CAN_FILTER *cf = conn->canFilter;
  memset(&cf->filter[idx], 0, sizeof(CAN_Filter));
  cf->filter[idx].FilterIndex = idx;
  canFilter(cf->msgRam, &cf->filter[idx]);
  return STATUS_NOERROR;
}

bool j2534_start_filter(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, error = ERR_NOT_SUPPORTED;
  uint8_t size = 4;
  memcpy(&channelID, rx_packet->data, 4);
  j2534_conn* conn = get_connection(channelID);
  error = handle_can_filter(conn, rx_packet->data);
  memcpy(retBuff, &error, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, size, cmd_j2534_ack);
}

bool j2534_stop_filter(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, filterID, error = ERR_NOT_SUPPORTED;
  uint8_t size = 4;
  memcpy(&channelID, rx_packet->data, 4);
  memcpy(&filterID, rx_packet->data+4, 4);
  j2534_conn* conn = get_connection(channelID);
  error = clear_can_filter(conn, filterID);
  memcpy(retBuff, &error, 4);
  return prepareReplyPacket(tx_packet, rx_packet, retBuff, size, cmd_j2534_ack);
}

uint32_t handldle_get_config(j2534_conn* conn, SCONFIG_LIST* cfgList) {
  uint32_t err = ERR_NOT_SUPPORTED;
  for (uint8_t i = 0; i < cfgList->NumOfParams; i++) {
    SCONFIG *cfg = &cfgList->ConfigPtr[i];
    switch (cfg->Parameter) {
    case DATA_RATE:
      cfg->Value = conn->bitRate;
      err = STATUS_NOERROR;
      break;
    case LOOPBACK:
      cfg->Value = conn->loopback;
      err = STATUS_NOERROR;
      break;
    case BIT_SAMPLE_POINT:
      cfg->Value = conn->bitSamplePoint;
      err = STATUS_NOERROR;
      break;
    case SYNC_JUMP_WIDTH:
      cfg->Value = conn->syncJumpWidth;
      err = STATUS_NOERROR;
      break;
    case CAN_MIXED_FORMAT:
      cfg->Value = conn->canMixedFormat;
      err = STATUS_NOERROR;
      break;
    case J1962_PINS:
      cfg->Value = conn->J1962Pins;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_HS_DATA_RATE:
      cfg->Value = conn->swCanHsDataRate;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_SPEEDCHANGE_ENABLE:
      cfg->Value = conn->swCanSpeedChangeEnable;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_RES_SWITCH:
      cfg->Value = conn->swCanResSwitch;
      err = STATUS_NOERROR;
      break;
    default:
      break;
    }
  }
  return err;
}

uint32_t handldle_set_config(j2534_conn* conn, SCONFIG_LIST* cfgList) {
  uint32_t err = ERR_NOT_SUPPORTED;
  for (uint8_t i = 0; i < cfgList->NumOfParams; i++) {
    SCONFIG *cfg = &cfgList->ConfigPtr[i];
    switch (cfg->Parameter) {
    case DATA_RATE:
      conn->bitRate = cfg->Value;
      canStop(conn->canp);
      if (!canBaudRate(conn->canCfg, conn->bitRate, &conn->syncJumpWidth, &conn->bitSamplePoint)) {
        return ERR_INVALID_BAUDRATE;
      }
      canStart(conn->canp, conn->canCfg);
      err = STATUS_NOERROR;
     break;
    case LOOPBACK:
      conn->loopback = cfg->Value == 1;
      if (conn->loopback) {
        canStop(conn->canp);
        hsCanConfig.CCCR |= FDCAN_CCCR_TEST | FDCAN_CCCR_MON;
        hsCanConfig.TEST |= FDCAN_TEST_LBCK;
        canStart(conn->canp, conn->canCfg);
      } else {
        canStop(conn->canp);
        hsCanConfig.CCCR &= ~FDCAN_CCCR_TEST | ~FDCAN_CCCR_MON;
        hsCanConfig.TEST &= ~FDCAN_TEST_LBCK;
        canStart(conn->canp, conn->canCfg);
      }
      err = STATUS_NOERROR;
      break;
    case CAN_MIXED_FORMAT:
      conn->canMixedFormat = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case J1962_PINS:
      conn->J1962Pins = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_HS_DATA_RATE:
      conn->swCanHsDataRate = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_SPEEDCHANGE_ENABLE:
      conn->swCanSpeedChangeEnable = cfg->Value;
      err = STATUS_NOERROR;
      break;
    case SW_CAN_RES_SWITCH:
      conn->swCanResSwitch = cfg->Value;
      err = STATUS_NOERROR;
      break;
    default:
      break;
    }
  }
  return err;
}

bool j2534_ioctl(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, ioctl, err = ERR_NOT_SUPPORTED;
  uint8_t retSize = 4;
  memcpy(&channelID, rx_packet->data, 4);
  memcpy(&ioctl, rx_packet->data + 4, 4);
  j2534_conn* conn = get_connection(channelID);

  switch(ioctl) {
  case GET_CONFIG:
  case SET_CONFIG:
    uint32_t pSize = 0;
    memcpy(&pSize, rx_packet->data + 8, 4);
    SCONFIG c[16] = {0}; //Handle better
    memcpy(c, rx_packet->data + 12, rx_packet->data_len-12);
    SCONFIG_LIST cfgList = {.NumOfParams = pSize, .ConfigPtr = c};
    if (ioctl == GET_CONFIG) {
      err = handldle_get_config(conn, &cfgList);
      memcpy(retBuff+4, &cfgList.NumOfParams, 4);
      memcpy(retBuff+8, cfgList.ConfigPtr, pSize*8);
      retSize = 4+pSize*8+4;
    } else {
      err = handldle_set_config(conn, &cfgList);
    }
    break;
  case READ_VBATT:
    uint16_t vBat = getSupplyVoltage();
    memcpy(retBuff + 4, &vBat, 2);
    retSize = 6;
    err = STATUS_NOERROR;
    break;
  case CLEAR_RX_BUFFER:
    err = STATUS_NOERROR;
    break;
  case CLEAR_MSG_FILTERS:
      clear_can_filters(conn);
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
  memcpy(retBuff, &err, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, retSize, cmd_j2534_ack);;
}

uint32_t write_message(j2534_conn* conn, uint32_t timeout, uint16_t len, uint8_t* data) {
  uint32_t err = ERR_NOT_SUPPORTED;
  CANTxFrame tx = {0};
  memcpy(&tx, data, len);
  if (canTransmit(conn->canp, CAN_ANY_MAILBOX, &tx, TIME_MS2I(timeout)) == MSG_OK) {
    err = STATUS_NOERROR;
  } else {
    err = ERR_TIMEOUT;
  }
  return err;
}

bool j2534_write_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, timeout, err = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 4);
  memcpy(&timeout, rx_packet->data + 4, 4);
  if (timeout == 0)
    timeout = 100;

  j2534_conn* conn = get_connection(channelID);
  err = write_message(conn, timeout, rx_packet->data_len-8, rx_packet->data+8);
  memcpy(retBuff, &err, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_j2534_ack);
}

uint32_t start_periodic_message(j2534_conn* conn) {
  (void)conn;
  return ERR_NOT_SUPPORTED;
}

bool j2534_start_periodic_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, err = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 4);
  j2534_conn* conn = get_connection(channelID);

  err = start_periodic_message(conn);
  memcpy(retBuff, &err, 4);

  return prepareReplyPacket(tx_packet, rx_packet, retBuff, 4, cmd_j2534_ack);
}

uint32_t stop_periodic_message(j2534_conn* conn) {
  (void)conn;
  return ERR_NOT_SUPPORTED;
}

bool j2534_stop_periodic_message(packet_t *rx_packet, packet_t *tx_packet) {
  uint32_t channelID, err = ERR_NOT_SUPPORTED;
  memcpy(&channelID, rx_packet->data, 4);
  j2534_conn* conn = get_connection(channelID);

  err = stop_periodic_message(conn);
  memcpy(retBuff, &err, 4);

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
    return j2534_start_filter(rx_packet, tx_packet);
  case cmd_j2534_stop_filter:
    return j2534_stop_filter(rx_packet, tx_packet);
  case cmd_j2534_write_message:
    return j2534_write_message(rx_packet, tx_packet);
  case cmd_j2534_start_periodic_message:
    return j2534_start_periodic_message(rx_packet, tx_packet);
  case cmd_j2534_stop_periodic_message:
    return j2534_stop_periodic_message(rx_packet, tx_packet);
  default:
    break;
  }
  return false;
}
