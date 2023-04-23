// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */
#include <string.h>
#include "j2534.h"
#include "j2534can.h"

CAN_RamAddress hscan_ram, swcan_ram;

CANRamConfig hscan_ram_cfg = {
  .MessageRAMOffset = 0,
  .StdFiltersNbr = FILTER_NBR,
  .ExtFiltersNbr = FILTER_NBR,
  .RxFifo0ElmtsNbr = 32,
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
   //fixme
  .MessageRAMOffset = 0,
  .StdFiltersNbr = FILTER_NBR,
  .ExtFiltersNbr = FILTER_NBR,
  .RxFifo0ElmtsNbr = 2,
  .RxFifo0ElmtSize = FDCAN_DATA_BYTES_8,
  .RxFifo1ElmtsNbr = 2,
  .RxFifo1ElmtSize = FDCAN_DATA_BYTES_8,
  .RxBuffersNbr = 2,
  .RxBufferSize = FDCAN_DATA_BYTES_8,
  .TxEventsNbr = 1,
  .TxBuffersNbr = 1,
  .TxFifoQueueElmtsNbr = 1,
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

j2534_can_cfg canCfgHs = {.canp = &CAND1,
                     .canCfg = &hsCanConfig,
                     .canFilter = &hsCf,
                     .ramCfg = &hscan_ram_cfg,
                     .ramAdr = &hscan_ram
};

j2534_can_cfg canCfgSw = {.canp = &CAND2,
                     .canCfg = &swCanConfig,
                     .canFilter = &swCf,
                     .ramCfg = &swcan_ram_cfg,
                     .ramAdr = &swcan_ram
};


bool rx_hscan_msg(void *rxmsg, packet_t *packet) {
  CANRxFrame *msg = (CANRxFrame*) rxmsg;
  packet->cmd_code = cmd_j2534_read_message;
  uint16_t protocol = CAN;
  memcpy(packet->data, &protocol, 2);
  memcpy(packet->data + 2, msg, 8 + msg->DLC);
  packet->data_len = 10 + msg->DLC;
  return true;
}

bool rx_swcan_msg(void *rxmsg, packet_t *packet) {
  CANRxFrame *msg = (CANRxFrame*) rxmsg;
  packet->cmd_code = cmd_j2534_read_message;
  uint16_t protocol = SW_CAN_PS;
  memcpy(packet->data, &protocol, 2);
  memcpy(packet->data + 2, msg, 10 + msg->DLC);
  packet->data_len = 10 + msg->DLC;
  return true;
}

uint32_t updateConfig(j2534_conn* conn) {
  switch (conn->protocol) {
    case CAN:
    case CAN_PS:
    case ISO15765:
      conn->cfg = (void*)&canCfgHs;
      break;
    case SW_CAN_PS:
      conn->cfg = (void*)&canCfgSw;
      break;
    default:
      return ERR_INVALID_PROTOCOL_ID;
  }
  return STATUS_NOERROR;
}

uint32_t registerCallback(j2534_conn* conn) {
  switch (conn->protocol) {
    case CAN:
    case CAN_PS:
    case ISO15765:
      registerHsCanCallback(rx_hscan_msg);
      break;
    case SW_CAN_PS:
      registerSwCanCallback(rx_swcan_msg);
      break;
    default:
      return ERR_INVALID_PROTOCOL_ID;
  }
  return STATUS_NOERROR;
}

uint32_t removeCallback(j2534_conn* conn) {
  switch (conn->protocol) {
    case CAN:
    case CAN_PS:
    case ISO15765:
      registerHsCanCallback(NULL);
      break;
    case SW_CAN_PS:
      registerSwCanCallback(NULL);
      break;
    default:
      return ERR_INVALID_PROTOCOL_ID;
  }
  return STATUS_NOERROR;
}

uint32_t handle_connect_can(j2534_conn* conn) {
  uint32_t err = updateConfig(conn);
  if (err != STATUS_NOERROR) {
    return err;
  }
  j2534_can_cfg* can = conn->cfg;
  if (!canBaudRate(can->canCfg, conn->DataRate, &conn->pcfg->SyncJumpWidth, &conn->pcfg->BitSamplePoint)) {
    return ERR_INVALID_BAUDRATE;
  }

  canMemorryConfig(can->canp, can->canCfg, can->ramCfg, can->ramAdr);
  canGlobalFilter(can->canCfg, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  ioctl_clear_filters_can(conn);
  err = registerCallback(conn);
  if (err != STATUS_NOERROR) {
    return err;
  }
  canStart(can->canp, can->canCfg);
  return STATUS_NOERROR;
}

uint32_t handle_disconnect_can(j2534_conn* conn) {
  j2534_can_cfg* can = conn->cfg;
  canStop(can->canp);
  uint32_t err = removeCallback(conn);
  conn->isConnected = false;
  return err;
}

uint32_t start_filter_can(j2534_conn* conn, uint8_t* data, uint32_t* idx) {
  CAN_FILTER *cf = ((j2534_can_cfg*)conn->cfg)->canFilter;
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
  *idx = cf->index;
  cf->index++;
  return STATUS_NOERROR;
}

uint32_t ioctl_clear_filters_can(j2534_conn* conn) {
  j2534_can_cfg *can = conn->cfg;
  CAN_FILTER *cf = can->canFilter;
  memset(cf->filter, 0, FILTER_NBR-1);
  for(uint8_t i = 0; i < FILTER_NBR; i++) {
    cf->filter[i].FilterIndex = i;
    canFilter(cf->msgRam, &cf->filter[i]);
  }
  cf->index = 0;
  return STATUS_NOERROR;
}

uint32_t stop_filter_can(j2534_conn* conn, uint32_t idx) {
  CAN_FILTER *cf = ((j2534_can_cfg*)conn->cfg)->canFilter;
  memset(&cf->filter[idx], 0, sizeof(CAN_Filter));
  cf->filter[idx].FilterIndex = idx;
  canFilter(cf->msgRam, &cf->filter[idx]);
  return STATUS_NOERROR;
}

uint32_t write_message_can(j2534_conn* conn, uint32_t timeout, uint16_t len, uint8_t* data) {
  uint32_t err = ERR_NOT_SUPPORTED;
  if(len < 8) {
    return err;
  }
  CANTxFrame *tx = (CANTxFrame*)data;
  j2534_can_cfg* can = conn->cfg;
  if (canTransmit(can->canp, CAN_ANY_MAILBOX, tx, TIME_MS2I(timeout)) == MSG_OK) {
    err = STATUS_NOERROR;
  } else {
    err = ERR_TIMEOUT;
  }
  return err;
}

uint32_t start_periodic_can_msg(j2534_conn* conn, uint8_t data) {
  (void)conn;
  (void)data;
  return ERR_NOT_SUPPORTED;
}

uint32_t stop_periodic_can_msg(j2534_conn* conn, uint32_t msg) {
  (void)conn;
  (void)msg;
  return ERR_NOT_SUPPORTED;
}

uint32_t ioctl_datarate_can(j2534_conn* conn) {
  j2534_protocol_cfg *pcfg = conn->pcfg;
  j2534_can_cfg *can = conn->cfg;
  canStop(can->canp);
  if (!canBaudRate(can->canCfg, conn->DataRate, &pcfg->SyncJumpWidth, &pcfg->BitSamplePoint)) {
    return ERR_INVALID_BAUDRATE;
  }
  canStart(can->canp, can->canCfg);
  return STATUS_NOERROR;
}

uint32_t ioctl_loopback_can(j2534_conn* conn) {
  j2534_protocol_cfg *pcfg = conn->pcfg;
  j2534_can_cfg *can = conn->cfg;
  canStop(can->canp);
  if (pcfg->Loopback == 1) {
    can->canCfg->CCCR |= FDCAN_CCCR_TEST | FDCAN_CCCR_MON;
    can->canCfg->TEST |= FDCAN_TEST_LBCK;
  } else {
    can->canCfg->CCCR &= ~FDCAN_CCCR_TEST | ~FDCAN_CCCR_MON;
    can->canCfg->TEST &= ~FDCAN_TEST_LBCK;
  }
  canStart(can->canp, can->canCfg);
  return STATUS_NOERROR;
}
