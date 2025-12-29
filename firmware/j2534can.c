// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */
#include <string.h>
#include "j2534.h"
#include "j2534can.h"
#include "debug.h"

static CANConfig hsCanConfig = {
  OPMODE_CAN,
  //OPMODE_FDCAN,                  /* OP MODE */
  0,                               /* NBTP */
  0,                               /* DBTP */
  0,                               /* TDCR */
  0,                               /* CCCR */
  0,                               /* TEST */
  0                                /* GFC */
};
static uint32_t hs_cf_index = 0;
static CANFilter hs_filters[FILTER_NBR] = {0};

j2534_can_cfg canCfgHs = {
  .canp = &CAND1,
  .canCfg = &hsCanConfig,
  .cf_index = &hs_cf_index,
  .filters = hs_filters,
};

static CANConfig swCanConfig = {
  OPMODE_CAN,
  //OPMODE_FDCAN,                  /* OP MODE */
  0,                               /* NBTP */
  0,                               /* DBTP */
  0,                               /* TDCR */
  0,                               /* CCCR */
  0,                               /* TEST */
  0                                /* GFC */
};
static uint32_t sw_cf_index = 0;
static CANFilter sw_filters[FILTER_NBR] = {0};

j2534_can_cfg canCfgSw = {
  .canp = &CAND2,
  .canCfg = &swCanConfig,
  .cf_index = &sw_cf_index,
  .filters = sw_filters,
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
  j2534_can_cfg *can = conn->cfg;
  uint32_t cf_index = *can->cf_index;
  uint32_t pattern, mask, flags;
  uint8_t size = data[11], type = data[10];
  memcpy(&flags, data + 4, 4);
  memcpy(&mask, data + 12, size);
  memcpy(&pattern, data + 24, size);

  switch (type)
  {
  case PASS_FILTER:
    can->filters[cf_index].filter_cfg = CAN_FILTER_CFG_FIFO_0;
    break;
  case BLOCK_FILTER:
    can->filters[cf_index].filter_cfg = CAN_FILTER_CFG_REJECT;
    break;
  
  default:
    return ERR_NOT_SUPPORTED;
    break;
  }
  
  if(flags & CAN_29BIT_ID) {
    can->filters[cf_index].filter_type = CAN_FILTER_TYPE_EXT;
  } else {
    can->filters[cf_index].filter_type = CAN_FILTER_TYPE_STD;
  }
  can->filters[cf_index].filter_mode = CAN_FILTER_MODE_CLASSIC;
  can->filters[cf_index].identifier1 = pattern;
  can->filters[cf_index].identifier2 = mask;
  *idx = cf_index++;
  canSTM32SetFilters(can->canp, cf_index, can->filters);
  return STATUS_NOERROR;
}

uint32_t ioctl_clear_filters_can(j2534_conn* conn) {
  j2534_can_cfg *can = conn->cfg;
  CANFilter filter[FILTER_NBR] = {0};
  can->filters = filter;
  *can->cf_index = 0;
  canSTM32SetFilters(can->canp, FILTER_NBR, can->filters);

  return STATUS_NOERROR;
}

uint32_t stop_filter_can(j2534_conn* conn, uint32_t idx) {
  j2534_can_cfg *can = conn->cfg;
  can->filters[idx].filter_cfg = 0;
  can->filters[idx].filter_type = 0;
  can->filters[idx].filter_mode = 0;
  can->filters[idx].identifier1 = 0;
  can->filters[idx].identifier2 = 0;
  canSTM32SetFilters(can->canp, idx, can->filters);
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
