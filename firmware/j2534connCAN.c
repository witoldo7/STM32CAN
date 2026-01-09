// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */
#include <string.h>
#include "j2534.h"
#include "j2534connCAN.h"
#include "debug.h"

CANConfig hsCanConfig = {
    OPMODE_CAN,
    // OPMODE_FDCAN,                  /* OP MODE */
    0, /* NBTP */
    0, /* DBTP */
    0, /* TDCR */
    0, /* CCCR */
    0, /* TEST */
    0  /* GFC */
};

j2534_can_cfg canCfgHs = {
    .canp = &CAND1,
    .canCfg = &hsCanConfig,
    .cf_index = 0,
    .filters = {{0}},
};

CANConfig swCanConfig = {
    OPMODE_CAN,
    // OPMODE_FDCAN,                  /* OP MODE */
    0, /* NBTP */
    0, /* DBTP */
    0, /* TDCR */
    0, /* CCCR */
    0, /* canp */
    0  /* GFC */
};

j2534_can_cfg canCfgSw = {
    .canp = &CAND2,
    .canCfg = &swCanConfig,
    .cf_index = 0,
    .filters = {{0}},
};

bool rx_can_msg(void *conn, void *rxmsg, packet_t *packet)
{
  int16_t protocol = ((j2534_conn *)conn)->protocol;
  CANRxFrame *msg = (CANRxFrame *)rxmsg;
  memcpy(packet->data, &protocol, 2);
  memcpy(packet->data + 2, msg, 8 + msg->DLC);
  packet->data_len = 10 + msg->DLC;
  return true;
}

uint32_t updateConfig(j2534_conn *conn)
{
  switch (conn->protocol)
  {
  case CAN:
  case CAN_PS:
    conn->cfg = (void *)&canCfgHs;
    break;
  case SW_CAN_PS:
    conn->cfg = (void *)&canCfgSw;
    break;
  default:
    return ERR_INVALID_PROTOCOL_ID;
  }
  return STATUS_NOERROR;
}

uint32_t registerCallback(j2534_conn *conn)
{
  switch (conn->protocol)
  {
  case CAN:
  case CAN_PS:
    registerHsCanCallback(rx_can_msg, conn);
    break;
  case SW_CAN_PS:
    registerSwCanCallback(rx_can_msg, conn);
    break;
  default:
    return ERR_INVALID_PROTOCOL_ID;
  }
  return STATUS_NOERROR;
}

uint32_t removeCallback(j2534_conn *conn)
{
  switch (conn->protocol)
  {
  case CAN:
  case CAN_PS:
    registerHsCanCallback(NULL, NULL);
    break;
  case SW_CAN_PS:
    registerSwCanCallback(NULL, NULL);
    break;
  default:
    return ERR_INVALID_PROTOCOL_ID;
  }
  return STATUS_NOERROR;
}

uint32_t handle_connect_can(j2534_conn *conn)
{
  uint32_t err = updateConfig(conn);
  if (err != STATUS_NOERROR)
  {
    return err;
  }

  j2534_can_cfg *can = conn->cfg;
  if (!canBaudRate(can->canCfg, conn->DataRate, &conn->pcfg->SyncJumpWidth, &conn->pcfg->BitSamplePoint))
  {
    return ERR_INVALID_BAUDRATE;
  }

  canGlobalFilter(can->canCfg, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  conn->ioctl_clear_filters(conn);

  err = registerCallback(conn);
  if (err != STATUS_NOERROR)
  {
    return err;
  }

  canStart(can->canp, can->canCfg);
  return STATUS_NOERROR;
}

uint32_t handle_disconnect_can(j2534_conn *conn)
{
  j2534_can_cfg *can = conn->cfg;
  canStop(can->canp);
  uint32_t err = removeCallback(conn);
  conn->isConnected = false;
  return err;
}

uint32_t write_message_can(j2534_conn *conn, uint32_t timeout, uint16_t len, uint8_t *data)
{
  uint32_t err = ERR_NOT_SUPPORTED;
  if (len < 8)
  {
    return err;
  }
  CANTxFrame *tx = (CANTxFrame *)data;
  j2534_can_cfg *can = conn->cfg;
  if (canTransmit(can->canp, CAN_ANY_MAILBOX, tx, TIME_MS2I(timeout)) == MSG_OK)
  {
    err = STATUS_NOERROR;
  }
  else
  {
    err = ERR_TIMEOUT;
  }
  return err;
}

uint32_t start_periodic_can_msg(j2534_conn *conn, uint8_t data)
{
  (void)conn;
  (void)data;
  return ERR_NOT_SUPPORTED;
}

uint32_t stop_periodic_can_msg(j2534_conn *conn, uint32_t msg)
{
  (void)conn;
  (void)msg;
  return ERR_NOT_SUPPORTED;
}
