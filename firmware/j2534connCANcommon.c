// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2025 Witold Olechowski
 */
#include <string.h>
#include "j2534.h"
#include "debug.h"

uint32_t start_filter_can_common(j2534_conn *conn, uint8_t *data, uint32_t *idx)
{
  j2534_can_cfg *can = conn->cfg;
  uint32_t pattern, mask, flow, flags;
  uint8_t size = data[11], type = data[10];
  memcpy(&flags, data + 4, 4);
  memcpy(&mask, data + 12, size);
  memcpy(&pattern, data + 24, size);
  memcpy(&flow, data + 36, size);

  switch (type)
  {
  case PASS_FILTER:
    can->filters[can->cf_index].filter_cfg = CAN_FILTER_CFG_FIFO_0;
    break;
  case BLOCK_FILTER:
    can->filters[can->cf_index].filter_cfg = CAN_FILTER_CFG_REJECT;
    break;
  case FLOW_CONTROL_FILTER:
    can->filters[can->cf_index].filter_cfg = CAN_FILTER_CFG_FIFO_0; // check
    can->flow[can->cf_index] = flow;
    break;
  default:
    return ERR_NOT_SUPPORTED;
    break;
  }

  if (flags & CAN_29BIT_ID)
  {
    can->filters[can->cf_index].filter_type = CAN_FILTER_TYPE_EXT;
  }
  else
  {
    can->filters[can->cf_index].filter_type = CAN_FILTER_TYPE_STD;
  }
  can->filters[can->cf_index].filter_mode = CAN_FILTER_MODE_CLASSIC;
  can->filters[can->cf_index].identifier1 = pattern;
  can->filters[can->cf_index].identifier2 = mask;
  DBG_PRNT("type: %d,  pattern: %x, mask %x, flow: %x, idx: %d\r\n", can->filters[can->cf_index].filter_cfg, can->filters[can->cf_index].identifier1, can->filters[can->cf_index].identifier2, flow, can->cf_index);

  *idx = can->cf_index++;
  canSTM32SetFilters(can->canp, can->cf_index, can->filters);
  return STATUS_NOERROR;
}

uint32_t ioctl_clear_filters_can_common(j2534_conn *conn)
{
  j2534_can_cfg *can = conn->cfg;

  for (uint8_t i = 0; i < FILTER_NBR; i++)
  {
    can->filters[i].filter_cfg = 0;
    can->filters[i].filter_type = 0;
    can->filters[i].filter_mode = 0;
    can->filters[i].identifier1 = 0;
    can->filters[i].identifier2 = 0;
    can->flow[i] = 0;
  }
  can->cf_index = 0;
  canSTM32SetFilters(can->canp, FILTER_NBR, can->filters);

  return STATUS_NOERROR;
}

uint32_t stop_filter_can_common(j2534_conn *conn, uint32_t idx)
{
  j2534_can_cfg *can = conn->cfg;
  can->filters[idx].filter_cfg = 0;
  can->filters[idx].filter_type = 0;
  can->filters[idx].filter_mode = 0;
  can->filters[idx].identifier1 = 0;
  can->filters[idx].identifier2 = 0;
  can->flow[idx] = 0;
  canSTM32SetFilters(can->canp, idx, can->filters);
  return STATUS_NOERROR;
}

uint32_t ioctl_datarate_can_common(j2534_conn *conn)
{
  j2534_protocol_cfg *pcfg = conn->pcfg;
  j2534_can_cfg *can = conn->cfg;
  canStop(can->canp);
  if (!canBaudRate(can->canCfg, conn->DataRate, &pcfg->SyncJumpWidth, &pcfg->BitSamplePoint))
  {
    return ERR_INVALID_BAUDRATE;
  }
  canStart(can->canp, can->canCfg);
  return STATUS_NOERROR;
}

uint32_t ioctl_loopback_can_common(j2534_conn *conn)
{
  j2534_protocol_cfg *pcfg = conn->pcfg;
  j2534_can_cfg *can = conn->cfg;
  canStop(can->canp);
  if (pcfg->Loopback == 1)
  {
    can->canCfg->CCCR |= FDCAN_CCCR_TEST | FDCAN_CCCR_MON;
    can->canCfg->TEST |= FDCAN_TEST_LBCK;
  }
  else
  {
    can->canCfg->CCCR &= ~FDCAN_CCCR_TEST | ~FDCAN_CCCR_MON;
    can->canCfg->TEST &= ~FDCAN_TEST_LBCK;
  }
  canStart(can->canp, can->canCfg);
  return STATUS_NOERROR;
}