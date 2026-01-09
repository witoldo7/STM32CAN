// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2026 Witold Olechowski
 */
#include <string.h>
#include "j2534.h"
#include "j2534connCAN.h"
#include "debug.h"

uint32_t handle_connect_default(void *conn)
{
  (void)conn;
  return ERR_NOT_SUPPORTED;
}
uint32_t handle_disconnect_default(void *conn)
{
  (void)conn;
  return ERR_NOT_SUPPORTED;
}
uint32_t start_filter_default(void *conn, uint8_t *data, uint32_t *idx)
{
  (void)conn;
  (void)data;
  (void)idx;
  return ERR_NOT_SUPPORTED;
}
uint32_t stop_filter_default(void *conn, uint32_t idx)
{
  (void)conn;
  (void)idx;
  return ERR_NOT_SUPPORTED;
}
uint32_t write_message_default(void *conn, uint32_t timeout, uint16_t len, uint8_t *data)
{
  (void)conn;
  (void)timeout;
  (void)len;
  (void)data;
  return ERR_NOT_SUPPORTED;
}
uint32_t read_message_default(void *conn, uint32_t timeout, uint32_t msgNum, uint16_t len, uint8_t *data)
{
  (void)conn;
  (void)timeout;
  (void)msgNum;
  (void)len;
  (void)data;
  return ERR_NOT_SUPPORTED;
}
uint32_t start_periodic_default_msg(void *conn, uint8_t data)
{
  (void)conn;
  (void)data;
  return ERR_NOT_SUPPORTED;
}
uint32_t stop_periodic_default_msg(void *conn, uint32_t msg)
{
  (void)conn;
  (void)msg;
  return ERR_NOT_SUPPORTED;
}
uint32_t ioctl_clear_filters_default(void *conn)
{
  (void)conn;
  return ERR_NOT_SUPPORTED;
}
uint32_t ioctl_datarate_default(void *conn)
{
  (void)conn;
  return ERR_NOT_SUPPORTED;
}
uint32_t ioctl_loopback_default(void *conn)
{
  (void)conn;
  return ERR_NOT_SUPPORTED;
}
uint32_t ioctl_fast_init_default(void *conn, uint8_t *in, uint8_t *out)
{
  (void)conn;
  (void)in;
  (void)out;
  return ERR_NOT_SUPPORTED;
}
uint32_t ioctl_five_baud_init_default(void *conn, uint8_t *in, uint8_t out)
{
  (void)conn;
  (void)in;
  (void)out;
  return ERR_NOT_SUPPORTED;
}
