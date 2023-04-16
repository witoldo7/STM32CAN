// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */
#include "j2534.h"
#include "j2534kline.h"

uint32_t handle_connect_kline(j2534_conn* conn) {
  return STATUS_NOERROR;
}

uint32_t handle_disconnect_kline(j2534_conn* conn) {
  return STATUS_NOERROR;
}

uint32_t stop_filter_kline(j2534_conn* conn, uint32_t idx) {
  return STATUS_NOERROR;
}

uint32_t start_filter_kline(j2534_conn* conn, uint8_t* data, uint32_t* idx) {
  idx=1;
  return STATUS_NOERROR;
}

uint32_t write_message_kline(j2534_conn* conn, uint32_t timeout, uint16_t len, uint8_t* data) {
  return STATUS_NOERROR;
}

uint32_t start_periodic_msg_kline(j2534_conn* conn, uint8_t data) {
  return STATUS_NOERROR;
}

uint32_t stop_periodic_msg_kline(j2534_conn* conn, uint32_t msg) {
  return STATUS_NOERROR;
}

uint32_t ioctl_clear_filters_kline(j2534_conn* conn) {
  return STATUS_NOERROR;
}

uint32_t ioctl_datarate_kline(j2534_conn* conn) {
  return STATUS_NOERROR;
}

uint32_t ioctl_loopback_kline(j2534_conn* conn) {
  return STATUS_NOERROR;
}

uint32_t ioctl_fast_init_kline(void* conn) {
  (void)conn;
  return STATUS_NOERROR;
}

uint32_t ioctl_five_baud_init_kline(void* conn) {
  (void)conn;
  return STATUS_NOERROR;
}



