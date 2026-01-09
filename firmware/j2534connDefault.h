// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2026 Witold Olechowski
 */

#ifndef J2534CONNDEFAULT_H_
#define J2534CONNDEFAULT_H_

uint32_t handle_connect_default(void *conn);
uint32_t handle_disconnect_default(void *conn);
uint32_t start_filter_default(void *conn, uint8_t *data, uint32_t *idx);
uint32_t stop_filter_default(void *conn, uint32_t idx);
uint32_t write_message_default(void *conn, uint32_t timeout, uint16_t len, uint8_t *data);
uint32_t read_message_default(void *conn, uint32_t timeout, uint32_t msgNum, uint16_t len, uint8_t *data);
uint32_t start_periodic_default_msg(void *conn, uint8_t data);
uint32_t stop_periodic_default_msg(void *conn, uint32_t msg);
uint32_t ioctl_clear_filters_default(void *conn);
uint32_t ioctl_datarate_default(void *conn);
uint32_t ioctl_loopback_default(void *conn);
uint32_t ioctl_fast_init_default(void *conn, uint8_t *in, uint8_t *out);
uint32_t ioctl_five_baud_init_default(void *conn, uint8_t *in, uint8_t out);


#endif /* J2534CONNDEFAULT_H_ */
