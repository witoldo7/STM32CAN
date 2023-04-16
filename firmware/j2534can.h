// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#ifndef J2534CAN_H_
#define J2534CAN_H_

uint32_t handle_connect_can(j2534_conn* conn);
uint32_t handle_disconnect_can(j2534_conn* conn);
uint32_t start_filter_can(j2534_conn* conn, uint8_t* data, uint32_t* idx);
uint32_t stop_filter_can(j2534_conn* conn, uint32_t idx);
uint32_t write_message_can(j2534_conn* conn, uint32_t timeout, uint16_t len, uint8_t* data);
uint32_t start_periodic_can_msg(j2534_conn* conn, uint8_t data);
uint32_t stop_periodic_can_msg(j2534_conn* conn, uint32_t msg);
uint32_t ioctl_clear_filters_can(j2534_conn* conn);
uint32_t ioctl_datarate_can(j2534_conn* conn);
uint32_t ioctl_loopback_can(j2534_conn* conn);

#endif /* J2534CAN_H_ */
