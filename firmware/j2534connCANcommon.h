// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#ifndef J2534CAN_H_
#define J2534CAN_H_

uint32_t start_filter_can_common(j2534_conn *conn, uint8_t *data, uint32_t *idx);
uint32_t ioctl_clear_filters_can_common(j2534_conn* conn);
uint32_t stop_filter_can_common(j2534_conn* conn, uint32_t idx);
uint32_t ioctl_datarate_can_common(j2534_conn* conn);
uint32_t ioctl_loopback_can_common(j2534_conn* conn);

#endif /* J2534CAN_H_ */
