// SPDX-License-Identifier: Apache-2.0

/*
 * STM32kline Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#ifndef J2534KLINE_H_
#define J2534KLINE_H_

#define ISO14230_SHORTHDR 0x01
#define ISO14230_LONGHDR 0x2
#define ISO14230_LENBYTE 0x4
#define ISO14230_FMTLEN 0x8
#define ISO14230_FUNCADDR 0x10

#define LENGTH_MASK 0x3F
#define TESTER_PRESENT_PERIOD_MS 3000
#define KWP_MAX_PAYLOAD 255
#define KWP_SID_TESTER_PRESENT 0x3E
#define KWP_SID_STOM_COMM 0x82
#define KWP_SID_STOM_COMM_RESP 0xC2
#define KWP_SID_ACCESS_TIM 0x83
#define KWP_SID_ACCESS_TIM_RESP 0xC3
#define KWP_SID_NEGATIVE_RESP 0x7F

#define KWP_FMT_ISO_FUNCADDR 0x01
#define KWP_FMT_FRAMED 0x02
#define KWP_FMT_DATAONLY 0x04
#define KWP_FMT_CKSUMMED 0x08
#define KWP_FMT_BADCS 0x10

typedef struct
{
    uint8_t fmt;   /* Message format */
    uint8_t tgt;   /* Destination from received frame */
    uint8_t src;   /* Source from received frame */
    uint16_t len;  /* Calculated data length */
    uint8_t *data; /* Payload */
    uint8_t crc;
    uint32_t rxtime; /* Processing timestamp*/
} kwp_msg;

uint32_t handle_connect_kline(j2534_conn *conn);
uint32_t handle_disconnect_kline(j2534_conn *conn);
uint32_t start_filter_kline(j2534_conn *conn, uint8_t *data, uint32_t *idx);
uint32_t stop_filter_kline(j2534_conn *conn, uint32_t idx);
uint32_t write_message_kline(j2534_conn *conn, uint32_t timeout, uint16_t len, uint8_t *data);
uint32_t start_periodic_msg_kline(j2534_conn *conn, uint8_t data);
uint32_t stop_periodic_msg_kline(j2534_conn *conn, uint32_t msg);

uint32_t ioctl_clear_filters_kline(j2534_conn *conn);
uint32_t ioctl_datarate_kline(j2534_conn *conn);
uint32_t ioctl_loopback_kline(j2534_conn *conn);
uint32_t ioctl_fast_init_kline(j2534_conn *conn, uint8_t *in, uint8_t *out);
uint32_t ioctl_five_baud_init_kline(j2534_conn *conn, uint8_t *in, uint8_t *out);

#endif /* J2534KLINE_H_ */
