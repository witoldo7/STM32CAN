// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2025 Witold Olechowski
 */

#ifndef J2534CONNISO15765_H_
#define J2534CONNISO15765_H_

typedef enum
{
    ISOTP_PCI_TYPE_SINGLE = 0x00,
    ISOTP_PCI_TYPE_FIRST_FRAME = 0x10,
    TSOTP_PCI_TYPE_CONSECUTIVE_FRAME = 0x20,
    ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME = 0x30
} IsoTpProtocolControlInformation;

typedef struct
{
    uint8_t payload[5120];
    uint16_t payloadSize;
    uint16_t payloadPos;
} isoPayload;

uint32_t handle_connect_ISO15765(j2534_conn *conn);
uint32_t handle_disconnect_ISO15765(j2534_conn *conn);
uint32_t start_filter_ISO15765(j2534_conn *conn, uint8_t *data, uint32_t *idx);
uint32_t write_message_ISO15765(j2534_conn *conn, uint32_t timeout, uint16_t len, uint8_t *data);

#endif /* J2534CONNISO15765_H_ */