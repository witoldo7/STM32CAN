// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2025 Witold Olechowski
 */
#include <string.h>
#include "hal.h"
#include "j2534.h"
#include "j2534connISO15765.h"
#include "debug.h"
#include "usbadapter.h"

CANConfig ISO15765CanConfigHS = {
    OPMODE_CAN,
    // OPMODE_FDCAN,                  /* OP MODE */
    0, /* NBTP */
    0, /* DBTP */
    0, /* TDCR */
    0, /* CCCR */
    0, /* TEST */
    0  /* GFC */
};

j2534_can_cfg ISO15765CfgHs = {
    .canp = &CAND1,
    .canCfg = &ISO15765CanConfigHS,
    .cf_index = 0,
    .filters = {{0}},
};

CANConfig ISO15765CanConfigSw = {
    OPMODE_CAN,
    // OPMODE_FDCAN,                  /* OP MODE */
    0, /* NBTP */
    0, /* DBTP */
    0, /* TDCR */
    0, /* CCCR */
    0, /* canp */
    0  /* GFC */
};

j2534_can_cfg ISO15765CfgSw = {
    .canp = &CAND2,
    .canCfg = &ISO15765CanConfigSw,
    .cf_index = 0,
    .filters = {{0}},
};

isoPayload rxPayload; // For receiving
isoPayload txPayload; // For sending
uint8_t rx_frame_count = 0;

uint32_t updateISO15765Cfg(j2534_conn *conn)
{
    switch (conn->protocol)
    {
    case ISO15765:
        conn->cfg = (void *)&ISO15765CfgHs;
        break;
    case SW_ISO15765_PS:
        conn->cfg = (void *)&ISO15765CfgSw;
        break;
    default:
        return ERR_INVALID_PROTOCOL_ID;
    }
    return STATUS_NOERROR;
}

void buildPacket(j2534_conn *conn, packet_t *packet, uint32_t rxstatus, uint32_t timestamp,
                 uint16_t data_len, uint16_t data_ext_len, uint8_t *data)
{
    // protocol 2, datalen 2, datalenext 2, rxstatus 4, timestamp 4, buff[] var
    uint16_t protocol = conn->protocol;
    memcpy(packet->data, &protocol, 2);
    memcpy(packet->data + 2, &data_len, 2);
    memcpy(packet->data + 4, &data_ext_len, 2);
    memcpy(packet->data + 6, &rxstatus, 4);
    memcpy(packet->data + 10, &timestamp, 4);
    memcpy(packet->data + 14, data, data_len);
    packet->data_len = 15 + data_len;
}

void sendPacketUsb(j2534_conn *conn, packet_t *packet, uint32_t rxstatus, uint32_t timestamp,
                   uint16_t data_len, uint16_t data_ext_len, uint8_t *data)
{
    buildPacket(conn, packet, rxstatus, timestamp, data_len, data_ext_len, data);
    addUsbMsgToMailbox(packet);
}

void recv_single_frame(j2534_conn *conn, CANRxFrame *msg, packet_t *packet)
{
    uint32_t mid, size = msg->DLC + 4;
    uint8_t data[size];
    uint32_t timestamp = msg->RXTS;
    uint32_t rxstatus = (msg->common.XTD ? CAN_29BIT_ID : 0);

    if (msg->common.XTD)
        mid = msg->ext.EID & 0x1FFFFFFF;
    else
        mid = msg->std.SID & 0x7FF;

    if (conn->flags & ISO15765_ADDR_TYPE)
    {
        data[0] = (uint8_t)(mid >> 16);
        data[1] = (uint8_t)(mid >> 8);
        data[2] = (uint8_t)mid;
        data[3] = (uint8_t)msg->data8[0];

        memcpy(data + 4, msg->data8 + 2, msg->data8[1]);
    }
    else
    {
        size = msg->data8[0] + 4;
        data[0] = (uint8_t)(mid >> 24);
        data[1] = (uint8_t)(mid >> 16);
        data[2] = (uint8_t)(mid >> 8);
        data[3] = (uint8_t)mid;

        memcpy(data + 4, msg->data8 + 1, msg->data8[0]);
    }

    buildPacket(conn, packet, rxstatus, timestamp, size, size, data);
}

void send_first_frame(j2534_conn *conn, CANRxFrame *msg, packet_t *packet)
{
    j2534_can_cfg *can = conn->cfg;
    CANTxFrame ff = {0};
    uint32_t id = msg->std.SID;
    DBG_PRNT("flow: %02x req id: %02x\r\n", can->flow[id], id);

    uint8_t size = ((msg->data8[0] & 0x0F) << 8) | (msg->data8[1] + 4);
    rxPayload.payloadSize = size;
    rxPayload.payloadPos = 10;
    memcpy(&rxPayload.payload[4], &msg->data8[2], 6);
    rxPayload.payload[0] = id >> 24;
    rxPayload.payload[1] = id >> 16;
    rxPayload.payload[2] = id >> 8;
    rxPayload.payload[3] = id >> 0;
    rx_frame_count = 0; // fixme make connection specific

    ff.std.SID = can->flow[0]; // fixme
    ff.DLC = 8;
    ff.data8[0] = ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME;
    ff.data8[1] = conn->pcfg->Iso15765Bs;
    ff.data8[2] = conn->pcfg->Iso15765Stmin;
    msg_t status = canTransmit(can->canp, CAN_ANY_MAILBOX, &ff, TIME_MS2I(100));

    DBG_PRNT("wtiteff: len: %d, id: %x, data:", ff.DLC, ff.std.SID);
    for (uint8_t i = 0; i < ff.DLC; i++)
    {
        DBG_PRNT(" %02x,", ff.data8[i]);
    }
    DBG_PRNT("\r\n status: %d\r\n", status);

    buildPacket(conn, packet, START_OF_MESSAGE, 0, 4, 0, rxPayload.payload);
}

bool recv_consecutive_frame(j2534_conn *conn, CANRxFrame *msg, packet_t *packet)
{
    j2534_can_cfg *can = conn->cfg;
    uint8_t max_copy = min(rxPayload.payloadSize - rxPayload.payloadPos, 7);
    memcpy(&rxPayload.payload[rxPayload.payloadPos], msg->data8 + 1, max_copy);
    rxPayload.payloadPos += max_copy;
    rx_frame_count++;
    if (rxPayload.payloadPos >= rxPayload.payloadSize)
    {
        uint32_t rxstatus = (msg->common.XTD ? CAN_29BIT_ID : 0);
        buildPacket(conn, packet, rxstatus, msg->RXTS, rxPayload.payloadSize, rxPayload.payloadSize, rxPayload.payload);
        return true;
    }
    if (rx_frame_count >= 8)
    {
        rx_frame_count = 0;
        CANTxFrame ff = {0};
        ff.std.SID = can->flow[0]; // fixme
        ff.DLC = 8;
        ff.data8[0] = ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME;
        ff.data8[1] = conn->pcfg->Iso15765Bs;
        ff.data8[2] = conn->pcfg->Iso15765Stmin;
        msg_t status = canTransmit(can->canp, CAN_ANY_MAILBOX, &ff, TIME_MS2I(100));
    }
    return false;
}

void handle_flow_control(j2534_conn *conn, CANRxFrame *msg, packet_t *packet)
{
}

bool rx_ISO15765Hs_msg(void *conn, void *rxmsg, packet_t *packet)
{
    CANRxFrame *msg = (CANRxFrame *)rxmsg;
    j2534_conn *jcon = (j2534_conn *)conn;
    bool send = true;
    uint8_t cmp = jcon->flags & ISO15765_ADDR_TYPE;
    DBG_PRNT("read: len: %d, id: %x, data:", msg->DLC, msg->std.SID);
    for (uint8_t i = 0; i < msg->DLC; i++)
    {
        DBG_PRNT(" %02x,", msg->data8[i]);
    }
    DBG_PRNT("\r\nPCI: %02x\r\n", msg->data8[cmp] & 0xF0);

    switch (msg->data8[cmp] & 0xF0)
    {
    case ISOTP_PCI_TYPE_SINGLE:
        recv_single_frame(conn, msg, packet);
        break;
    case ISOTP_PCI_TYPE_FIRST_FRAME:
        send_first_frame(conn, msg, packet);
        break;
    case TSOTP_PCI_TYPE_CONSECUTIVE_FRAME:
        send = recv_consecutive_frame(conn, msg, packet);
        break;
    case ISOTP_PCI_TYPE_FLOW_CONTROL_FRAME:
        handle_flow_control(conn, msg, packet);
        break;
    default:
        DBG_PRNT("Id: %04x, invalid pci: %02x \r\n", msg->common.XTD ? msg->ext.EID : msg->std.SID, msg->data8[cmp]);
        break;
    }

    return send;
}

bool rx_ISO15765Sw_msg(void *conn, void *rxmsg, packet_t *packet)
{
    DBG_PRNT("rx_ISO15765Sw_msg \r\n");
    (void)conn;
    CANRxFrame *msg = (CANRxFrame *)rxmsg;
    uint16_t protocol = SW_CAN_PS;
    memcpy(packet->data, &protocol, 2);
    memcpy(packet->data + 2, msg, 10 + msg->DLC);
    packet->data_len = 10 + msg->DLC;
    return true;
}

uint32_t registerCanCallback(j2534_conn *conn)
{
    switch (conn->protocol)
    {
    case ISO15765:
        registerHsCanCallback(rx_ISO15765Hs_msg, conn);
        break;
    case SW_ISO15765_PS:
        registerSwCanCallback(rx_ISO15765Sw_msg, conn);
        break;
    default:
        return ERR_INVALID_PROTOCOL_ID;
    }
    return STATUS_NOERROR;
}

uint32_t removeCanCallback(j2534_conn *conn)
{
    switch (conn->protocol)
    {
    case ISO15765:
        registerHsCanCallback(NULL, NULL);
        break;
    case SW_ISO15765_PS:
        registerSwCanCallback(NULL, NULL);
        break;
    default:
        return ERR_INVALID_PROTOCOL_ID;
    }
    return STATUS_NOERROR;
}

uint32_t handle_connect_ISO15765(j2534_conn *conn)
{
    uint32_t err = updateISO15765Cfg(conn);
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

    err = registerCanCallback(conn);
    if (err != STATUS_NOERROR)
    {
        return err;
    }

    canStart(can->canp, can->canCfg);
    return STATUS_NOERROR;
}

uint32_t handle_disconnect_ISO15765(j2534_conn *conn)
{
    j2534_can_cfg *can = conn->cfg;
    canStop(can->canp);
    uint32_t err = removeCanCallback(conn);
    conn->isConnected = false;
    return err;
}

uint32_t write_message_ISO15765(j2534_conn *conn, uint32_t timeout, uint16_t len, uint8_t *data)
{
    uint32_t err = ERR_NOT_SUPPORTED;
    if (len < 8)
    {
        return ERR_INVALID_MSG;
    }
    j2534_can_cfg *can = conn->cfg;

    // data: txflags 4, len 2, lenext 2, data[]
    uint32_t txflags, id;
    uint16_t dlen, dlenExt;
    CANTxFrame tx = {0};
    memcpy(&txflags, data, 4);
    memcpy(&dlen, data + 4, 2);
    memcpy(&dlenExt, data + 6, 2);
    uint8_t *canData = data + 8;

    bool xtd = txflags & CAN_29BIT_ID;
    tx.common.XTD = xtd;
    if (dlen < 4)
        return ERR_INVALID_MSG;
    uint8_t data_size = dlen - 4;

    if (data_size <= 11)
    {
        id = data_size << 24 | canData[1] << 16 | canData[2] << 8 | canData[3];
        tx.data8[0] = data_size;
        memcpy(tx.data8 + 1, canData + 4, data_size);
    }
    else
    {
        id = canData[0] << 24 | canData[1] << 16 | canData[2] << 8 | canData[3];
        tx.data8[0] = 0x10 | ((data_size - 4) & 0x0F00) >> 8;
        tx.data8[1] = (data_size - 4) & 0xFF;
        memcpy(tx.data8 + 2, canData + 4, 6);

        txPayload.payloadSize = data_size;
        txPayload.payloadPos = 10;
        memcpy(txPayload.payload, canData, data_size);
    }

    if (xtd)
    {
        tx.ext.EID = id;
    }
    else
    {
        tx.std.SID = id;
    }
    tx.DLC = 8;

    DBG_PRNT("write: len %d, id: %x data: ", tx.DLC, tx.std.SID);
    for (uint8_t i = 0; i < tx.DLC; i++)
    {
        DBG_PRNT("%02x, ", tx.data8[i]);
    }
    DBG_PRNT("\r\n");
    if (canTransmit(can->canp, CAN_ANY_MAILBOX, &tx, TIME_MS2I(timeout)) == MSG_OK)
    {
        if (data_size > 11)
        { // TXDONE after end of multiframe
            return STATUS_NOERROR;
        }
        uint8_t buff[32] = {0};
        packet_t packet = {.cmd_code = cmd_j2534_read_message, .data = buff, .term = cmd_term_ack};
        uint32_t flags = TX_MSG_TYPE | TX_INDICATION;
        uint32_t mid;
        if (tx.common.XTD)
            mid = tx.ext.EID;
        else
            mid = tx.std.SID;
        uint8_t idbuff[4] = {(uint8_t)(mid >> 24), (uint8_t)(mid >> 16), (uint8_t)(mid >> 8), (uint8_t)mid};
        sendPacketUsb(conn, &packet, flags, 0, 4, 0, idbuff);

        err = STATUS_NOERROR;
    }
    else
    {
        err = ERR_TIMEOUT;
    }
    return err;
}
