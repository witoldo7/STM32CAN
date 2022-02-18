#include "hal.h"
#include <stdio.h>
#include "combi.h"
#include "string.h"

#include "usbcombi.h"
CAN_bit_timing_config_t can_configs[9] = {{5, 15, 60}, {2, 11, 63}, {2, 12, 56},
                                          {2, 12, 28}, {2, 13, 21}, {2, 11, 12},
                                          {2, 11, 6}, {2, 11, 5}, {1, 5, 6}};

uint8_t version[2] = {0x03, 0x01};
uint8_t egt_temp[5] = {0};
CANTxFrame txmsg = {.IDE = CAN_IDE_STD, .RTR = CAN_RTR_DATA};

bool set_can_bitrate(BITRATE bitrate) {
  CAN1->BTR &= ~(((0x03) << 24) | ((0x07) << 20) | ((0x0F) << 16) | (0x1FF));
  CAN1->BTR |= (((can_configs[bitrate].TS2 - 1) & 0x07) << 20)
      | (((can_configs[bitrate].TS1 - 1) & 0x0F) << 16)
      | ((can_configs[bitrate].BRP - 1) & 0x1FF);
  return true;
}

bool exec_cmd_bdm(packet_t *rx_packet, packet_t *tx_packet) {
  (void)rx_packet;
  (void)tx_packet;
  return true;
}

bool exec_cmd_swcan(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_swcan_open:
    if (rx_packet->data_len == 1) {
      if (*rx_packet->data != 0x1) {
        rccDisableCAN2();
        return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
      }
      rccEnableCAN2(true);
      return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    break;
  case cmd_can_txframe:
    if (rx_packet->data_len != 15) {
      return false;
    }
    txmsg.SID = (uint32_t)rx_packet->data[0]
        | (uint32_t)(rx_packet->data[1] << 8)
        | (uint32_t)(rx_packet->data[2] << 16)
        | (uint32_t)(rx_packet->data[3] << 24);
    txmsg.DLC = rx_packet->data[12];
    memcpy(txmsg.data8, rx_packet->data + 4, rx_packet->data[12]);

    return (canTransmit(&CAND2, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100)) == MSG_OK )
        && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  }
  return false;
}

bool exec_cmd_can(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_can_ecuconnect:
    return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_can_filter:
    if (rx_packet->data_len != 0 && rx_packet->data_len <= 8) {
      if (*rx_packet->data == 0x0) {
        return false;
      }
      CANFilter filter[2] = {{1, 1, 0, 0, 0, 0}, {2, 1, 0, 0, 0, 0}};
      uint8_t filters_number = rx_packet->data[0];
      uint8_t j = 0, id, id1, id2, id3;
      for (uint8_t i = 0; i < filters_number; i = i + 4, j++) {
        switch (filters_number - i) {
        case 1:
          id = (uint16_t)rx_packet->data[1 + i] | (uint16_t)(rx_packet->data[2 + i] << 8);
          filter[j].register1 = (id << 5) | 0b010;
          filter[j].register2 = 0;
          break;
        case 2:
          id = (uint16_t)rx_packet->data[1 + i] | (uint16_t)(rx_packet->data[2 + i] << 8);
          id1 = (uint16_t)rx_packet->data[3 + i] | (uint16_t)(rx_packet->data[4 + i] << 8);
          filter[j].register1 = (((id << 5) | 0b010) << 16) | ((id1 << 5) | 0b010);
          filter[j].register2 = 0;
          break;
        case 3:
          id = (uint16_t)rx_packet->data[1 + i] | (uint16_t)(rx_packet->data[2 + i] << 8);
          id1 = (uint16_t)rx_packet->data[3 + i] | (uint16_t)(rx_packet->data[4 + i] << 8);
          id2 = (uint16_t)rx_packet->data[5 + i] | (uint16_t)(rx_packet->data[6 + i] << 8);
          filter[j].register1 = (((id << 5) | 0b010) << 16) | ((id1 << 5) | 0b010);
          filter[j].register2 = ((id2 << 5) | 0b010);
          break;
        case 4:
          id = (uint16_t)rx_packet->data[1 + i] | (uint16_t)(rx_packet->data[2 + i] << 8);
          id1 = (uint16_t)rx_packet->data[3 + i] | (uint16_t)(rx_packet->data[4 + i] << 8);
          id2 = (uint16_t)rx_packet->data[5 + i] | (uint16_t)(rx_packet->data[6 + i] << 8);
          id3 = (uint16_t)rx_packet->data[7 + i] | (uint16_t)(rx_packet->data[8 + i] << 8);
          filter[j].register1 = (((id << 5) | 0b010) << 16) | ((id1 << 5) | 0b010);
          filter[j].register2 = (((id2 << 5) | 0b010) << 16) | ((id3 << 5) | 0b010);
          break;
        }
      }
      canSTM32SetFilters(&CAND1, 0xE, j, &filter[0]);
      rccEnableCAN1(true);
    }
    return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_can_open:
    if (rx_packet->data_len == 1) {
      if (*rx_packet->data != 0x1) {
        rccDisableCAN1();
        return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
      }
      rccEnableCAN1(true);
      return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    break;
  case cmd_can_bitrate:
    if (rx_packet->data_len == 4) {
      uint32_t bitrate = rx_packet->data[3]
          | (uint32_t)*rx_packet->data << 0x18
          | (uint32_t)rx_packet->data[1] << 0x10
          | (uint32_t)rx_packet->data[2] << 8;
      switch (bitrate) {
      case 500000:
        set_can_bitrate(CAN_500KBPS);
        break;
      case 47619:
        set_can_bitrate(CAN_47KBPS);
        break;
      default:
        return false;
      }
      return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    break;
  case cmd_can_txframe:
    if (rx_packet->data_len != 15) {
      return false;
    }
    txmsg.SID = (uint32_t)rx_packet->data[0]
        | (uint32_t)(rx_packet->data[1] << 8)
        | (uint32_t)(rx_packet->data[2] << 16)
        | (uint32_t)(rx_packet->data[3] << 24);
    txmsg.DLC = rx_packet->data[12];
    memcpy(txmsg.data8, rx_packet->data + 4, rx_packet->data[12]);

    return (canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100)) == MSG_OK )
        && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  }
  return false;
}

bool exec_cmd_board(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_brd_fwversion:
    return CombiSendReplyPacket(tx_packet, rx_packet, version, 2, cmd_term_ack);
  case cmd_brd_adcfilter:
    return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_brd_adc:
    return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_brd_egt:
    return CombiSendReplyPacket(tx_packet, rx_packet, egt_temp, 5, cmd_term_ack);
  }
  return false;
}

bool CombiSendReplyPacket(packet_t *reply, packet_t *source, uint8_t *data,
                          uint16_t data_len, uint8_t term) {
  if ((reply == (packet_t*)0x0) || (source == (packet_t*)0x0)) {
    return false;
  }
  else {
    reply->cmd_code = source->cmd_code;
    reply->data_len = data_len;
    if ((data != 0) && (data_len != 0)) {
      reply->data = data;
    }
    reply->term = term;
  }
  return true;
}

uint8_t CombiSendPacket(packet_t *packet, uint8_t *buffer) {
  uint8_t size = 0;
  if (packet != (packet_t*)0x0) {
    buffer[0] = packet->cmd_code;
    buffer[1] = (uint8_t)(packet->data_len >> 8);
    buffer[2] = (uint8_t)packet->data_len;
    if (*(packet->data) != 0 && packet->data_len != 0) {
      memcpy(buffer + 3, packet->data, packet->data_len);
      size = packet->data_len + 3;
      buffer[size] = packet->term;
      size++;
    }
    else {
      buffer[3] = packet->term;
      size = 4;
    }
    return size;
  }
  return size;
}
