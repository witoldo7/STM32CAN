#include "hal.h"
#include <stdio.h>
#include "combi.h"
#include "string.h"
#include "usbcombi.h"
#include "bdm.h"
#include "bdmcpu32.h"

void swab(uint16_t *word);
bool readflash(LONG start_addr, LONG size);
bool writeflash(char *flash_type, LONG start_addr, LONG size);

uint8_t version[2] = {0x03, 0x01};
uint8_t egt_temp[5] = {0};
CANTxFrame txmsg = {.IDE = CAN_IDE_STD, .RTR = CAN_RTR_DATA};
CANFilter filter[28] = {};
CAN_bit_timing_t can_configs[3] = {{1, 8, 105}, {1, 8, 98}, {1, 8, 6}};
icucnt_t last_width_ch1 = 1, last_period_ch1 = 1;
void set_can_filter(CANDriver *can, uint8_t* data) {
  uint8_t j = 0, id, id1, id2, id3;
  if (can == &CAND2) {
    j += 0xE;
  }
  for (uint8_t i = 0; i < data[0]; i = i + 4, j++) {
    filter[j].filter = j;
    filter[j].mode = 1;
    filter[j].scale = 0;
    filter[j].assignment = 0;
    switch (data[0] - i) {
    case 1:
      id = (uint16_t)data[1 + i] | (uint16_t)(data[2 + i] << 8);
      filter[j].register1 = (id << 5) | 0b010;
      filter[j].register2 = 0;
      break;
    case 2:
      id = (uint16_t)data[1 + i] | (uint16_t)(data[2 + i] << 8);
      id1 = (uint16_t)data[3 + i] | (uint16_t)(data[4 + i] << 8);
      filter[j].register1 = (((id << 5) | 0b010) << 16) | ((id1 << 5) | 0b010);
      filter[j].register2 = 0;
      break;
    case 3:
      id = (uint16_t)data[1 + i] | (uint16_t)(data[2 + i] << 8);
      id1 = (uint16_t)data[3 + i] | (uint16_t)(data[4 + i] << 8);
      id2 = (uint16_t)data[5 + i] | (uint16_t)(data[6 + i] << 8);
      filter[j].register1 = (((id << 5) | 0b010) << 16) | ((id1 << 5) | 0b010);
      filter[j].register2 = ((id2 << 5) | 0b010);
      break;
    case 4:
      id = (uint16_t)data[1 + i] | (uint16_t)(data[2 + i] << 8);
      id1 = (uint16_t)data[3 + i] | (uint16_t)(data[4 + i] << 8);
      id2 = (uint16_t)data[5 + i] | (uint16_t)(data[6 + i] << 8);
      id3 = (uint16_t)data[7 + i] | (uint16_t)(data[8 + i] << 8);
      filter[j].register1 = (((id << 5) | 0b010) << 16) | ((id1 << 5) | 0b010);
      filter[j].register2 = (((id2 << 5) | 0b010) << 16) | ((id3 << 5) | 0b010);
      break;
    }
  }
  canSTM32SetFilters(can, 0xE, j, &filter[0]);
}

bool set_can_bitrate(BITRATE bitrate) {
  CAN1->BTR = CAN_BTR_SJW(0) | CAN_BTR_TS2(can_configs[bitrate].TS2)
      | CAN_BTR_TS1(can_configs[bitrate].TS1) | CAN_BTR_BRP(can_configs[bitrate].BRP);
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
  case cmd_swcan_txframe:
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
  case cmd_swcan_filter:
    if (*rx_packet->data == 0x0) {
      return false;
    }
    set_can_filter(&CAND2, rx_packet->data);
    rccEnableCAN2(true);
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
      set_can_filter(&CAND1, rx_packet->data);
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
      uint32_t bitrate = rx_packet->data[3] | (uint32_t)*rx_packet->data << 0x18 | (uint32_t)rx_packet->data[1] << 0x10
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
    txmsg.SID = (uint32_t)rx_packet->data[0] | (uint32_t)(rx_packet->data[1] << 8)
        | (uint32_t)(rx_packet->data[2] << 16) | (uint32_t)(rx_packet->data[3] << 24);
    txmsg.DLC = rx_packet->data[12];
    memcpy(txmsg.data8, rx_packet->data + 4, rx_packet->data[12]);

    return (canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100)) == MSG_OK )
        && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  }
  return false;
}


void float2Bytes(float val, uint8_t* bytes_array){
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  u.float_variable = val;
  memcpy(bytes_array, u.temp_array, 4);
}

bool GetADCValue(uint8_t *val, uint8_t channel) {
  if (channel > 4) {
    return false;
  }
  if (channel == 0) {
    //last_period_ch1  > 0 ? (*val =(1.0f / last_period_ch1)) : (*val = 1.0f);
   last_period_ch1 = icuGetPeriodX(&ICUD3);
   if (last_period_ch1 == 0) {
     last_period_ch1 += 1;
   }

    float fref = (100000.0f / (last_period_ch1 * 1.0)) + 0.1;
    float2Bytes(fref, val);
  } else {
    float2Bytes(2.2f, val);
  }
  return true;
}

bool exec_cmd_board(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_brd_fwversion:
    return CombiSendReplyPacket(tx_packet, rx_packet, version, 2, cmd_term_ack);
  case cmd_brd_adcfilter:
    return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_brd_adc:
    uint8_t value[5] = {0};
    bool ret = GetADCValue(value, rx_packet->data[0]);
    return ret & CombiSendReplyPacket(tx_packet, rx_packet, value , 4, cmd_term_ack);
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

bool exec_cmd_bdm(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_bdm_stop_chip:
    return (stop_chip() == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_reset_chip:
    return (reset_chip() == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_run_chip:
    if (rx_packet->data_len == 4) {
      uint32_t addr = rx_packet->data[3] | (uint32_t)rx_packet->data[0] << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8;
      return (run_chip(&addr) == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    break;
  case cmd_bdm_step_chip:
    return (step_chip() == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_restart_chip:
    return (restart_chip() == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_mem_read:
    if (rx_packet->data_len < 2) {
      return false;
    }
    if (rx_packet->data[0] == 1) {
      uint8_t ret[2];
      if (rx_packet->data[1] == 0) {
        return (memdump_byte(ret) == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, ret, 1, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_byte(ret, &addr) == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, ret, 1, cmd_term_ack);;
      }
      return false;
    }
    if (rx_packet->data[0] == 2) {
      uint8_t ret[2];
      if (rx_packet->data[1] == 0) {
        return (memdump_word((WORD*)ret) == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, ret, 2, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_word((WORD*)ret, &addr) == TERM_OK)
            && CombiSendReplyPacket(tx_packet, rx_packet, ret, 2, cmd_term_ack);;
      }
      return false;
    }
    if (rx_packet->data[0] == 4) {
      uint8_t ret[4];
      if (rx_packet->data[1] == 0) {
        return (memdump_long((LONG*)ret) == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_long((LONG*)ret, &addr) == TERM_OK)
            && CombiSendReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);;
      }
      return false;
    }
    return false;
  case cmd_bdm_mem_write:
    if (rx_packet->data_len == 5) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      return (memwrite_byte(&addr, rx_packet->data[4]) == TERM_OK)
          && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    if (rx_packet->data_len == 6) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      WORD data = rx_packet->data[4] << 8 | rx_packet->data[5];
      return (memwrite_word(&addr, data) == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    if (rx_packet->data_len == 8) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      LONG data = (uint32_t)rx_packet->data[7] | (uint32_t)rx_packet->data[4] << 24 | (uint32_t)rx_packet->data[5] << 16
          | (uint32_t)rx_packet->data[6] << 8;
      return (memwrite_long(&addr, &data) == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    return false;
  case cmd_bdm_sysreg_read:
    if (rx_packet->data_len == 1) {
      uint8_t ret[4];
      return (sysreg_read((LONG*)ret, rx_packet->data[0]) == TERM_OK)
          && CombiSendReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
    }
    return false;
  case cmd_bdm_sysreg_write:
    if (rx_packet->data_len == 5) {
      LONG data = (uint32_t)rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
          | (uint32_t)rx_packet->data[3] << 8;

      return (sysreg_write(rx_packet->data[0], &data) == TERM_OK)
          && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    return false;
  case cmd_bdm_adreg_read:
    if (rx_packet->data_len == 1) {
      uint8_t ret[4];
      return (adreg_read((LONG*)ret, rx_packet->data[0]) == TERM_OK)
          && CombiSendReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
    }
    return false;
  case cmd_bdm_adreg_write:
    if (rx_packet->data_len == 5) {
      LONG data = (uint32_t)rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
          | (uint32_t)rx_packet->data[3] << 8;

      return (adreg_write(rx_packet->data[0], &data) == TERM_OK)
          && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    return false;
  case cmd_bdm_read_flash:
    if (rx_packet->data_len == 8) {
      uint32_t addr = (uint32_t)rx_packet->data[0] << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      uint32_t size = (uint32_t)rx_packet->data[7] | (uint32_t)rx_packet->data[4] << 24
          | (uint32_t)rx_packet->data[5] << 16 | (uint32_t)rx_packet->data[6] << 8;
      return readflash(addr, size);
    }
    return false;
  case cmd_bdm_erase_flash:
    if (rx_packet->data_len == 14) {
      const char flash_type = (char)rx_packet->data[0];
      LONG start_addr = (uint32_t)rx_packet->data[6] << 24 | (uint32_t)rx_packet->data[7] << 16
          | (uint32_t)rx_packet->data[8] << 8 | (uint32_t)rx_packet->data[9];
      LONG end_addr = (uint32_t)rx_packet->data[0xd] | (uint32_t)rx_packet->data[10] << 24
          | (uint32_t)rx_packet->data[0xb] << 16 | (uint32_t)rx_packet->data[0xc] << 8;
      return erase_flash(&flash_type, &start_addr, &end_addr);
    }
    return false;
  case cmd_bdm_write_flash:
    if (rx_packet->data_len == 14) {
      char flash_type = (char)rx_packet->data[0];
      uint32_t start_addr = (uint32_t)rx_packet->data[6] << 24 | (uint32_t)rx_packet->data[7] << 16
          | (uint32_t)rx_packet->data[8] << 8 | (uint32_t)rx_packet->data[9];
      uint32_t end_addr = (uint32_t)rx_packet->data[0xd] | (uint32_t)rx_packet->data[10] << 24
          | (uint32_t)rx_packet->data[0xb] << 16 | (uint32_t)rx_packet->data[0xc] << 8;
      return writeflash(&flash_type, start_addr, end_addr);
    }
    return false;
  case cmd_bdm_pinstate:
    uint8_t pin = 1; //PIN_PWR.read(); TODO
    return (pin == 1) && CombiSendReplyPacket(tx_packet, rx_packet, &pin, 1, cmd_term_ack);
  }

  return false;
}

bool readflash(LONG start_addr, LONG size) {
  char result;
  WORD curr_word;
  WORD *buf_ptr;
  LONG curr_addr;
  uint8_t flash_buf[256];
  packet_t tx_packet, rx_packet;
  uint8_t buffer[IN_PACKETSIZE] = {0};
  uint8_t buffsize = 0;

  tx_packet.cmd_code = cmd_bdm_read_flash;
  tx_packet.data_len = 256;
  tx_packet.data = flash_buf;
  tx_packet.term = cmd_term_ack;
  buf_ptr = (WORD*)flash_buf;
  curr_addr = start_addr;
  while (curr_addr < start_addr + size) {
   // status = CombiReceivePacket(&rx_packet, 0);
    if ((rx_packet.cmd_code == cmd_bdm_read_flash) && (rx_packet.term == cmd_term_nack)) {
      return false;
    }
    if (curr_addr == start_addr) {
      result = memread_word(&curr_word, &curr_addr);
    }
    else {
      result = memdump_word(&curr_word);
    }
    if (result != TERM_OK) {
      return false;
    }
    swab(&curr_word);
    *buf_ptr = curr_word;
    buf_ptr = buf_ptr + 1;
    curr_addr = curr_addr + 2;
    if (((curr_addr - start_addr) & 0xff) == 0) {
      buffsize = CombiSendPacket(&tx_packet, buffer);
      usb_send(&USBD1, EP_IN, buffer, buffsize);
      buf_ptr = (WORD*)flash_buf;
    }
  }

  return result == TERM_OK;
}

bool writeflash(char *flash_type, LONG start_addr, LONG size) {
  packet_t tx_packet, rx_packet;
  bool status;
  WORD curr_word;
  WORD *buf_ptr;
  uint8_t flash_buf[32];
  uint32_t bytes_written;
  bool (*reset_func)(void);
  bool (*flash_func)(const uint32_t*, uint16_t);
  uint8_t buffer[IN_PACKETSIZE] = {0};
  uint8_t buffsize = 0;

  if (strncmp(flash_type, "29f010", 6) == 0 || strncmp(flash_type, "29f400", 6) == 0) {
    reset_func = &reset_am29;
    flash_func = &flash_am29;
  }
  else if (strncmp(flash_type, "28f010", 6) == 0) {
    reset_func = &reset_am28;
    flash_func = &flash_am28;
  }
  else {
    return false;
  }

  // reset the flash
  if (!reset_func()) {
    return false;
  }

  uint32_t curr_addr = start_addr;
  if (strncmp(flash_type, "29f010", 6) == 0) {
    curr_addr = 0;
  }

  tx_packet.cmd_code = cmd_bdm_write_flash;
  tx_packet.data_len = 0;
  tx_packet.data = (BYTE*)0x0;
  tx_packet.term = cmd_term_ack;
  buffsize = CombiSendPacket(&tx_packet, buffer);
  usb_send(&USBD1, EP_IN, buffer, buffsize);

  rx_packet.data = flash_buf;
  bytes_written = 0;
  do {
    if (size <= bytes_written) {
      status = reset_func();
      if (status == true) {
        return true;
      }
      reset_chip();
      return false;
    }
    //status = CombiReceivePacket(&rx_packet, 1000);
    if ((((rx_packet.cmd_code != cmd_bdm_write_flash)) || (rx_packet.term == cmd_term_nack)) || (rx_packet.data_len != 256)) {
      return false;
    }
    buf_ptr = (WORD*)flash_buf;
    for (uint16_t byte_cnt = 0; byte_cnt < 0x100; byte_cnt = byte_cnt + 2) {
      swab(buf_ptr);
      curr_word = *buf_ptr;
      buf_ptr = buf_ptr + 1;
      status = flash_func(&curr_addr, curr_word);
      if (status != true) {
        reset_chip();
        return false;
      }
      curr_addr = curr_addr + 2;
    }
    bytes_written = bytes_written + 0x100;
    buffsize = CombiSendPacket(&tx_packet, buffer);
    usb_send(&USBD1, EP_IN, buffer, buffsize);
  } while (status == true);

  // reset flash
  return (reset_func() && status);
}

void swab(uint16_t *word) {
  uint16_t tmp;
  if (word != 0) {
    tmp = *word;
    *word = *word << 8;
    *word = tmp >> 8 | *word;
  }
  return;
}
