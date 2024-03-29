// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#include "bdmutility.h"
#include "bdm.h"
#include "bdmcpu32.h"
#include <string.h>
#include "usbadapter.h"

bool readflash(LONG start_addr, LONG size);
bool writeflash(char *flash_type, LONG start_addr, LONG size);

bool exec_cmd_bdm(packet_t *rx_packet, packet_t *tx_packet) {
  switch (rx_packet->cmd_code) {
  case cmd_bdm_stop_chip:
    return (stop_chip() == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_reset_chip:
    return (reset_chip() == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_run_chip:
    if (rx_packet->data_len == 4) {
      uint32_t addr = rx_packet->data[3] | (uint32_t)rx_packet->data[0] << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8;
      return (run_chip(&addr) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    break;
  case cmd_bdm_step_chip:
    return (step_chip() == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_restart_chip:
    return (restart_chip() == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
  case cmd_bdm_mem_read:
    if (rx_packet->data_len < 2) {
      return false;
    }
    if (rx_packet->data[0] == 1) {
      uint8_t ret[2];
      if (rx_packet->data[1] == 0) {
        return (memdump_byte(ret) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, ret, 1, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_byte(ret, &addr) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, ret, 1, cmd_term_ack);;
      }
      return false;
    }
    if (rx_packet->data[0] == 2) {
      uint8_t ret[2];
      if (rx_packet->data[1] == 0) {
        return (memdump_word((WORD*)ret) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, ret, 2, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_word((WORD*)ret, &addr) == TERM_OK)
            && prepareReplyPacket(tx_packet, rx_packet, ret, 2, cmd_term_ack);;
      }
      return false;
    }
    if (rx_packet->data[0] == 4) {
      uint8_t ret[4];
      if (rx_packet->data[1] == 0) {
        return (memdump_long((LONG*)ret) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
      }
      else if (rx_packet->data_len == 6) {
        uint32_t addr = (uint32_t)rx_packet->data[5] | (uint32_t)rx_packet->data[2] << 24
            | (uint32_t)rx_packet->data[3] << 16 | (uint32_t)rx_packet->data[4] << 8;
        return (memread_long((LONG*)ret, &addr) == TERM_OK)
            && prepareReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);;
      }
      return false;
    }
    return false;
  case cmd_bdm_mem_write:
    if (rx_packet->data_len == 5) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      return (memwrite_byte(&addr, rx_packet->data[4]) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    if (rx_packet->data_len == 6) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      WORD data = rx_packet->data[4] << 8 | rx_packet->data[5];
      return (memwrite_word(&addr, data) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    if (rx_packet->data_len == 8) {
      uint32_t addr = (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16
          | (uint32_t)rx_packet->data[2] << 8 | (uint32_t)rx_packet->data[3];
      LONG data = (uint32_t)rx_packet->data[7] | (uint32_t)rx_packet->data[4] << 24 | (uint32_t)rx_packet->data[5] << 16
          | (uint32_t)rx_packet->data[6] << 8;
      return (memwrite_long(&addr, &data) == TERM_OK) && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    return false;
  case cmd_bdm_sysreg_read:
    if (rx_packet->data_len == 1) {
      uint8_t ret[4];
      return (sysreg_read((LONG*)ret, rx_packet->data[0]) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
    }
    return false;
  case cmd_bdm_sysreg_write:
    if (rx_packet->data_len == 5) {
      LONG data = (uint32_t)rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
          | (uint32_t)rx_packet->data[3] << 8;

      return (sysreg_write(rx_packet->data[0], &data) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
    }
    return false;
  case cmd_bdm_adreg_read:
    if (rx_packet->data_len == 1) {
      uint8_t ret[4];
      return (adreg_read((LONG*)ret, rx_packet->data[0]) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, ret, 4, cmd_term_ack);
    }
    return false;
  case cmd_bdm_adreg_write:
    if (rx_packet->data_len == 5) {
      LONG data = (uint32_t)rx_packet->data[4] | (uint32_t)rx_packet->data[1] << 24 | (uint32_t)rx_packet->data[2] << 16
          | (uint32_t)rx_packet->data[3] << 8;

      return (adreg_write(rx_packet->data[0], &data) == TERM_OK)
          && prepareReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack);
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
    return (pin == 1) && prepareReplyPacket(tx_packet, rx_packet, &pin, 1, cmd_term_ack);
  }

  return false;
}

bool readflash(LONG start_addr, LONG size) {
  char result;
  WORD curr_word;
  WORD *buf_ptr;
  LONG curr_addr;
  uint8_t flash_buf[256];
  packet_t tx_packet = {0}, rx_packet = {0};
  uint8_t buffer[300] = {0};
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
      buffsize = covertPacketToBuffer(&tx_packet, buffer);
      usb_send(&USBD1, EP_IN, buffer, buffsize);
      buf_ptr = (WORD*)flash_buf;
    }
  }

  return result == TERM_OK;
}

bool writeflash(char *flash_type, LONG start_addr, LONG size) {
  packet_t tx_packet = {0}, rx_packet = {0};
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
  buffsize = covertPacketToBuffer(&tx_packet, buffer);
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
    buffsize = covertPacketToBuffer(&tx_packet, buffer);
    usb_send(&USBD1, EP_IN, buffer, buffsize);
  } while (status == true);

  // reset flash
  return (reset_func() && status);
}
