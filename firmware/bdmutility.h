// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#ifndef BDMUTILITY_H_
#define BDMUTILITY_H_

#include "ch.h"
#include "utils.h"

enum bdm_command_t {
  cmd_bdm_stop_chip = 0x40,
  cmd_bdm_reset_chip = 0x41,
  cmd_bdm_run_chip = 0x42,
  cmd_bdm_step_chip = 0x43,
  cmd_bdm_restart_chip = 0x44,
  cmd_bdm_mem_read = 0x45,
  cmd_bdm_mem_write = 0x46,
  cmd_bdm_sysreg_read = 0x47,
  cmd_bdm_sysreg_write = 0x48,
  cmd_bdm_adreg_read = 0x49,
  cmd_bdm_adreg_write = 0x4A,
  cmd_bdm_read_flash = 0x4B,
  cmd_bdm_erase_flash = 0x4C,
  cmd_bdm_write_flash = 0x4D,
  cmd_bdm_pinstate = 0x4E,
};

extern bool exec_cmd_bdm(packet_t *rx_packet, packet_t *tx_packet);

#endif /* BDMUTILITY_H_ */
