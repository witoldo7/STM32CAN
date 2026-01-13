// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#ifndef USBADAPTER_H
#define USBADAPTER_H

#include "hal.h"
#include "utils.h"

typedef struct {
  uint16_t size;
  uint8_t data[];
} usb_packet;

extern const USBConfig usb_config;
extern const SerialUSBConfig serusbcfg1;
extern SerialUSBDriver SDU1;

void addUsbMsgToMailbox(packet_t *packet);

THD_FUNCTION(usbThd_rx, p);
THD_FUNCTION(usbThd_tx, p);
THD_FUNCTION(cmdThd, p);
#endif
