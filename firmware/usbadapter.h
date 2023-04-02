// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#ifndef USBADAPTER_H
#define USBADAPTER_H

#include "hal.h"

#define IN_PACKETSIZE  0x40
#define OUT_PACKETSIZE 0x40
#define EP_IN 2
#define EP_OUT 2
extern uint8_t receiveBuf[OUT_PACKETSIZE*2];
extern uint8_t transferBuf[IN_PACKETSIZE*2];

extern const USBConfig usb_config;
extern const SerialUSBConfig serusbcfg1;
extern SerialUSBDriver SDU1;
extern void dataReceived(USBDriver *usbp, usbep_t ep);
extern void usb_send(USBDriver *usbp, usbep_t ep, const uint8_t *buf, size_t n);
extern bool start_receive(USBDriver *usbp, usbep_t ep, uint8_t *buf, size_t n);
extern bool combiConfigureHookI(USBDriver *usbp);

#endif
