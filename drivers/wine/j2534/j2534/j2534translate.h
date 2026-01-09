// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#ifndef J2534TRANSLATE_H_
#define J2534TRANSLATE_H_

char* translateError(uint32_t err);
char* translateErrorDetail(uint32_t err);
char* translateIoctl(uint32_t ioctl);
char* translateParam(uint32_t param);
char* translateProtocol(uint32_t protocol);
char* translateFilterType(uint32_t type);

#endif /* J2534TRANSLATE_H_ */
