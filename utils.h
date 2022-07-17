/*
 * utils.h
 *
 *  Created on: 17 lip 2022
 *      Author: witold
 */

#ifndef UTILS_H_
#define UTILS_H_
#include "hal.h"

void swab(uint16_t *word);
bool canBaudRate(CANConfig *can_cfg, uint32_t can_baudrate);

#endif /* UTILS_H_ */
