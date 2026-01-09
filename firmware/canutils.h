// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski.
 *
 */

#ifndef CANUTILS_H_
#define CANUTILS_H_

#include "utils.h"

#define FDCAN_ACCEPT_IN_RX_FIFO0 ((uint32_t)0x00000000U) /*!< Accept in Rx FIFO 0 */
#define FDCAN_ACCEPT_IN_RX_FIFO1 ((uint32_t)0x00000001U) /*!< Accept in Rx FIFO 1 */
#define FDCAN_REJECT ((uint32_t)0x00000002U)             /*!< Reject              */

#define FDCAN_FILTER_REMOTE ((uint32_t)0x00000000U) /*!< Filter remote frames */
#define FDCAN_REJECT_REMOTE ((uint32_t)0x00000001U) /*!< Reject all remote frames */

#define CAN_CTRLMODE_LOOPBACK 0x01       /* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY 0x02     /* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES 0x04      /* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT 0x08       /* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING 0x10 /* Bus-error reporting */
#define CAN_CTRLMODE_FD 0x20             /* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK 0x40    /* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO 0x80     /* CAN FD in non-ISO mode */
#define CAN_CTRLMODE_CC_LEN8_DLC 0x100   /* Classic CAN DLC option */
#define CAN_CTRLMODE_TDC_AUTO 0x200      /* CAN transiver automatically calculates TDCV */
#define CAN_CTRLMODE_TDC_MANUAL 0x400    /* TDCV is manually set up by user */

uint8_t can_fd_dlc2len(uint8_t dlc);
bool canBaudRate(CANConfig *can_cfg, uint32_t can_baudrate, uint32_t *sjw, uint32_t *bsp);
void canGlobalFilter(CANConfig *can_cfg, uint32_t NonMatchingStd, uint32_t NonMatchingExt, uint32_t RejectRemoteStd,
                     uint32_t RejectRemoteExt);

uint8_t CvtEltSize(uint8_t e);
void registerHsCanCallback(bool (*cb)(void *conn, void *rxmsg, packet_t *packet), void *conn);
void registerSwCanCallback(bool (*cb)(void *conn, void *rxmsg, packet_t *packet), void *conn);
THD_FUNCTION(hscan_rx, p);
THD_FUNCTION(swcan_rx, p);

#endif /* CANUTILS_H_ */
