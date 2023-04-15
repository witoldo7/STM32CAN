// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski.
 *
 */

#ifndef CANUTILS_H_
#define CANUTILS_H_

#include "utils.h"

#define FDCAN_MESSAGE_RAM_SIZE 0x2800U
#define FDCAN_MESSAGE_RAM_END_ADDRESS (SRAMCAN_BASE + FDCAN_MESSAGE_RAM_SIZE - 0x4U)

#define FDCAN_FILTER_RANGE         ((uint32_t)0x00000000U) /*!< Range filter from FilterID1 to FilterID2                        */
#define FDCAN_FILTER_DUAL          ((uint32_t)0x00000001U) /*!< Dual ID filter for FilterID1 or FilterID2                       */
#define FDCAN_FILTER_MASK          ((uint32_t)0x00000002U) /*!< Classic filter: FilterID1 = filter, FilterID2 = mask            */
#define FDCAN_FILTER_RANGE_NO_EIDM ((uint32_t)0x00000003U) /*!< Range filter from FilterID1 to FilterID2, EIDM mask not applied */

#define FDCAN_FILTER_DISABLE       ((uint32_t)0x00000000U) /*!< Disable filter element                                    */
#define FDCAN_FILTER_TO_RXFIFO0    ((uint32_t)0x00000001U) /*!< Store in Rx FIFO 0 if filter matches                      */
#define FDCAN_FILTER_TO_RXFIFO1    ((uint32_t)0x00000002U) /*!< Store in Rx FIFO 1 if filter matches                      */
#define FDCAN_FILTER_REJECT        ((uint32_t)0x00000003U) /*!< Reject ID if filter matches                               */
#define FDCAN_FILTER_HP            ((uint32_t)0x00000004U) /*!< Set high priority if filter matches                       */
#define FDCAN_FILTER_TO_RXFIFO0_HP ((uint32_t)0x00000005U) /*!< Set high priority and store in FIFO 0 if filter matches   */
#define FDCAN_FILTER_TO_RXFIFO1_HP ((uint32_t)0x00000006U) /*!< Set high priority and store in FIFO 1 if filter matches   */
#define FDCAN_FILTER_TO_RXBUFFER   ((uint32_t)0x00000007U) /*!< Store into Rx Buffer, configuration of FilterType ignored */

#define FDCAN_STANDARD_ID ((uint32_t)0x00000000U) /*!< Standard ID element */
#define FDCAN_EXTENDED_ID ((uint32_t)0x40000000U) /*!< Extended ID element */

#define FDCAN_ACCEPT_IN_RX_FIFO0 ((uint32_t)0x00000000U) /*!< Accept in Rx FIFO 0 */
#define FDCAN_ACCEPT_IN_RX_FIFO1 ((uint32_t)0x00000001U) /*!< Accept in Rx FIFO 1 */
#define FDCAN_REJECT             ((uint32_t)0x00000002U) /*!< Reject              */

#define FDCAN_FILTER_REMOTE ((uint32_t)0x00000000U) /*!< Filter remote frames */
#define FDCAN_REJECT_REMOTE ((uint32_t)0x00000001U) /*!< Reject all remote frames */

#define FDCAN_DATA_BYTES_8  ((uint32_t)0x00000004U) /*!< 8 bytes data field  */
#define FDCAN_DATA_BYTES_12 ((uint32_t)0x00000005U) /*!< 12 bytes data field */
#define FDCAN_DATA_BYTES_16 ((uint32_t)0x00000006U) /*!< 16 bytes data field */
#define FDCAN_DATA_BYTES_20 ((uint32_t)0x00000007U) /*!< 20 bytes data field */
#define FDCAN_DATA_BYTES_24 ((uint32_t)0x00000008U) /*!< 24 bytes data field */
#define FDCAN_DATA_BYTES_32 ((uint32_t)0x0000000AU) /*!< 32 bytes data field */
#define FDCAN_DATA_BYTES_48 ((uint32_t)0x0000000EU) /*!< 48 bytes data field */
#define FDCAN_DATA_BYTES_64 ((uint32_t)0x00000012U) /*!< 64 bytes data field */

#define CAN_CTRLMODE_LOOPBACK       0x01    /* Loopback mode */
#define CAN_CTRLMODE_LISTENONLY     0x02    /* Listen-only mode */
#define CAN_CTRLMODE_3_SAMPLES      0x04    /* Triple sampling mode */
#define CAN_CTRLMODE_ONE_SHOT       0x08    /* One-Shot mode */
#define CAN_CTRLMODE_BERR_REPORTING 0x10    /* Bus-error reporting */
#define CAN_CTRLMODE_FD             0x20    /* CAN FD mode */
#define CAN_CTRLMODE_PRESUME_ACK    0x40    /* Ignore missing CAN ACKs */
#define CAN_CTRLMODE_FD_NON_ISO     0x80    /* CAN FD in non-ISO mode */
#define CAN_CTRLMODE_CC_LEN8_DLC    0x100   /* Classic CAN DLC option */
#define CAN_CTRLMODE_TDC_AUTO       0x200   /* CAN transiver automatically calculates TDCV */
#define CAN_CTRLMODE_TDC_MANUAL     0x400   /* TDCV is manually set up by user */

typedef struct {
  uint32_t IdType;
  uint32_t FilterIndex;
  uint32_t FilterType;
  uint32_t FilterConfig;
  uint32_t FilterID1;
  uint32_t FilterID2;
  uint32_t RxBufferIndex;
  uint32_t IsCalibrationMsg;
} CAN_Filter;

typedef struct {
  uint32_t StandardFilterSA;
  uint32_t ExtendedFilterSA;
  uint32_t RxFIFO0SA;
  uint32_t RxFIFO1SA;
  uint32_t RxBufferSA;
  uint32_t TxEventFIFOSA;
  uint32_t TxBufferSA;
  uint32_t TxFIFOQSA;
  uint32_t TTMemorySA;
  uint32_t EndAddress;
} CAN_RamAddress;

typedef struct {
  uint32_t MessageRAMOffset;
  uint32_t StdFiltersNbr;
  uint32_t ExtFiltersNbr;
  uint32_t RxFifo0ElmtsNbr;
  uint32_t RxFifo0ElmtSize;
  uint32_t RxFifo1ElmtsNbr;
  uint32_t RxFifo1ElmtSize;
  uint32_t RxBuffersNbr;
  uint32_t RxBufferSize;
  uint32_t TxEventsNbr;
  uint32_t TxBuffersNbr;
  uint32_t TxFifoQueueElmtsNbr;
  uint32_t TxElmtSize;
} CANRamConfig;

uint8_t can_fd_dlc2len(uint8_t dlc);
bool canBaudRate(CANConfig *can_cfg, uint32_t can_baudrate, uint32_t *sjw, uint32_t *bsp);
bool canMemorryConfig(CANDriver *canp, CANConfig *can_cfg, CANRamConfig *cfg, CAN_RamAddress *msgRam);
void canGlobalFilter(CANConfig *can_cfg, uint32_t NonMatchingStd, uint32_t NonMatchingExt, uint32_t RejectRemoteStd,
                     uint32_t RejectRemoteExt);
void canFilter(CAN_RamAddress *msgRam, CAN_Filter *filter);
uint8_t CvtEltSize(uint8_t e);
void registerHsCanCallback(bool (*cb)(void *rxmsg, packet_t *packet));
void registerSwCanCallback(bool (*cb)(void *rxmsg, packet_t *packet));
THD_FUNCTION(hscan_rx, p);
THD_FUNCTION(swcan_rx, p);

#endif /* CANUTILS_H_ */
