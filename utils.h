#ifndef UTILS_H_
#define UTILS_H_

#include "hal.h"

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

#define FDCAN_DATA_BYTES_8  ((uint32_t)0x00000004U)

typedef struct
{
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
  uint32_t      StandardFilterSA;
  uint32_t      ExtendedFilterSA;
  uint32_t      RxFIFO0SA;
  uint32_t      RxFIFO1SA;
  uint32_t      RxBufferSA;
  uint32_t      TxEventFIFOSA;
  uint32_t      TxBufferSA;
  uint32_t      TxFIFOQSA;
  uint32_t      TTMemorySA;
  uint32_t      EndAddress;
} CAN_RamAddress;

typedef struct {
  uint32_t      MessageRAMOffset;
  uint32_t      StdFiltersNbr;
  uint32_t      ExtFiltersNbr;
  uint32_t      RxFifo0ElmtsNbr;
  uint32_t      RxFifo0ElmtSize;
  uint32_t      RxFifo1ElmtsNbr;
  uint32_t      RxFifo1ElmtSize;
  uint32_t      RxBuffersNbr;
  uint32_t      RxBufferSize;
  uint32_t      TxEventsNbr;
  uint32_t      TxBuffersNbr;
  uint32_t      TxFifoQueueElmtsNbr;
  uint32_t      TxElmtSize;
} CANRamConfig;

void swab(uint16_t *word);
bool canBaudRate(CANConfig *can_cfg, uint32_t can_baudrate);
bool canMemorryConfig(CANDriver *canp, CANConfig *can_cfg, CANRamConfig *cfg, CAN_RamAddress *msgRam);
void canGlobalFilter(CANConfig *can_cfg, uint32_t NonMatchingStd, uint32_t NonMatchingExt,
                     uint32_t RejectRemoteStd, uint32_t RejectRemoteExt);
void canFilter(CAN_RamAddress *msgRam, CAN_Filter *filter);
#endif /* UTILS_H_ */
