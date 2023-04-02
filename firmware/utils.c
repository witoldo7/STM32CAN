// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#include <utils.h>
#include "hal.h"
#include "string.h"

//------------------------ADC-BEGIN-------------------------

#define ADC_GRP1_NUM_CHANNELS       1
#define ADC_GRP1_BUF_DEPTH      1

/* Buffers are allocated with size and address aligned to the cache
   line size.*/
#if CACHE_LINE_SIZE > 0
CC_ALIGN_DATA(CACHE_LINE_SIZE)
#endif
adcsample_t samples1[CACHE_SIZE_ALIGN(adcsample_t, ADC_GRP1_NUM_CHANNELS * ADC_GRP1_BUF_DEPTH)];

/*
 * ADC conversion group 1.
 * Mode:        One shot, 1 channel, SW triggered.
 * Channels:    IN5.
 */
const ADCConversionGroup adcgrpcfg1 = {
  .circular     = false,
  .num_channels = ADC_GRP1_NUM_CHANNELS,
  .end_cb       = NULL,
  .error_cb     = NULL,
  .cfgr         = 0U,
  .cfgr2        = 0U,
  .ccr          = 0U,
  .pcsel        = ADC_SELMASK_IN5,
  .ltr1         = 0x00000000U,
  .htr1         = 0x03FFFFFFU,
  .ltr2         = 0x00000000U,
  .htr2         = 0x03FFFFFFU,
  .ltr3         = 0x00000000U,
  .htr3         = 0x03FFFFFFU,
  .smpr         = {
    ADC_SMPR1_SMP_AN5(ADC_SMPR_SMP_810P5),
    0U
  },
  .sqr          = {
    ADC_SQR1_SQ1_N(ADC_CHANNEL_IN5),
    0U,
    0U,
    0U
  }
};

uint16_t getSupplyVoltage(void) {
  uint16_t voltage = 0;
  adcConvert(&ADCD1, &adcgrpcfg1, samples1, ADC_GRP1_BUF_DEPTH);
  voltage = samples1[0] / 3;
  cacheBufferInvalidate(samples1, sizeof (samples1) / sizeof (adcsample_t));
  return voltage;
}

void get_vbat(uint8_t *vbat) {
  uint16_t voltage = getSupplyVoltage();
  memcpy(vbat, &voltage, 2);
}
//------------------------ADC-END-----------------------------

//------------------------FDCAN-BEGIN-----------------------------
#define FDCAN_MESSAGE_RAM_SIZE 0x2800U
#define FDCAN_MESSAGE_RAM_END_ADDRESS (SRAMCAN_BASE + FDCAN_MESSAGE_RAM_SIZE - 0x4U)

static const uint8_t dlc2len[] = {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 12, 16, 20, 24, 32, 48, 64
};

uint8_t can_fd_dlc2len(uint8_t dlc) {
  return dlc2len[dlc & 0x0F];
}

void canFilter(CAN_RamAddress *msgRam, CAN_Filter *filter) {
  uint32_t filterElementW1;
  uint32_t filterElementW2;
  uint32_t *filterAddress;

  if (filter->IdType == FDCAN_STANDARD_ID) {
    if (filter->FilterConfig == FDCAN_FILTER_TO_RXBUFFER) {
      filterElementW1 = ((FDCAN_FILTER_TO_RXBUFFER << 27U)       |
                           (filter->FilterID1 << 16U)       |
                           (filter->IsCalibrationMsg << 8U) |
                           filter->RxBufferIndex);
    } else {
      filterElementW1 = ((filter->FilterType << 30U)   |
                         (filter->FilterConfig << 27U) |
                         (filter->FilterID1 << 16U)    |
                         filter->FilterID2);
    }

    filterAddress = (uint32_t *)(msgRam->StandardFilterSA + (filter->FilterIndex * 4U));
    *filterAddress = filterElementW1;
  } else {
    filterElementW1 = ((filter->FilterConfig << 29U) | filter->FilterID1);
    if (filter->FilterConfig == FDCAN_FILTER_TO_RXBUFFER) {
      filterElementW2 = filter->RxBufferIndex;
    } else {
      filterElementW2 = ((filter->FilterType << 30U) | filter->FilterID2);
    }

    filterAddress = (uint32_t *)(msgRam->ExtendedFilterSA + (filter->FilterIndex * 4U * 2U));
    *filterAddress = filterElementW1;
    filterAddress++;
    *filterAddress = filterElementW2;
  }
}

void canGlobalFilter(CANConfig *can_cfg, uint32_t NonMatchingStd, uint32_t NonMatchingExt,
                     uint32_t RejectRemoteStd, uint32_t RejectRemoteExt) {
  can_cfg->GFC = ((NonMatchingStd << FDCAN_GFC_ANFS_Pos) |
                 (NonMatchingExt << FDCAN_GFC_ANFE_Pos)  |
                 (RejectRemoteStd << FDCAN_GFC_RRFS_Pos) |
                 (RejectRemoteExt << FDCAN_GFC_RRFE_Pos));
}

bool canMemorryConfig(CANDriver *canp, CANConfig *can_cfg, CANRamConfig *cfg, CAN_RamAddress *msgRam) {
  uint32_t RAMcounter;
  uint32_t startAddress = cfg->MessageRAMOffset;

  startAddress = cfg->MessageRAMOffset;
  can_cfg->SIDFC = (cfg->StdFiltersNbr << FDCAN_SIDFC_LSS_Pos) | (startAddress << FDCAN_SIDFC_FLSSA_Pos);
  startAddress += cfg->StdFiltersNbr;

  can_cfg->XIDFC = (cfg->ExtFiltersNbr << FDCAN_XIDFC_FLESA_Pos) | (startAddress << FDCAN_XIDFC_LSE_Pos);
  startAddress += cfg->ExtFiltersNbr * 2U;

  can_cfg->RXF0C = (cfg->RxFifo0ElmtsNbr << FDCAN_RXF0C_F0S_Pos) | (startAddress << FDCAN_RXF0C_F0SA_Pos);
  startAddress += cfg->RxFifo0ElmtsNbr * cfg->RxFifo0ElmtSize;

  can_cfg->RXF1C = (cfg->RxFifo1ElmtsNbr << FDCAN_RXF1C_F1S_Pos) | (startAddress << FDCAN_RXF1C_F1SA_Pos);
  startAddress += cfg->RxFifo1ElmtsNbr * cfg->RxFifo1ElmtSize;

  can_cfg->RXBC = startAddress << FDCAN_RXBC_RBSA_Pos;
  startAddress += cfg->RxBuffersNbr * cfg->RxBufferSize;

  can_cfg->TXEFC = cfg->TxEventsNbr << FDCAN_TXEFC_EFS_Pos;
  startAddress += cfg->TxEventsNbr * 2U;

  can_cfg->TXBC  = (cfg->TxBuffersNbr << FDCAN_TXBC_NDTB_Pos) | (cfg->TxFifoQueueElmtsNbr << FDCAN_TXBC_TFQS_Pos)
      | (startAddress << FDCAN_TXBC_TBSA_Pos);

  msgRam->StandardFilterSA = (uint32_t)(canp->ram_base + (cfg->MessageRAMOffset * 4U));
  msgRam->ExtendedFilterSA = msgRam->StandardFilterSA + (cfg->StdFiltersNbr * 4U);
  msgRam->RxFIFO0SA = msgRam->ExtendedFilterSA + (cfg->ExtFiltersNbr * 2U * 4U);
  msgRam->RxFIFO1SA = msgRam->RxFIFO0SA + (cfg->RxFifo0ElmtsNbr * cfg->RxFifo0ElmtSize * 4U);
  msgRam->RxBufferSA = msgRam->RxFIFO1SA + (cfg->RxFifo1ElmtsNbr * cfg->RxFifo1ElmtSize * 4U);
  msgRam->TxEventFIFOSA = msgRam->RxBufferSA + (cfg->RxBuffersNbr * cfg->RxBufferSize * 4U);
  msgRam->TxBufferSA = msgRam->TxEventFIFOSA + (cfg->TxEventsNbr * 2U * 4U);
  msgRam->TxFIFOQSA = msgRam->TxBufferSA + (cfg->TxBuffersNbr * cfg->TxElmtSize * 4U);
  msgRam->EndAddress = msgRam->TxFIFOQSA + (cfg->TxFifoQueueElmtsNbr * cfg->TxElmtSize * 4U);

  if (msgRam->EndAddress > FDCAN_MESSAGE_RAM_END_ADDRESS) {
    return false;
  } else {
    for (RAMcounter = msgRam->StandardFilterSA; RAMcounter < msgRam->EndAddress; RAMcounter += 4U)
    {
      *(uint32_t *)(RAMcounter) = 0x00000000;
    }
  }
  return true;
}

/**
 * Try to compute the timing registers for the can interface and set the configuration
 * https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/arch/chibios/modules/uavcan/uavcan.c
 */
bool canBaudRate(CANConfig *can_cfg, uint32_t can_baudrate) {
  if (can_baudrate < 1) {
    return false;
  }

  // Hardware configurationn
  const uint32_t pclk = STM32_FDCANCLK;

  static const int MaxBS1 = 16;
  static const int MaxBS2 = 8;

  /*
    * Ref. "Automatic Baudrate Detection in CANopen Networks", U. Koppe, MicroControl GmbH & Co. KG
    *      CAN in Automation, 2003
    *
    * According to the source, optimal quanta per bit are:
    *   Bitrate        Optimal Maximum
    *   1000 kbps      8       10
    *   500  kbps      16      17
    *   250  kbps      16      17
    *   125  kbps      16      17
    */
  const int max_quanta_per_bit = (can_baudrate >= 1000000) ? 10 : 17;
  static const int MaxSamplePointLocation = 880;

  /*
    * Computing (prescaler * BS):
    *   BITRATE = 1 / (PRESCALER * (1 / PCLK) * (1 + BS1 + BS2))       -- See the Reference Manual
    *   BITRATE = PCLK / (PRESCALER * (1 + BS1 + BS2))                 -- Simplified
    * let:
    *   BS = 1 + BS1 + BS2                                             -- Number of time quanta per bit
    *   PRESCALER_BS = PRESCALER * BS
    * ==>
    *   PRESCALER_BS = PCLK / BITRATE
    */
  const uint32_t prescaler_bs = pclk / can_baudrate;

// Searching for such prescaler value so that the number of quanta per bit is highest.
  uint8_t bs1_bs2_sum = max_quanta_per_bit - 1;
  while ((prescaler_bs % (1 + bs1_bs2_sum)) != 0) {
    if (bs1_bs2_sum <= 2) {
      return false;          // No solution
    }
    bs1_bs2_sum--;
  }

  const uint32_t prescaler = prescaler_bs / (1 + bs1_bs2_sum);
  if ((prescaler < 1U) || (prescaler > 1024U)) {
    return false;              // No solution
  }

  /*
    * Now we have a constraint: (BS1 + BS2) == bs1_bs2_sum.
    * We need to find the values so that the sample point is as close as possible to the optimal value.
    *
    *   Solve[(1 + bs1)/(1 + bs1 + bs2) == 7/8, bs2]  (* Where 7/8 is 0.875, the recommended sample point location *)
    *   {{bs2 -> (1 + bs1)/7}}
    *
    * Hence:
    *   bs2 = (1 + bs1) / 7
    *   bs1 = (7 * bs1_bs2_sum - 1) / 8
    *
    * Sample point location can be computed as follows:
    *   Sample point location = (1 + bs1) / (1 + bs1 + bs2)
    *
    * Since the optimal solution is so close to the maximum, we prepare two solutions, and then pick the best one:
    *   - With rounding to nearest
    *   - With rounding to zero
    */
// First attempt with rounding to nearest
  uint8_t bs1 = ((7 * bs1_bs2_sum - 1) + 4) / 8;
  uint8_t bs2 = bs1_bs2_sum - bs1;
  uint16_t sample_point_permill = 1000 * (1 + bs1) / (1 + bs1 + bs2);

// Second attempt with rounding to zero
  if (sample_point_permill > MaxSamplePointLocation) {
    bs1 = (7 * bs1_bs2_sum - 1) / 8;
    bs2 = bs1_bs2_sum - bs1;
    sample_point_permill = 1000 * (1 + bs1) / (1 + bs1 + bs2);
  }

  /*
    * Final validation
    * Helpful Python:
    * def sample_point_from_btr(x):
    *     assert 0b0011110010000000111111000000000 & x == 0
    *     ts2,ts1,brp = (x>>20)&7, (x>>16)&15, x&511
    *     return (1+ts1+1)/(1+ts1+1+ts2+1)
    *
    */
  if ((can_baudrate != (pclk / (prescaler * (1 + bs1 + bs2)))) || (bs1 < 1) || (bs1 > MaxBS1) || (bs2 < 1)
      || (bs2 > MaxBS2)) {
    return false;
  }

  // Configure the interface
  can_cfg->NBTP = (0 << FDCAN_NBTP_NSJW_Pos) | ((bs1 - 1) << FDCAN_NBTP_NTSEG1_Pos) | ((
                          bs2 - 1) << FDCAN_NBTP_NTSEG2_Pos) | ((prescaler - 1) << FDCAN_NBTP_NBRP_Pos);
  can_cfg->CCCR = FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE;

  return true;
}

//------------------------FDCAN-END-----------------------------

//-------------------------OTHERS-------------------------------
void swab(uint16_t *word) {
  uint16_t tmp;
  if (word != 0) {
    tmp = *word;
    *word = *word << 8;
    *word = tmp >> 8 | *word;
  }
  return;
}

bool prepareReplyPacket(packet_t *reply, packet_t *source, uint8_t *data,
                          uint16_t data_len, uint8_t term) {
  reply->cmd_code = source->cmd_code;
  reply->data_len = data_len;
  if (data_len != 0) {
    memcpy(reply->data, data, data_len);
  }
  reply->term = term;
  return true;
}

uint8_t covertPacketToBuffer(packet_t *packet, uint8_t *buffer) {
  uint8_t size = 0;
  buffer[0] = packet->cmd_code;
  buffer[1] = (uint8_t)(packet->data_len >> 8);
  buffer[2] = (uint8_t)packet->data_len;
  if (packet->data_len != 0) {
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
