// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski.
 *
 */

#include "canutils.h"
#include "usbadapter.h"

bool (*hscan_rx_cb)(void*, packet_t*);
bool (*swcan_rx_cb)(void*, packet_t*);

void registerHsCanCallback(bool (*cb)(void *rxmsg, packet_t *packet)) {
  hscan_rx_cb = cb;
}

void registerSwCanCallback(bool (*cb)(void *rxmsg, packet_t *packet)) {
  swcan_rx_cb = cb;
}

const uint32_t CvtEltSizeArr[] = {
    0, 0, 0, 0, 0, 1, 2, 3, 4, 0,
    5, 0, 0, 0, 6, 0, 0, 0, 7
};

static const uint8_t dlc2len[] = {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 12, 16, 20, 24, 32, 48, 64
};

uint8_t can_fd_dlc2len(uint8_t dlc) {
  return dlc2len[dlc & 0x0F];
}

uint8_t CvtEltSize(uint8_t e) {
  return CvtEltSizeArr[e];
}

void canGlobalFilter(CANConfig *can_cfg, uint32_t NonMatchingStd, uint32_t NonMatchingExt,
                     uint32_t RejectRemoteStd, uint32_t RejectRemoteExt) {
  can_cfg->RXGFC = ((NonMatchingStd << FDCAN_GFC_ANFS_Pos) |
                 (NonMatchingExt << FDCAN_GFC_ANFE_Pos)  |
                 (RejectRemoteStd << FDCAN_GFC_RRFS_Pos) |
                 (RejectRemoteExt << FDCAN_GFC_RRFE_Pos));
}

/**
 * Try to compute the timing registers for the can interface and set the configuration
 * https://github.com/paparazzi/paparazzi/blob/master/sw/airborne/arch/chibios/modules/uavcan/uavcan.c
 */
bool canBaudRate(CANConfig *can_cfg, uint32_t can_baudrate, uint32_t *sjw, uint32_t *bsp) {
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

  if (bsp != NULL)
    *bsp = sample_point_permill/1000;
  if (sjw != NULL)
    *sjw = bs1;

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

THD_FUNCTION(hscan_rx, p) {
  if (p != NULL) {
    hscan_rx_cb = p;
  }
  event_listener_t el;
  static CANRxFrame rxmsg = {};
  static uint8_t size = 0;
  static uint8_t buffer[80] = {0};
  static uint8_t canbuff[66] = {0};
  packet_t tx_packet = {.data = canbuff};
  chEvtRegister(&CAND1.rxfull_event, &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
      continue;
    }
    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK ) {
      if (hscan_rx_cb == NULL)
        continue;
      if (!hscan_rx_cb(&rxmsg, &tx_packet))
        continue;
      size = covertPacketToBuffer(&tx_packet, buffer);
      usb_send(&USBD1, EP_IN, buffer, size);
    }
  }
  chEvtUnregister(&CAND1.rxfull_event, &el);
}

THD_FUNCTION(swcan_rx, p) {
  if (p != NULL) {
    swcan_rx_cb = p;
  }
  event_listener_t el;
  CANRxFrame rxmsg = {};
  uint8_t size = 0;
  uint8_t buffer[80] = {0};
  uint8_t packetbuff[66] = {0};
  packet_t tx_packet = {.data = packetbuff};
  chEvtRegister(&CAND2.rxfull_event, &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
      continue;
    }
    while (canReceive(&CAND2, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK ) {
      if (swcan_rx_cb == NULL)
        continue;
      if (!swcan_rx_cb(&rxmsg, &tx_packet))
        continue;
      size = covertPacketToBuffer(&tx_packet, buffer);
      usb_send(&USBD1, EP_IN, buffer, size);
    }
  }
  chEvtUnregister(&CAND2.rxfull_event, &el);
}
