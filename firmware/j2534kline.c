// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */
#include "hal.h"
#include "j2534.h"
#include "j2534kline.h"
#include <string.h>

// Timings
#define MAXSENDTIME 500             // Max read timeout
#define ISORequestByteDelay 10  // Time delay between byte to send.
#define ISORequestDelay 50      // Time between requests.

uint32_t sendRequest(j2534_conn* conn, const uint8_t *request, uint8_t reqLen, uint8_t *response, uint8_t respLen);

// K-Line
SerialConfig uart2Cfg = {
 10400, // bit rate
 0,
 USART_CR2_STOP1_BITS,
 0
};

uint8_t calcChecksum(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++) {
        crc = crc + data[i];
    }
    return crc;
}

uint32_t handle_connect_kline(j2534_conn* conn) {
  uart2Cfg.speed = conn->DataRate;
  sdStart(&SD2, &uart2Cfg);
  return STATUS_NOERROR;
}

uint32_t handle_disconnect_kline(j2534_conn* conn) {
  (void)conn;
  sdStop(&SD2);
  return STATUS_NOERROR;
}

uint32_t stop_filter_kline(j2534_conn* conn, uint32_t idx) {
  (void)conn;
  (void)idx;
  return STATUS_NOERROR;
}

uint32_t start_filter_kline(j2534_conn* conn, uint8_t* data, uint32_t* idx) {
  (void)conn;
  (void)data;
  *idx=1;
  return STATUS_NOERROR;
}

uint32_t write_message_kline(j2534_conn* conn, uint32_t timeout, uint16_t len, uint8_t* data) {
  (void)conn;
  (void)timeout;
  (void)len;
  (void)data;
  return STATUS_NOERROR;
}

uint32_t start_periodic_msg_kline(j2534_conn* conn, uint8_t data) {
  (void)conn;
  (void)data;
  return STATUS_NOERROR;
}

uint32_t stop_periodic_msg_kline(j2534_conn* conn, uint32_t msg) {
  (void)conn;
  (void)msg;
  return STATUS_NOERROR;
}

uint32_t ioctl_clear_filters_kline(j2534_conn* conn) {
  (void)conn;
  return STATUS_NOERROR;
}

uint32_t ioctl_datarate_kline(j2534_conn* conn) {
  sdStop(&SD2);
  uart2Cfg.speed = conn->DataRate;
  sdStart(&SD2, &uart2Cfg);
  return STATUS_NOERROR;
}

uint32_t ioctl_loopback_kline(j2534_conn* conn) {
  (void)conn;
  return STATUS_NOERROR;
}

/* Suzuki DL1000
 * SDS: 81  12  F1 (81) 05
 *       |   |   |   |   |________________CRC
 *       |   |   |   |___________________Data
 *       |   |   |_____________Tester address
 *       |   |________Destination ECU address
 *       |__________________ 80 + 1 byte data
 *
 * ECU: 80  F1  12  03 (C1  EA  8F) C0
 *       |   |   |   |  ---DATA---   |____CRC
 *       |   |   |   |____________Data length
 *       |   |   |_____________Tester address
 *       |   |________Destination ECU address
 *       |____ 80 + variable length in byte 4
 *
 * Saab 9-5 -> T7 (tech2win) miss crc
 * REQ: 81  41  f1  (81)
 * ECU: 83  f1  41  (c1  6b  8f)
 */
uint32_t ioctl_fast_init_kline(j2534_conn* conn, uint8_t* in, uint8_t* out) {
  //in b0 len, [b1:bx] data
  j2534_protocol_cfg *pcfg = conn->pcfg;
  uint8_t request[5];
  uint8_t bytesToSend = in[0];

  if (bytesToSend < 4) {
    return ERR_INVALID_IOCTL_VALUE;
  }

  if ((bytesToSend < 5) && (conn->flags & ISO9141_NO_CHECKSUM))
    return ERR_INVALID_IOCTL_VALUE;

  if (!(conn->flags & ISO9141_NO_CHECKSUM)) {
    request[4] = calcChecksum(request, 4);
    bytesToSend = 5;
  }
  memcpy(request, in, bytesToSend);

  palSetLineMode(LINE_KL_TX, PAL_MODE_OUTPUT_PUSHPULL);
  if (!(conn->flags & ISO9141_K_LINE_ONLY))
    palSetLineMode(LINE_KL_RX, PAL_MODE_OUTPUT_PUSHPULL);

  palSetLine(LINE_KL_TX);
  if (!(conn->flags & ISO9141_K_LINE_ONLY))
    palSetLine(LINE_KL_RX);
  chThdSleepMilliseconds(pcfg->Tidle);

  palClearLine(LINE_KL_TX);
  if (!(conn->flags & ISO9141_K_LINE_ONLY))
    palClearLine(LINE_KL_RX);
  chThdSleepMilliseconds(pcfg->Tinil);

  palSetLine(LINE_KL_TX);
  if (!(conn->flags & ISO9141_K_LINE_ONLY))
    palSetLine(LINE_KL_RX);
  chThdSleepMilliseconds(pcfg->Twup - pcfg->Tinil);

  // Set pin mode back to UART
  palSetLineMode(LINE_KL_TX, PAL_MODE_ALTERNATE(7));
  if (!(conn->flags & ISO9141_K_LINE_ONLY))
    palSetLineMode(LINE_KL_RX, PAL_MODE_ALTERNATE(7));

  return sendRequest(conn, in+1, in[0], out+1, out[0]);
}

uint32_t ioctl_five_baud_init_kline(j2534_conn* conn, uint8_t* in, uint8_t* out) {
  (void)conn;
  (void)in;
  (void)out;
  return STATUS_NOERROR;
}

uint32_t sendRequest(j2534_conn* conn, const uint8_t *request, uint8_t reqLen, uint8_t *response, uint8_t respLen) {
  j2534_protocol_cfg *pcfg = conn->pcfg;

  for (uint8_t i = 0; i < reqLen; i++) {
      sdPut(&SD2, request[i]);
      if (pcfg->P4Min > 0)
        chThdSleepMilliseconds(pcfg->P4Min/2);
  }
  // Clear serial buffer, echo bytes, not valid data.
  chSysLock();
  iqResetI(&SD2.iqueue);
  chSysUnlock();

  // Read header with content length.
  respLen = sdReadTimeout(&SD2, response, 4, TIME_MS2I(pcfg->P1Max * 4));
  if(respLen < 4) {
    return ERR_FAILED;
  }
  uint8_t remainLen = 0;
  if ((response[0] & 0x0F) > 0) {
    remainLen = (response[1] & 0x0F) - 1;
  } else {
    remainLen = response[4];
  }
  if (respLen > 0) {
    respLen += sdReadTimeout(&SD2, response + 4, remainLen, TIME_MS2I(pcfg->P1Max * remainLen));
  }
  return STATUS_NOERROR;
}

