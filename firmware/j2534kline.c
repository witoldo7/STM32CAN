// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */
#include "hal.h"
#include "j2534.h"
#include "j2534kline.h"
#include <string.h>
#include "debug.h"
#include "usbadapter.h"

#define KLINE_WA_SIZE THD_WORKING_AREA_SIZE(512)
#define TESTER_WA_SIZE THD_WORKING_AREA_SIZE(512)
thread_t *klinetp = NULL, *testertp = NULL;
static mutex_t j2534_mtx;

SerialConfig uart2Cfg = {
    10400, // bit rate
    0,
    USART_CR2_STOP1_BITS,
    0};
uint8_t alive_buff[2] = {0};
uint8_t term_buff[2] = {0};

kwp_msg kwp_alive = {.data = alive_buff},
        kwp_term = {.data = term_buff};

static THD_FUNCTION(klineThd, p);
static THD_FUNCTION(testerThd, p);

static struct
{
  bool session_active;
  bool keep_alive;
  uint8_t tgt; /* Destination from received frame */
  uint8_t src; /* Source from received frame */
  uint16_t modeflags;

} kwp_session = {false, false, 0, 0, 0};

uint8_t calcChecksum(uint8_t *data, uint8_t len);
void calcChecksumMsg(kwp_msg *msg);
bool sendRecv(j2534_conn *conn, uint8_t *request, uint8_t len, uint8_t *response, uint8_t *resplen, bool resp);
uint16_t msgToBuffer(kwp_msg *msg, uint8_t *b);
void snedPacket(uint8_t *buff, uint8_t len, uint16_t rx_status);
bool kline_msg(uint8_t *buff, uint8_t len, packet_t *packet, uint16_t rx_status);

void snedPacket(uint8_t *buff, uint8_t len, uint16_t rx_status)
{
  static uint8_t buffer[300] = {0};
  static uint8_t pbuffer[270] = {0};
  packet_t tx_packet = {.cmd_code = cmd_j2534_read_message, .data = pbuffer, .term = cmd_term_ack};
  kline_msg(buff, len, &tx_packet, rx_status);
  size_t size = covertPacketToBuffer(&tx_packet, buffer);
  usb_send(&USBD1, EP_IN, buffer, size);
}

void printKWPmsg(kwp_msg *msg)
{
  DBG_PRNT("msg: fmt: %02x, tgt: %02x, src: %02x, len: %02x, crc: %02x, data: ",
           msg->fmt, msg->tgt, msg->src, msg->len, msg->crc);
  for (uint8_t i = 0; i < msg->len; i++)
  {
    DBG_PRNT("%02x, ", msg->data[i]);
  }
  DBG_PRNT("\r\n     buff: ");
  uint8_t buff[msg->len + 5];
  uint16_t len = msgToBuffer(msg, buff);
  for (uint8_t i = 0; i < len; i++)
  {
    DBG_PRNT("%02x, ", buff[i]);
  }
  DBG_PRNT("\r\n");
}

void build_kwp_msgs(void)
{
  kwp_alive.fmt = 0x80;
  kwp_alive.tgt = kwp_session.tgt;
  kwp_alive.src = kwp_session.src;
  kwp_alive.len = 1;
  kwp_alive.data[0] = KWP_SID_TESTER_PRESENT;
  // kwp_alive.data[1] = suppress ? 0x80 : 0x01;
  calcChecksumMsg(&kwp_alive);
  printKWPmsg(&kwp_alive);

  kwp_term.fmt = 0x80;
  kwp_term.tgt = kwp_session.tgt;
  kwp_term.src = kwp_session.src;
  kwp_term.len = 1;
  kwp_term.data[0] = KWP_SID_STOM_COMM;
  calcChecksumMsg(&kwp_term);
  printKWPmsg(&kwp_term);
}

bool kline_msg(uint8_t *buff, uint8_t len, packet_t *packet, uint16_t rx_status)
{
  packet->cmd_code = cmd_j2534_read_message;
  uint16_t protocol = ISO14230;
  memcpy(packet->data, &protocol, 2);
  memcpy(packet->data + 2, &rx_status, 2);
  packet->data[4] = len;
  if (len > 0)
    memcpy(packet->data + 5, buff, len);
  packet->data_len = 5 + len;
  return true;
}

uint16_t msgToBuffer(kwp_msg *msg, uint8_t *b)
{
  bool lenFmt = (msg->fmt & LENGTH_MASK) > 0;
  uint8_t pos = 0;

  b[pos++] = msg->fmt;
  b[pos++] = msg->tgt;
  b[pos++] = msg->src;

  if (!lenFmt)
    b[pos++] = msg->len;

  if (msg->len > 0)
    memcpy(b + pos, msg->data, msg->len);

  b[pos++ + msg->len] = msg->crc;
  return pos + msg->len;
}

void calcChecksumMsg(kwp_msg *msg)
{
  uint8_t buff[msg->len + 5];
  uint16_t len = msgToBuffer(msg, buff);
  msg->crc = calcChecksum(buff, len - 1);
}

uint8_t calcChecksum(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;

  for (uint8_t i = 0; i < len; i++)
  {
    crc = crc + data[i];
  }
  return crc;
}

bool sendRecv(j2534_conn *conn, uint8_t *request, uint8_t len, uint8_t *response, uint8_t *resplen, bool resp)
{
  chMtxLock(&j2534_mtx);
  j2534_protocol_cfg *pcfg = conn->pcfg;
  DBG_PRNT("send: len %d, data: ", len);
  for (uint8_t i = 0; i < len; i++)
  {
    DBG_PRNT("%02x ", request[i]);
    sdPut(&SD2, request[i]);
    if (pcfg->P4Min > 0 && i < (len - 1)) // skip delay after last byte
      chThdSleepMilliseconds(pcfg->P4Min / 2);
  }
  DBG_PRNT("\r\n");
  // Clear serial buffer, echo bytes, not valid data.
  uint8_t tbuff[len];
  bool status = (sdReadTimeout(&SD2, tbuff, len, TIME_MS2I(pcfg->P1Max * len)) == len);

  if (resp)
  {
    chThdSleepMilliseconds(pcfg->P2Min);
    *resplen = sdReadTimeout(&SD2, response, 4, TIME_MS2I(pcfg->P1Max * 4));

    uint8_t remainLen = 0;
    bool respSidPos = (response[0] & LENGTH_MASK) > 0;
    if (respSidPos)
    {
      remainLen += (response[0] & LENGTH_MASK);
    }
    else
    {
      remainLen += response[3];
    }

    if (remainLen > 0)
    {
      *resplen += sdReadTimeout(&SD2, response + 4, remainLen, TIME_MS2I(pcfg->P1Max * remainLen));
    }
    DBG_PRNT("recv: len %d, data: ", *resplen);
    for (uint8_t i = 0; i < *resplen; i++)
    {
      DBG_PRNT("%02x ", response[i]);
    }
    DBG_PRNT("\r\n");

    uint8_t a, b;
    b = respSidPos ? response[3] : response[4];
    a = ((request[0] & LENGTH_MASK) > 0) ? request[3] : request[4];
    status = (a + 0x40) == b;

    DBG_PRNT("status: %d\r\n", status);
  }
  chThdSleepMilliseconds(pcfg->P3Min / 2);
  chMtxUnlock(&j2534_mtx);
  return status;
}

uint32_t handle_connect_kline(j2534_conn *conn)
{
  chMtxObjectInit(&j2534_mtx);
  uart2Cfg.speed = conn->DataRate;
  sdStart(&SD2, &uart2Cfg);
  return STATUS_NOERROR;
}

bool terminate_session(j2534_conn *conn)
{
  bool status = false;
  uint8_t buff[7] = {0};
  uint8_t rlen = 0;
  uint8_t rbuff[7] = {0};
  uint8_t len = msgToBuffer(&kwp_term, buff);
  if (kwp_session.session_active)
  {
    bool status = sendRecv(conn, buff, len, rbuff, &rlen, true);
    DBG_PRNT("term ok : %d \r\n", status);
  }

  kwp_session.session_active = false;
  if (klinetp)
  {
    chThdTerminate(klinetp);
    chThdWait(klinetp);
    klinetp = NULL;
  }
  if (testertp)
  {
    chThdTerminate(testertp);
    chThdWait(testertp);
    testertp = NULL;
  }
  return status;
}

uint32_t handle_disconnect_kline(j2534_conn *conn)
{
  DBG_PRNT("Disconnect \r\n");
  terminate_session(conn);
  sdStop(&SD2);
  DBG_PRNT("disc done\r\n");
  return STATUS_NOERROR;
}

uint32_t stop_filter_kline(j2534_conn *conn, uint32_t idx)
{
  (void)conn;
  (void)idx;
  return STATUS_NOERROR;
}

uint32_t start_filter_kline(j2534_conn *conn, uint8_t *data, uint32_t *idx)
{
  (void)conn;
  (void)data;
  *idx = 1;
  return STATUS_NOERROR;
}

uint32_t write_message_kline(j2534_conn *conn, uint32_t timeout, uint16_t len, uint8_t *data)
{
  (void)timeout;
  uint8_t rlen = len, resplen = 0;
  uint8_t buff[256] = {0};

  if (!(conn->flags & ISO9141_NO_CHECKSUM))
  {
    data[len] = calcChecksum(data, len);
    rlen++;
  }
  bool status = sendRecv(conn, data, rlen, buff, &resplen, true);

  DBG_PRNT("write OK: %d \r\n", status);
  if (resplen > 0)
  {
    uint8_t buffer[32] = {0};
    snedPacket(buffer, 0, START_OF_MESSAGE);
    if (!(conn->flags & ISO9141_NO_CHECKSUM))
    {
      resplen--;
    }
    snedPacket(buff, resplen, 0);
  }
  return STATUS_NOERROR;
}

uint32_t start_periodic_msg_kline(j2534_conn *conn, uint8_t data)
{
  (void)conn;
  (void)data;
  return STATUS_NOERROR;
}

uint32_t stop_periodic_msg_kline(j2534_conn *conn, uint32_t msg)
{
  (void)conn;
  (void)msg;
  return STATUS_NOERROR;
}

uint32_t ioctl_clear_filters_kline(j2534_conn *conn)
{
  (void)conn;
  return STATUS_NOERROR;
}

uint32_t ioctl_datarate_kline(j2534_conn *conn)
{
  sdStop(&SD2);
  uart2Cfg.speed = conn->DataRate;
  sdStart(&SD2, &uart2Cfg);
  return STATUS_NOERROR;
}

uint32_t ioctl_loopback_kline(j2534_conn *conn)
{
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
 *                  B3
 * ECU: 80  F1  12  03 (C1  EA  8F) C0
 *       |   |   |   |  ---DATA---   |____CRC
 *       |   |   |   |_______LEN* Data length
 *       |   |   |________SRC* Tester address
 *       |   |___TGT* Destination ECU address
 *       |_FMT [8:7]CFG [0:6]LEN IF 0: LEN (B3)
 *
 *  * Optional, depends from FMT
 *  CFG 0 0 no address information
 *      0 1 Exception mode (CARB)
 *      1 0 with address information, physical addressing
 *      1 1 with address information, functional addressing
 *
 * Saab 9-5 -> T7 (tech2win) -> J2534 MSG
 * REQ: 81  41  f1  (81)
 * ECU: 83  f1  41  (c1  6b  8f)
 */
uint32_t ioctl_fast_init_kline(j2534_conn *conn, uint8_t *in, uint8_t *response)
{
  j2534_protocol_cfg *pcfg = conn->pcfg;
  if (kwp_session.session_active)
  {
    terminate_session(conn);
    chThdSleepMilliseconds(pcfg->P3Max / 2);
  }

  // in b0 len, [b1:bx] data
  uint8_t request[5] = {0};
  uint8_t bytesToSend = in[0];

  if (bytesToSend < 4)
  {
    return ERR_INVALID_IOCTL_VALUE;
  }

  if ((bytesToSend < 5) && (conn->flags & ISO9141_NO_CHECKSUM))
    return ERR_INVALID_IOCTL_VALUE;

  memcpy(request, in + 1, bytesToSend);

  if (!(conn->flags & ISO9141_NO_CHECKSUM))
  {
    request[4] = calcChecksum(request, 4);
    bytesToSend++;
  }

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

  for (uint8_t i = 0; i < bytesToSend; i++)
  {
    sdPut(&SD2, request[i]);
    if (pcfg->P4Min > 0)
      chThdSleepMilliseconds(pcfg->P4Min / 2);
  }
  // Clear serial buffer, echo bytes, not valid data.
  chSysLock();
  iqResetI(&SD2.iqueue);
  chSysUnlock();

  chThdSleepMilliseconds(pcfg->Tinil);
  // Read header with content length.
  response[0] = sdReadTimeout(&SD2, response + 1, 3, TIME_MS2I(3 * pcfg->P1Max / 2));
  if (response[0] < 3)
  {
    return ERR_FAILED;
  }
  uint8_t remainLen = (response[1] & LENGTH_MASK) + 1;
  if (remainLen > 0)
  {
    response[0] += sdReadTimeout(&SD2, response + 4, remainLen, TIME_MS2I(pcfg->P1Max / 2 * remainLen));
  }

  if (!(conn->flags & ISO9141_NO_CHECKSUM))
  {
    response[0]--;
  }

  if (!(request[3] + 0x40 == response[4]))
  {
    return ERR_INVALID_IOCTL_VALUE; // ???
  }

  uint8_t kb1 = response[5];
  uint8_t kb2 = response[6];
  if (kb2 != 0x8F)
  {
    DBG_PRNT("kb2: %d != 0x8F");
    return ERR_INVALID_IOCTL_VALUE; // ???
  }
  // iso14230 5.2.4.1 & Table 8
  kwp_session.modeflags |= ((kb1 & 1) ? ISO14230_FMTLEN : 0) |
                           ((kb1 & 2) ? ISO14230_LENBYTE : 0) |
                           ((kb1 & 4) ? ISO14230_SHORTHDR : 0) |
                           ((kb1 & 8) ? ISO14230_LONGHDR : 0);
  DBG_PRNT("modeflags: ISO14230_FMTLEN %d, ISO14230_LENBYTE %d, ISO14230_SHORTHDR %d, ISO14230_LONGHDR %d \r\n",
           kwp_session.modeflags & ISO14230_FMTLEN, kwp_session.modeflags & ISO14230_LENBYTE, kwp_session.modeflags & ISO14230_SHORTHDR, kwp_session.modeflags & ISO14230_LONGHDR);

  kwp_session.session_active = true;
  kwp_session.tgt = request[1];
  kwp_session.src = request[2];
  build_kwp_msgs();

  if (!testertp)
  {
    DBG_PRNT("Create testertp\r\n");
    testertp = chThdCreateFromHeap(NULL, TESTER_WA_SIZE, "testerThd", LOWPRIO, testerThd, conn);
  }

  if (!klinetp)
  {
    DBG_PRNT("Create klinetp\r\n");
    klinetp = chThdCreateFromHeap(NULL, KLINE_WA_SIZE, "klineThd", NORMALPRIO + 1, klineThd, conn);
  }
  chThdSleepMilliseconds(pcfg->P3Min / 2);
  return STATUS_NOERROR;
}

uint32_t ioctl_five_baud_init_kline(j2534_conn *conn, uint8_t *in, uint8_t *out)
{
  (void)conn;
  (void)in;
  (void)out;
  return STATUS_NOERROR;
}

static THD_FUNCTION(testerThd, arg)
{
  chRegSetThreadName("KWP_TesterPresent");
  j2534_conn *conn = (j2534_conn *)arg;
  systime_t next_deadline = chVTGetSystemTime();
  uint8_t resp[32] = {0};
  uint8_t resplen = 0;

  uint8_t buff[7] = {0};
  msgToBuffer(&kwp_alive, buff);

  while (!chThdShouldTerminateX())
  {
    next_deadline += TIME_MS2I(TESTER_PRESENT_PERIOD_MS);
    chThdSleepUntil(next_deadline);
    if (kwp_session.session_active & kwp_session.keep_alive)
    {
      bool status = sendRecv(conn, buff, 6, resp, &resplen, true);
      DBG_PRNT("kwp alive %d \r\n", status);
    }
  }
}

THD_FUNCTION(klineThd, arg)
{
  chRegSetThreadName("kline comm");
  j2534_conn *conn = (j2534_conn *)arg;
  j2534_protocol_cfg *pcfg = conn->pcfg;

  while (!chThdShouldTerminateX())
  {
    chThdSleepMilliseconds(pcfg->P3Min);
  }
}