// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski
 */ 

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <usbadapter.h>

#include "ch.h"
#include "hal.h"
#include "combi.h"
#include "shell.h"
#include "chprintf.h"
#include "debug.h"
#include "j2534.h"
#include "socketcan.h"
#include "bdmutility.h"

BaseSequentialStream *GlobalDebugChannel;
static semaphore_t rxSem;
static semaphore_t processSem;

uint8_t receiveBuf[OUT_PACKETSIZE*2];
uint8_t transferBuf[IN_PACKETSIZE*2];

static uint8_t rxdata[300], txdata[300];
static packet_t rx_packet = {.data = rxdata};
static packet_t tx_packet = {.data = txdata};

bool (*hscan_rx_cb)(CANRxFrame*, packet_t*);
bool (*swcan_rx_cb)(CANRxFrame*, packet_t*);

void registerHsCanCallback(bool (*cb)(CANRxFrame *rxmsg, packet_t *packet)) {
  hscan_rx_cb = cb;
}

void registerSwCanCallback(bool (*cb)(CANRxFrame *rxmsg, packet_t *packet)) {
  swcan_rx_cb = cb;
}

/*
 * data Received Callback
 */
void dataReceived(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  chSysLockFromISR();
  chSemSignalI(&rxSem);        //memset(receiveBuf, 0, OUT_PACKETSIZE*2);

  chSysUnlockFromISR();
}

bool combiConfigureHookI(USBDriver *usbp) {
  if (usbGetDriverStateI(usbp) != USB_ACTIVE) {
    return true;
  }

  if (usbGetReceiveStatusI(usbp, EP_OUT)) {
    return true;
  }

  usbStartReceiveI(usbp, EP_OUT, receiveBuf, IN_PACKETSIZE);

  return false;
}

static THD_WORKING_AREA(usb_rx_wa, 8196);
static THD_FUNCTION(usb_rx, arg) {
  (void)arg;
  chRegSetThreadName("usbReceiver");
  bool is_completed = true;
  uint16_t cur_pos = 0;
  uint16_t len = 0;
  while (TRUE) {
    chSemWait(&rxSem);
    while (!(((USBDriver*)&USBD1)->state == USB_ACTIVE)) {
      __asm("nop");
    }
    if (*(receiveBuf) == 0) {
      continue;
    }
    uint8_t rec_size = ((USBDriver*)&USBD1)->epc[EP_OUT]->out_state->rxcnt;
    if (is_completed) {
      rx_packet.cmd_code = receiveBuf[0];
      len = (uint16_t)((receiveBuf[1] & 0xffffU) << 8) | (uint16_t)receiveBuf[2];
      if (len > 255) {
        continue;
      }
      rx_packet.data_len = len;
      memcpy(rx_packet.data, receiveBuf + 3, rec_size);
      cur_pos += rec_size;
      if ((len + 3) > cur_pos) {
        is_completed = false;
      } else {
        rx_packet.term = receiveBuf[rec_size-1];
        osalSysLock();
        chSemSignalI(&processSem);
        osalSysUnlock();
        cur_pos = 0;
        len = 0;
        is_completed = true;
      }
    } else {
      if ((cur_pos + rec_size) >= len + 3) {
        memcpy(rx_packet.data + cur_pos, receiveBuf, rec_size - 1);
        rx_packet.term = receiveBuf[rec_size-1];
        osalSysLock();
        chSemSignalI(&processSem);
        osalSysUnlock();
        cur_pos = 0;
        len = 0;
        is_completed = true;
      } else {
        memcpy(rx_packet.data + cur_pos, receiveBuf, rec_size);
        cur_pos += rec_size;
      }
    }
    start_receive(&USBD1, EP_OUT, receiveBuf, IN_PACKETSIZE*2);
  }
}

static THD_WORKING_AREA(combi_wa, 8196);
static THD_FUNCTION(combi, arg) {
  (void)arg;
  bool ret = false;
  uint8_t size = 0;
  chRegSetThreadName("combi");
  while (TRUE) {
    chSemWait(&processSem);
    if (rx_packet.term == cmd_term_ack) {
      switch (rx_packet.cmd_code & 0xe0) {
      case 0x20: //Board utility.
        ret = exec_cmd_board(&rx_packet, &tx_packet);
        break;
      case 0x40: //BDM utility.
        ret = exec_cmd_bdm(&rx_packet, &tx_packet);
        break;
      case 0x60: //SocketCAN utility.
        ret = exec_cmd_socketcan(&rx_packet, &tx_packet);
        break;
      case 0x80:  //CAN utility.
        ret = exec_cmd_can(&rx_packet, &tx_packet);
        break;
      case 0xA0:  //J2534 utility.
        ret = exec_cmd_j2534(&rx_packet, &tx_packet);
        break;
        break;
      default:
        continue;
      }
      if (ret != true) {
        prepareReplyPacket(&tx_packet, &rx_packet, 0, 0, cmd_term_nack);
      }

      size = covertPacketToBuffer(&tx_packet, transferBuf);
      usb_send(&USBD1, EP_IN, transferBuf, size);
    }
  }
}

static THD_WORKING_AREA(can_rx_wa, 4096);
static THD_FUNCTION(can_rx, p) {
  (void)p;
  //event_listener_t el;
  static CANRxFrame rxmsg = {};
  static uint8_t size = 0;
  static uint8_t buffer[80] = {0};
  static uint8_t canbuff[66] = {0};
  packet_t tx_packet = {.data = canbuff};
  chRegSetThreadName("can receiver");
  //chEvtRegister(&CAND1.rxfull_event, &el, 0);

  while (true) {
    // TODO: check from LLD side, some frame are dropped
   // if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
     // continue;
    //}
    while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK ) {
      if (hscan_rx_cb == NULL)
        continue;
      if (!hscan_rx_cb(&rxmsg, &tx_packet))
        continue;
      size = covertPacketToBuffer(&tx_packet, buffer);
      while(!(((USBDriver*)&USBD1)->state == USB_ACTIVE)) {
        __asm("nop");
      }
      usb_send(&USBD1, EP_IN, buffer, size);
    }
  }
 // chEvtUnregister(&CAND1.rxfull_event, &el);
}

static THD_WORKING_AREA(swcan_rx_wa, 4096);
static THD_FUNCTION(swcan_rx, p) {
  (void)p;
  event_listener_t el;
  CANRxFrame rxmsg = {};
  uint8_t size = 0;
  uint8_t buffer[IN_PACKETSIZE] = {0};
  uint8_t packetbuff[16] = {0};
  packet_t tx_packet = {.data = packetbuff};
  chRegSetThreadName("swcan receiver");
  chEvtRegister(&CAND2.rxfull_event, &el, 0);

  while (true) {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
      continue;
    }
    while (canReceive(&CAND2, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK ) {
      if (hscan_rx_cb == NULL)
        continue;
      if (!hscan_rx_cb(&rxmsg, &tx_packet))
        continue;
      size = covertPacketToBuffer(&tx_packet, buffer);
      usb_send(&USBD1, EP_IN, buffer, size);
    }
  }
  chEvtUnregister(&CAND2.rxfull_event, &el);
}

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static const ShellCommand commands[] = {
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};

static const SerialConfig sercfg = {
    115200,
    0,
    0,
    0
};

thread_t *shelltp = NULL;
/*
 * Shell exit event.
 */
static void ShellHandler(eventid_t id) {
  (void)id;
  if (chThdTerminatedX(shelltp)) {
    chThdRelease(shelltp);
    shelltp = NULL;
  }
}

int main(void) {
  static const evhandler_t evhndl[] = {
    ShellHandler
  };

  event_listener_t el0;

  halInit();
  chSysInit();
  chSemObjectInit(&rxSem, 1);
  chSemObjectInit(&processSem, 1);

  sdStart(&SD1, &sercfg);
  GlobalDebugChannel = (BaseSequentialStream *)&SD1;

  adcStart(&ADCD1, NULL);

  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg1);

  usbDisconnectBus(&USBD1);
  usbStart(&USBD1, &usb_config);

  /* USB mass storage configuration */
  usbConnectBus(&USBD1);
  chEvtRegister(&shell_terminated, &el0, 0);

  /*
   * Shell manager initialization.
   * Event zero is shell exit.
   */
  shellInit();

  chThdCreateStatic(usb_rx_wa, sizeof(usb_rx_wa), NORMALPRIO + 20, usb_rx, NULL);
  chThdCreateStatic(combi_wa, sizeof(combi_wa), NORMALPRIO + 7, combi, NULL);
  chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 2, can_rx, NULL);
  chThdCreateStatic(swcan_rx_wa, sizeof(swcan_rx_wa), NORMALPRIO + 2, swcan_rx, NULL);

  /*
   * Normal main() thread activity, handling SD card events and shell
   * start/exit.
   */
  while (TRUE) {
    if (!shelltp && SDU1.config->usbp->state == USB_ACTIVE) {
      /* Starting shells.*/
      shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                       "shell1", NORMALPRIO + 1,
                                       shellThread, (void *)&shell_cfg1);
      chThdSleepMilliseconds(500);

    }
    chEvtDispatch(evhndl, chEvtWaitOneTimeout(EVENT_MASK(0), TIME_MS2I(500)));
  }
}
