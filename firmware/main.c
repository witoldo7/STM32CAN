// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski
 */ 

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "debug.h"
#include "usbadapter.h"
#include "canutils.h"

BaseSequentialStream *GlobalDebugChannel;

#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define USB_WA_SIZE   THD_WORKING_AREA_SIZE(8048)
#define CMD_WA_SIZE   THD_WORKING_AREA_SIZE(8048)
#define HSCAN_WA_SIZE   THD_WORKING_AREA_SIZE(2048)
#define SWCAN_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

thread_t *hscantp = NULL;
thread_t *swcantp = NULL;
thread_t *shelltp = NULL;
thread_t *usbtp = NULL;
thread_t *cmdtp = NULL;

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

  sdStart(&SD1, &sercfg);
  GlobalDebugChannel = (BaseSequentialStream *)&SDU1;

  adcStart(&ADCD1, NULL);

  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg1);

  usbDisconnectBus(&USBD1);
  usbStart(&USBD1, &usb_config);

  usbConnectBus(&USBD1);
  chEvtRegister(&shell_terminated, &el0, 0);

  /*
   * Shell manager initialization.
   * Event zero is shell exit.
   */
  shellInit();

  /*
   * Normal main() thread activity, handling SD card events and shell
   * start/exit.
   */
  while (TRUE) {
    if (!shelltp && SDU1.config->usbp->state == USB_ACTIVE) {
      shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                       "shell1", NORMALPRIO + 1,
                                       shellThread, (void *)&shell_cfg1);
    }

    if (!usbtp && SDU1.config->usbp->state == USB_ACTIVE) {
      usbtp = chThdCreateFromHeap(NULL, USB_WA_SIZE, "usbThd", NORMALPRIO + 2, usbThd_rx, NULL);
    }

    if (!cmdtp && SDU1.config->usbp->state == USB_ACTIVE) {
      cmdtp = chThdCreateFromHeap(NULL, CMD_WA_SIZE, "cmdThd", NORMALPRIO + 1, cmdThd, NULL);
    }

    if (!swcantp) {
      swcantp = chThdCreateFromHeap(NULL, SWCAN_WA_SIZE, "swcan receiver", NORMALPRIO + 3, swcan_rx, NULL);
    }

    if (!hscantp) {
      hscantp = chThdCreateFromHeap(NULL, HSCAN_WA_SIZE, "hscan receiver", NORMALPRIO + 4, hscan_rx, NULL);
    }

    chEvtDispatch(evhndl, chEvtWaitOneTimeout(EVENT_MASK(0), TIME_MS2I(500)));
  }
}
