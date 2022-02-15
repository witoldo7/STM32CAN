#include "usbcombi.h"

#include "hal.h"

bool isReady = false;

static USBInEndpointState ep1instate;
volatile bool is_transmiting = false;
void dataTransmitted(USBDriver *usbp, usbep_t ep);

static const USBEndpointConfig ep1config = {
USB_EP_MODE_TYPE_INTR,
                                            NULL,
                                            NULL,
                                            NULL,
                                            INT_PACKETSIZE,
                                            0x0000, &ep1instate,
                                            NULL,
                                            1,
                                            NULL};

static USBInEndpointState ep2instate;
static USBOutEndpointState ep2outstate;

static const USBEndpointConfig ep2config = {
USB_EP_MODE_TYPE_BULK,    //Type and mode of the endpoint
    NULL,        //Setup packet notification callback (NULL for non-control EPs)
    dataTransmitted,          //IN endpoint notification callback
    dataReceived,             //OUT endpoint notification callback
    IN_PACKETSIZE,            //IN endpoint maximum packet size
    OUT_PACKETSIZE,           //OUT endpoint maximum packet size
    &ep2instate,              //USBEndpointState associated to the IN endpoint
    &ep2outstate,              //USBEndpointState associated to the OUTendpoint
    1,
    NULL      //Pointer to a buffer for setup packets (NULL for non-control EPs)
    };

static void usb_event(USBDriver *usbp, usbevent_t event) {
  (void)usbp;
  switch (event) {
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();
    if (usbp->state == USB_ACTIVE) {
      usbInitEndpointI(usbp, EP_INT, &ep1config);
      usbInitEndpointI(usbp, EP_IN, &ep2config);
      isReady = true;
    }
    else if (usbp->state == USB_SELECTED) {
      usbDisableEndpointsI(usbp);
      isReady = false;
    }
    chSysUnlockFromISR();
    return;
  case USB_EVENT_RESET:
  case USB_EVENT_UNCONFIGURED:
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();
    isReady = false;
    chSysUnlockFromISR();
    return;
  case USB_EVENT_WAKEUP:
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

static const USBDescriptor* get_descriptor(USBDriver *usbp, uint8_t dtype,
                                           uint8_t dindex, uint16_t lang) {
  (void)usbp;
  (void)lang;

  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4) {
      return &vcom_strings[dindex];
    }
    else {
      return &vcom_strings[4];
    }
  }
  return NULL;
}

static bool requests_hook(USBDriver *usbp) {

  if (((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK)
      == USB_RTYPE_RECIPIENT_INTERFACE)
      && (usbp->setup[1] == USB_REQ_SET_INTERFACE)) {
    usbSetupTransfer(usbp, NULL, 0, NULL);
    return true;
  }
  return false;
}

const USBConfig usb_config = {usb_event, get_descriptor, requests_hook,
NULL};

/*
 * data Transmitted Callback
 */
void dataTransmitted(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  is_transmiting = false;
}

void usb_send(USBDriver *usbp, usbep_t ep, const uint8_t *buf, size_t n) {
  while (is_transmiting) {
    chThdSleepMicroseconds(1);
  }
  is_transmiting = true;
  osalSysLock();
  usbStartTransmitI(usbp, ep, buf, n);
  osalSysUnlock();
}

bool start_receive(USBDriver *usbp, usbep_t ep, uint8_t *buf, size_t n) {
  if (usbGetDriverStateI(usbp) != USB_ACTIVE) {
    return true;
  }

  if (usbGetReceiveStatusI(usbp, ep)) {
    return true;
  }

  chSysLock();
  usbStartReceiveI(usbp, ep, buf, n);
  chSysUnlock();

  return false;
}

bool is_ready(void) {
  return isReady;
}

