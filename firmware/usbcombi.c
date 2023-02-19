// SPDX-License-Identifier: Apache-2.0

/* 
 * STM32CAN Firmware.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#include "hal.h"
#include "usbcombi.h"

#define CHUNK_INTERVAL_FS 0x00 /* 0x19 - 25ms for packet of 10 data reads (USB 2.0 FS) */
#define INT_PACKETSIZE  0x10
#define CDC_PACKETSIZE  0x40
#define EP_INT_CDC 1
#define EP_CDC 3

/*
 * Virtual serial ports over USB.
 */
SerialUSBDriver SDU1;

volatile bool is_transmiting = false;
void dataTransmitted(USBDriver *usbp, usbep_t ep);

static USBInEndpointState ep1instate;

static USBInEndpointState ep2instate;
static USBOutEndpointState ep2outstate;
static USBInEndpointState ep3instate;
static USBOutEndpointState ep3outstate;

static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  INT_PACKETSIZE,
  0,
  &ep1instate,
  NULL,
  1,
  NULL
};

static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_BULK,    //Type and mode of the endpoint
  NULL,                     //Setup packeUSB_EP_MODE_TYPE_BULKt notification callback (NULL for non-control EPs)
  dataTransmitted,          //IN endpoint notification callback
  dataReceived,             //OUT endpoint notification callback
  IN_PACKETSIZE,            //IN endpoint maximum packet size
  OUT_PACKETSIZE,           //OUT endpoint maximum packet size
  &ep2instate,              //USBEndpointState associated to the IN endpoint
  &ep2outstate,             //USBEndpointState associated to the OUTendpoint
  1,
  NULL      //Pointer to a buffer for setup packets (NULL for non-control EPs)
};

static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_BULK,    //Type and mode of the endpoint
  NULL,        //Setup packet notification callback (NULL for non-control EPs)
  sduDataTransmitted,          //IN endpoint notification callback
  sduDataReceived,             //OUT endpoint notification callback
  IN_PACKETSIZE,            //IN endpoint maximum packet size
  OUT_PACKETSIZE,           //OUT endpoint maximum packet size
  &ep3instate,              //USBEndpointState associated to the IN endpoint
  &ep3outstate,              //USBEndpointState associated to the OUTendpoint
  2,
  NULL      //Pointer to a buffer for setup packets (NULL for non-control EPs)
};

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE(0x0200, /* bcdUSB USB version */
  0xEF, /* bDeviceClass (FF=Vendor specific */
  0x02, /* bDeviceSubClass.                 */
  0x01, /* bDeviceProtocol.                 */
  0x40, /* bMaxPacketSize0.                 */
  0xffff, /* idVendor (ST).                   */
  0x0005, /* idProduct.                       */
  0x0200, /* bcdDevice Device Release Number  */
  1, /* Index of Manufacturer String Descriptor                  */
  2, /* iProduct.                        */
  3, /* iSerialNumber.                   */
  1) /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
    sizeof vcom_device_descriptor_data, vcom_device_descriptor_data};

/*
 * Configuration Descriptor
 */
static const uint8_t vcom_configuration_descriptor_data[ 9 + 9 + 5 + 5 + 4 + 5 + 7 + 9 + 7 + 7 + 9 + 7 + 7 ] = {
    /* Configuration Descriptor.*/
    USB_DESC_CONFIGURATION(sizeof vcom_configuration_descriptor_data, /* wTotalLength.  */
                           0x03, /* bNumInterfaces.                                     */
                           0x01, /* bConfigurationValue.                                */
                           0x00, /* iConfiguration.                                     */
                           0x80, /* bmAttributes (bus powered).                         */
                           200),  /* bMaxPower (100mA).                                  */
   /* CDC Int Interface Descriptor.*/
    USB_DESC_INTERFACE(0x00, /* bInterfaceNumber.                                                  */
                       0x00, /* bAlternateSetting.                                                 */
                       0x01, /* bNumEndpoints.                                                     */
                       CDC_COMMUNICATION_INTERFACE_CLASS, /* bInterfaceClass (Communications Interface Class, CDC section 4.2). */
                       CDC_ABSTRACT_CONTROL_MODEL, /* bInterfaceSubClass (Abstract Control Model, CDC section 4.3).      */
                       0x01, /* bInterfaceProtocol (AT commands, CDC section 4.4).                 */
                       0x04),   /* iInterface.                                                        */
    /* Header Functional Descriptor (CDC section 5.2.3).*/
    USB_DESC_BYTE(5),     /* bLength.                                                      */
    USB_DESC_BYTE(CDC_CS_INTERFACE),  /* bDescriptorType (CS_INTERFACE).                               */
    USB_DESC_BYTE(CDC_HEADER),  /* bDescriptorSubtype (Header Functional Descriptor.             */
    USB_DESC_BCD(0x0110), /* bcdCDC.                                                       */
    /* Call Management Functional Descriptor. */
    USB_DESC_BYTE(5),     /* bFunctionLength.                                              */
    USB_DESC_BYTE(CDC_CS_INTERFACE),  /* bDescriptorType (CS_INTERFACE).                               */
    USB_DESC_BYTE(CDC_CALL_MANAGEMENT),  /* bDescriptorSubtype (Call Management Functional Descriptor).   */
    USB_DESC_BYTE(0x00),  /* bmCapabilities (D0+D1).                                       */
    USB_DESC_BYTE(0x02),  /* bDataInterface.                                               */
    /* ACM Functional Descriptor.*/
    USB_DESC_BYTE(4),     /* bFunctionLength.                                              */
    USB_DESC_BYTE(CDC_CS_INTERFACE),  /* bDescriptorType (CS_INTERFACE).                               */
    USB_DESC_BYTE(CDC_ABSTRACT_CONTROL_MANAGEMENT),  /* bDescriptorSubtype (Abstract Control Management Descriptor).  */
    USB_DESC_BYTE(0x02),  /* bmCapabilities.                                               */
    /* Union Functional Descriptor.*/
    USB_DESC_BYTE(5),     /* bFunctionLength.                                              */
    USB_DESC_BYTE(CDC_CS_INTERFACE),  /* bDescriptorType (CS_INTERFACE).                               */
    USB_DESC_BYTE(CDC_UNION),  /* bDescriptorSubtype (Union Functional Descriptor).             */
    USB_DESC_BYTE(0x00),  /* bMasterInterface (Communication Class Interface).             */
    USB_DESC_BYTE(0x02),  /* bSlaveInterface0 (Data Class Interface).                      */
    /* Endpoint 1 Descriptor.*/
    USB_DESC_ENDPOINT(EP_INT_CDC | USB_RTYPE_DIR_DEV2HOST, /* bEndpointAddress */
                      USB_EP_MODE_TYPE_INTR,               /* bmAttributes (Interrupt).        */
                      INT_PACKETSIZE,                      /* wMaxPacketSize.                  */
                      0x01),                               /* bInterval.                       */
    /* Combi Data Interface Descriptor.*/
    USB_DESC_INTERFACE(0x01,                            /* bInterfaceNumber.                */
                       0x00,                            /* bAlternateSetting.               */
                       0x02,                            /* bNumEndpoints.                   */
                       0xFF,                            /* bInterfaceClass (Data Class Interface, CDC section 4.5).     */
                       0x00,                            /* bInterfaceSubClass (CDC section 4.6).                            */
                       0x00,                            /* bInterfaceProtocol (CDC section  4.7).                            */
                       0x05),                           /* iInterface.                      */
    /* Endpoint 2 Descriptor, Direction out*/
    USB_DESC_ENDPOINT(EP_OUT | USB_RTYPE_DIR_HOST2DEV,  /* bEndpointAddress     */
        USB_EP_MODE_TYPE_BULK,                          /* bmAttributes (Bulk)  */
        OUT_PACKETSIZE,                                 /* wMaxPacketSize       */
        CHUNK_INTERVAL_FS),                             /* bInterval            */
    /* Endpoint 2 Descriptor, Direction in*/
    USB_DESC_ENDPOINT(EP_IN | USB_RTYPE_DIR_DEV2HOST,   /* bEndpointAddress     */
        USB_EP_MODE_TYPE_BULK,                          /* bmAttributes (Bulk)  */
        IN_PACKETSIZE,                                  /* wMaxPacketSize       */
        CHUNK_INTERVAL_FS),                             /* bInterval            */
    /* CDC Data Interface Descriptor. */
    USB_DESC_INTERFACE(0x02,                            /* bInterfaceNumber.                                         */
                       0x00,                            /* bAlternateSetting.                                        */
                       0x02,                            /* bNumEndpoints.                                            */
                       CDC_DATA_INTERFACE_CLASS,        /* bInterfaceClass (Data Class Interface, CDC section 4.5).  */
                       0x00,                            /* bInterfaceSubClass (CDC section 4.6).                     */
                       0x00,                            /* bInterfaceProtocol (CDC section  4.7).                    */
                       0x04),                           /* iInterface.                                               */
    /* Endpoint 3 Descriptor, Direction out*/
    USB_DESC_ENDPOINT(EP_CDC | USB_RTYPE_DIR_HOST2DEV,   /* bEndpointAddress     */
        USB_EP_MODE_TYPE_BULK,                           /* bmAttributes (Bulk)  */
        CDC_PACKETSIZE,                                  /* wMaxPacketSize       */
        CHUNK_INTERVAL_FS),                              /* bInterval            */
    /* Endpoint 3 Descriptor, Direction in*/
    USB_DESC_ENDPOINT(EP_CDC | USB_RTYPE_DIR_DEV2HOST,   /* bEndpointAddress     */
        USB_EP_MODE_TYPE_BULK,                           /* bmAttributes (Bulk)  */
        CDC_PACKETSIZE,                                  /* wMaxPacketSize       */
        CHUNK_INTERVAL_FS),                              /* bInterval            */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
    sizeof vcom_configuration_descriptor_data,
    vcom_configuration_descriptor_data};

/*
 * U.S. English language identifier.
 */
static const uint8_t stringLanguage[] = {USB_DESC_BYTE(4),                      /* bLength.                         */
                                         USB_DESC_BYTE(USB_DESCRIPTOR_STRING),  /* bDescriptorType.                 */
                                         USB_DESC_WORD(0x0409)                  /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t stringVendor[] = {USB_DESC_BYTE(0x0C),                     /* bLength.                         */
                                       USB_DESC_BYTE(USB_DESCRIPTOR_STRING),    /* bDescriptorType.                 */
                                       'W', 0, 'Q', 0, 'C', 0, 'A', 0, 'N', 0,};

/*
 * Device Description string.
 */
static const uint8_t stringDeviceDescriptor[] = {USB_DESC_BYTE(0x18), /* bLength.                         */
                                                 USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
                                                 'C', 0, 'a', 0, 'n', 0, ' ', 0,
                                                 'A', 0, 'd', 0, 'a', 0, 'p', 0,
                                                 't', 0, 'e', 0, 'r', 0, };

/*
 * Serial Number string.
 */
static const uint8_t stringSerialNumber[] = {USB_DESC_BYTE(0x0A), /* bLength.                         */
                                             USB_DESC_BYTE(
                                             USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
                                             'B', 0, 'E', 0, 'E', 0, 'F', 0, };

/*
 * String not found string.
 */
static const uint8_t stringNotFound[] = {USB_DESC_BYTE(34), /* bLength .                        */
                                         USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
                                         'S', 0, 't', 0, 'r', 0, 'i', 0, 'n', 0,
                                         'g', 0, ' ', 0, 'n', 0, 'o', 0, 't', 0,
                                         ' ', 0, 'f', 0, 'o', 0, 'u', 0, 'n', 0,
                                         'd', 0};

/*
 * CAN string.
 */
static const uint8_t stringCAN[] = {USB_DESC_BYTE(0x18), /* bLength.                         */
                                             USB_DESC_BYTE(
                                             USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
                                             'C', 0, 'A', 0, 'N', 0, ' ', 0, 'A', 0,
                                             'D', 0, 'A', 0, 'P', 0, 'T', 0, 'E', 0,
                                             'R', 0,};

/*
 * CDC string.
 */
static const uint8_t stringCDC[] = {USB_DESC_BYTE(0x16), /* bLength.                         */
                                             USB_DESC_BYTE(
                                             USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
                                             'C', 0, 'D', 0, 'C', 0, ' ', 0, 'S', 0,
                                             'E', 0, 'R', 0, 'I', 0, 'A', 0, 'L', 0,};

/*
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
    {sizeof stringLanguage, stringLanguage},
    {sizeof stringVendor, stringVendor},
    {sizeof stringDeviceDescriptor, stringDeviceDescriptor},
    {sizeof stringSerialNumber, stringSerialNumber},
    {sizeof stringCDC, stringCDC},
    {sizeof stringCAN, stringCAN},
    {sizeof stringNotFound, stringNotFound}};


static void usb_event(USBDriver *usbp, usbevent_t event) {
  extern SerialUSBDriver SDU1;
  switch (event) {
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();
    if (usbp->state == USB_ACTIVE) {
      usbInitEndpointI(usbp, EP_INT_CDC, &ep1config);
      usbInitEndpointI(usbp, EP_IN, &ep2config);
      usbInitEndpointI(usbp, EP_CDC, &ep3config);
      /* Resetting the state of the CDC subsystem.*/
      sduConfigureHookI(&SDU1);
      combiConfigureHookI(usbp);
    } else if (usbp->state == USB_SELECTED) {
      usbDisableEndpointsI(usbp);
    }
    chSysUnlockFromISR();
    return;
  case USB_EVENT_RESET:
  case USB_EVENT_UNCONFIGURED:
  case USB_EVENT_SUSPEND:
    chSysLockFromISR();
    /* Disconnection event on suspend.*/
    sduSuspendHookI(&SDU1);
    chSysUnlockFromISR();
    return;
  case USB_EVENT_WAKEUP:
    chSysLockFromISR();
    /* Connection event on wakeup.*/
    sduWakeupHookI(&SDU1);
    chSysUnlockFromISR();
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
    if (dindex < 6) {
      return &vcom_strings[dindex];
    }
    else {
      return &vcom_strings[5];
    }
  }
  return NULL;
}

static bool requests_hook(USBDriver *usbp) {
  return sduRequestsHook(usbp);
}

/*
 * Handles the USB driver global events.
 */
static void sof_handler(USBDriver *usbp) {

  (void)usbp;

  osalSysLockFromISR();
  sduSOFHookI(&SDU1);
  osalSysUnlockFromISR();
}

const USBConfig usb_config = {usb_event, get_descriptor, requests_hook, sof_handler};

/*
 * data Transmitted Callback
 */
void dataTransmitted(USBDriver *usbp, usbep_t ep) {
  (void)usbp;
  (void)ep;
  osalSysLockFromISR();
  is_transmiting = false;
  osalSysUnlockFromISR();
}

void usb_send(USBDriver *usbp, usbep_t ep, const uint8_t *buf, size_t n) {
  while (is_transmiting) {
    chThdSleepMicroseconds(1);
  }
  osalSysLock();
  is_transmiting = true;
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

/*
 * Serial over USB driver configuration 1.
 */
const SerialUSBConfig serusbcfg1 = {
  &USBD1,
  EP_CDC,
  EP_CDC,
  EP_INT_CDC
};
