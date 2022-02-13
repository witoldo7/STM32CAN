#ifndef USBCFG_H_INCLUDED
#define USBCFG_H_INCLUDED

#include "hal.h"

#define CHUNK_INTERVAL_FS 0x00 /* 0x19 - 25ms for packet of 10 data reads (USB 2.0 FS) */
#define INT_PACKETSIZE  0x08
#define IN_PACKETSIZE  0x20
#define OUT_PACKETSIZE 0x20
#define EP_INT 1
#define EP_IN 2
#define EP_OUT 2

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {USB_DESC_DEVICE(0x0200, /* bcdUSB USB version:
 0x0100 (USB1.0)
 0x0110 (USB1.1)
 0x0200 (USB2.0)
 */
                                                                        0xFF, /* bDeviceClass (FF=Vendor specific */
                                                                        0x00, /* bDeviceSubClass.                 */
                                                                        0x00, /* bDeviceProtocol.                 */
                                                                        0x40, /* bMaxPacketSize0.                 */
                                                                        0xffff, /* idVendor (ST).                   */
                                                                        0x0005, /* idProduct.                       */
                                                                        0x0100, /* bcdDevice Device Release Number  */
                                                                        1, /* Index of Manufacturer String Descriptor                  */
                                                                        2, /* iProduct.                        */
                                                                        3, /* iSerialNumber.                   */
                                                                        1)
/* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
    sizeof vcom_device_descriptor_data, vcom_device_descriptor_data};

/*
 * Configuration Descriptor
 */
static const uint8_t vcom_configuration_descriptor_data[9 + 9 + 5 + 5 + 4 + 5
    + 7 + 9 + 7 + 7] = {
/* Configuration Descriptor.*/
//9 Bytes
    USB_DESC_CONFIGURATION(sizeof vcom_configuration_descriptor_data, /* wTotalLength.                    */
                           0x02, /* bNumInterfaces.                  */
                           0x01, /* bConfigurationValue.             */
                           0, /* iConfiguration.                  */
                           0xC0, /* bmAttributes (self powered).     */
                           50), /* bMaxPower (100mA).               */
    /* Interface Descriptor.*/
    //9 Bytes
    /* Interface Descriptor.*/
    USB_DESC_INTERFACE(0x00, /* bInterfaceNumber.                */
                       0x00, /* bAlternateSetting.               */
                       0x01, /* bNumEndpoints.                   */
                       0x02, /* bInterfaceClass (Communications
                        Interface Class, CDC section
                        4.2).                            */
                       0x02, /* bInterfaceSubClass (Abstract
                        Control Model, CDC section 4.3).   */
                       0x01, /* bInterfaceProtocol (AT commands,
                        CDC section 4.4).                */
                       0), /* iInterface.                      */
    /* Header Functional Descriptor (CDC section 5.2.3).*/
    USB_DESC_BYTE(5), /* bLength.                         */
    USB_DESC_BYTE(0x24), /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE(0x00), /* bDescriptorSubtype (Header
     Functional Descriptor.           */
    USB_DESC_BCD(0x0110), /* bcdCDC.                          */
    /* Call Management Functional Descriptor. */
    USB_DESC_BYTE(5), /* bFunctionLength.                 */
    USB_DESC_BYTE(0x24), /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE(0x01), /* bDescriptorSubtype (Call Management
     Functional Descriptor).          */
    USB_DESC_BYTE(0x01), /* bmCapabilities (D0+D1).          */
    USB_DESC_BYTE(0x01), /* bDataInterface.                  */
    /* ACM Functional Descriptor.*/
    USB_DESC_BYTE(4), /* bFunctionLength.                 */
    USB_DESC_BYTE(0x24), /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE(0x02), /* bDescriptorSubtype (Abstract
     Control Management Descriptor).  */
    USB_DESC_BYTE(0x02), /* bmCapabilities.                  */
    /* Union Functional Descriptor.*/
    USB_DESC_BYTE(5), /* bFunctionLength.                 */
    USB_DESC_BYTE(0x24), /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE(0x06), /* bDescriptorSubtype (Union
     Functional Descriptor).          */
    USB_DESC_BYTE(0x00), /* bMasterInterface (Communication
     Class Interface).                */
    USB_DESC_BYTE(0x01), /* bSlaveInterface0 (Data Class
     Interface).                      */
    /* Endpoint 1 Descriptor.*/
    USB_DESC_ENDPOINT(EP_INT|0x80, 0x03, /* bmAttributes (Interrupt).        */
                      0x0008, /* wMaxPacketSize.                  */
                      0x0A), /* bInterval.                       */

    /* Interface Descriptor.*/
    USB_DESC_INTERFACE(0x01, /* bInterfaceNumber.                */
                       0x00, /* bAlternateSetting.               */
                       0x02, /* bNumEndpoints.                   */
                       0xFF, /* bInterfaceClass (Data Class
                        Interface, CDC section 4.5).     */
                       0x00, /* bInterfaceSubClass (CDC section
                        4.6).                            */
                       0x00, /* bInterfaceProtocol (CDC section
                        4.7).                            */
                       0x00), /* iInterface.                      */
    // Endpoint 5 Descriptor Direction out
    //7 Bytes
    USB_DESC_ENDPOINT(EP_OUT|USB_RTYPE_DIR_HOST2DEV,// bEndpointAddress
        0x02,// bmAttributes (Bulk)
        OUT_PACKETSIZE,// wMaxPacketSize
        CHUNK_INTERVAL_FS),// bInterval
    /* Endpoint 2 Descriptor, Direction in*/
    //7 Bytes
    USB_DESC_ENDPOINT(EP_IN|USB_RTYPE_DIR_DEV2HOST, /* bEndpointAddress */
                      0x02, /* bmAttributes (Bulk)  */
                      IN_PACKETSIZE, /* wMaxPacketSize       */
                      CHUNK_INTERVAL_FS), /* bInterval            */

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
static const uint8_t stringLanguage[] = {USB_DESC_BYTE(4), /* bLength.                         */
                                         USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
                                         USB_DESC_WORD(0x0409) /* wLANGID (U.S. English).          */
};

/*
 * Vendor string.
 */
static const uint8_t stringVendor[] = {USB_DESC_BYTE(0x14), /* bLength.                         */
                                       USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
                                       'S', 0, 'h', 0, 't', 0, 'e', 0, 'p', 0,
                                       'c', 0, 'e', 0, 'l', 0, 'l', 0, };

/*
 * Device Description string.
 */
static const uint8_t stringDeviceDescriptor[] =
    {USB_DESC_BYTE(0x1A), /* bLength.                         */
     USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
     'C', 0, 'o', 0, 'm', 0, 'b', 0, 'i', 0, 'A', 0, 'd', 0, 'a', 0, 'p', 0,
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
 * Strings wrappers array.
 */
static const USBDescriptor vcom_strings[] = {
    {sizeof stringLanguage, stringLanguage},
    {sizeof stringVendor, stringVendor}, {sizeof stringDeviceDescriptor,
                                          stringDeviceDescriptor},
    {sizeof stringSerialNumber, stringSerialNumber}, {sizeof stringNotFound,
                                                      stringNotFound}};

extern const USBConfig usb_config;
extern bool is_ready(void);
extern void dataTransmitted(USBDriver *usbp, usbep_t ep);
extern void dataReceived(USBDriver *usbp, usbep_t ep);

#endif
