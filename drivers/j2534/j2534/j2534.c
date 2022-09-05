// SPDX-License-Identifier: MIT

/* 
 * WQCAN J2534 API Library.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#include <libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "log.h"
#include "utils.h"

char LAST_ERROR[80];

#define WQCAN_VENDOR_ID	    0xFFFF                      /* USB vendor ID used by the device */
#define WQCAN_PRODUCT_ID	0x0005                      /* USB product ID used by the device */
#define WQCAN_USB_EP_IN	    (LIBUSB_ENDPOINT_IN  | 2)   /* endpoint address */
#define WQCAN_USB_EP_OUT	(LIBUSB_ENDPOINT_OUT | 2)   /* endpoint address */
#define WQCAN_TIMEOUT	    3000                        /* Connection timeout (in ms) */

/* WQCAN command id */
#define WQCAN_CMD_READ_FW_VERSION 0x20
#define WQCAN_CMD_READ_VBAT 0x24

#define WQCAN_CMD_SWCAN_OPEN 0x60
#define WQCAN_CMD_SWCAN_BIT_RATE 0x61
#define WQCAN_CMD_SWCAN_RECEIVE_MESSAGE 0x62
#define WQCAN_CMD_SWCAN_TRANSMIT_MESSAGE 0x63

#define WQCAN_CMD_CAN_OPEN 0x80
#define WQCAN_CMD_CAN_BIT_RATE 0x81
#define WQCAN_CMD_CAN_RECEIVE_MESSAGE 0x85
#define WQCAN_CMD_CAN_TRANSMIT_MESSAGE 0x86

static libusb_context *ctx = NULL;
static libusb_device_handle *handle;

static uint8_t receiveBuf[64];
uint8_t transferBuf[64];

static int usb_send_replay(uint8_t *data, const size_t len, const int rlen, const uint32_t timeout, uint8_t *result, int bytes_read) {
	int bytes_written = 0, r = LIBUSB_ERROR_OTHER;

	if (len > 0) {
		r = libusb_bulk_transfer(handle, WQCAN_USB_EP_OUT, data, (int)len, &bytes_written, timeout);
	}
	if (r != LIBUSB_SUCCESS) {
		snprintf(LAST_ERROR, MAX_LEN,
			"USB data transfer error sending %d bytes: %s", (int)len, libusb_error_name(r));
	}
	else {
		if (timeout > 0 && result != NULL) {
			r = libusb_bulk_transfer(handle, WQCAN_USB_EP_IN, result, rlen, &bytes_read, timeout);

			if (r != LIBUSB_SUCCESS) {
				snprintf(LAST_ERROR, MAX_LEN, "USB data transfer error: %s", libusb_error_name(r));
				return r;
			}
		}
	}
	return r;
}

/*
 * Establish a connection with a PassThru device.
 */
long PTAPI PassThruOpen(void *pName, unsigned long *pDeviceID) {
	check_debug_log();
	log_trace("PassThruOpen: Device Name: %s, ID: %lu", (char*)pName, pDeviceID);
	
	libusb_init(&ctx);
	libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, 3);

    //Open Device with VendorID and ProductID
	handle = libusb_open_device_with_vid_pid(ctx, WQCAN_VENDOR_ID, WQCAN_PRODUCT_ID);
	if (!handle) {
		log_trace("device not found");
		libusb_exit(NULL);
		return ERR_DEVICE_NOT_CONNECTED;
	}

	int r = 1;
	//Claim Interface 0 from the device
    r = libusb_claim_interface(handle, 0);
	if (r < 0) {
		log_trace("usb_claim_interface error %d\n", r);
		libusb_close(handle);
		libusb_exit(NULL);
		return ERR_DEVICE_NOT_CONNECTED;
	}
	log_trace("Interface claimed\n");

	snprintf(LAST_ERROR, MAX_LEN, "PassThruOpen is not implemented");
	return STATUS_NOERROR;
}

/*
 * Terminate a connection with a Pass-Thru device.
 */
long PTAPI PassThruClose(unsigned long DeviceID) {
	log_trace("PassThruClose: DeviceID:  %lu", DeviceID);
	
	libusb_close(handle);
	libusb_exit(NULL);
	
	snprintf(LAST_ERROR, MAX_LEN, "PassThruClose is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Establish a connection using a protocol channel.
 */
long PTAPI PassThruConnect(unsigned long DeviceID, unsigned long protocolID, 
						   unsigned long flags, unsigned long baud, unsigned long *pChannelID) {
	log_trace("PassThruConnect: DeviceID: %lu, protocolID: %lu, flags: %08lX, baud: %lu",
			  DeviceID, protocolID, flags, baud);
	long err = ERR_NOT_SUPPORTED;
	switch (protocolID) {
		case J1850VPW:
		case J1850PWM:
		case J1850VPW_PS:
		case J1850PWM_PS:
		case ISO9141:
		case ISO9141_PS:
		case ISO14230:
		case ISO14230_PS:
			break;
		case CAN:
		case CAN_PS:
		case SW_CAN_PS:
			break;
		case ISO15765:
		case ISO15765_PS:
		case SCI_A_ENGINE:
		case SCI_A_TRANS:
		case SCI_B_ENGINE:
		case SCI_B_TRANS:
		case J2610_PS:
		default:
			break;
	}
	snprintf(LAST_ERROR, MAX_LEN, "PassThruConnect is partial implemented");
	return err;
}

/*
 * Terminate a connection with a protocol channel.
 */
long PTAPI PassThruDisconnect(unsigned long ChannelID) {
	log_trace("PassThruDisconnect: ChannelID: %lu", ChannelID);

	snprintf(LAST_ERROR, MAX_LEN, "PassThruDisconnect is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Read message(s) from a protocol channel.
 */
long PTAPI PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, 
							unsigned long *pNumMsgs, unsigned long Timeout) {
	log_trace("PassThruReadMsgs: ChannelID:%lu, Timeout: %u msec", ChannelID, Timeout);

	snprintf(LAST_ERROR, MAX_LEN, "PassThruReadMsgs is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Write message(s) to a protocol channel.
 */
long PTAPI PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
							 unsigned long *pNumMsgs, unsigned long timeInterval) {
	log_trace("PassThruWriteMsgs: ChannelID: %lu, Interval: %lu msec", ChannelID, timeInterval);

	snprintf(LAST_ERROR, MAX_LEN, "PassThruWriteMsgs is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Start sending a message at a specified time interval on a protocol channel.
 */
long PTAPI PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg, 
									unsigned long *pMsgID, unsigned long timeInterval) {
	log_trace("PassThruStartPeriodicMsg, ChannelID: %lu, Interval: %lu msec", ChannelID, timeInterval);
	
	snprintf(LAST_ERROR, MAX_LEN, "PassThruStartPeriodicMsg is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Stop a periodic message.
 */
long PTAPI PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long msgID) {
	log_trace("PassThruStopPeriodicMsg, ChannelID: %lu", ChannelID, msgID);
	
	snprintf(LAST_ERROR, MAX_LEN, "PassThruStopPeriodicMsg is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Start filtering incoming messages on a protocol channel.
 */
long PTAPI PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG *pMaskMsg,
								  PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, unsigned long *pMsgID) {
	log_trace("PassThruStartMsgFilter: ChannelID: %lu, FilterType: %lu, \n pMaskMsg: %s \n pPatternMsg: %s \n pFlowControlMsg: %s",
			  ChannelID, FilterType, parsemsg(pMaskMsg), parsemsg(pPatternMsg), parsemsg(pFlowControlMsg));

	snprintf(LAST_ERROR, MAX_LEN, "PassThruStartMsgFilter is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Stops filtering incoming messages on a protocol channel.
 */
long PTAPI PassThruStopMsgFilter(unsigned long ChannelID, unsigned long msgID) {
	log_trace("PassThruStopMsgFilter: ChannelID: lu, msgID: %lu", ChannelID, msgID);

	snprintf(LAST_ERROR, MAX_LEN, "PassThruStopMsgFilter is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Set a programming voltage on a specific pin.
 */
long PTAPI PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage) {
	log_trace("PassThruSetProgrammingVoltage: DeviceID: %lu, pinNumber: %lu, voltage: %lu", DeviceID, PinNumber, Voltage);

	snprintf(LAST_ERROR, MAX_LEN, "PassThruSetProgrammingVoltage is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Reads the version information of the firmware, this DLL and API implmentation.
 */
long PTAPI PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion, char *pDllVersion, char *pApiVersion) {
	log_trace("PassThruReadVersion: DeviceID: %lu", DeviceID);

	if (pFirmwareVersion == NULL || pDllVersion == NULL || pApiVersion == NULL) {
		snprintf(LAST_ERROR, MAX_LEN, "Error: Version* must not be NULL");
		return ERR_NULL_PARAMETER;
	}

	char dll_ver[MAX_LEN];

#if defined(LIBUSB_API_VERSION) || defined(LIBUSBX_API_VERSION) || defined(LIBUSB1010)
	const struct libusb_version *ver = libusb_get_version();
	snprintf(dll_ver, MAX_LEN, "%s (libusb-%u.%u.%u.%u%s)", J2534_DLL_VERSION,
		ver->major, ver->minor, ver->micro, ver->nano, ver->rc);
#else
	snprintf(dll_ver, MAX_LEN, "%s", DLL_VERSION);
#endif
	
	//TODO: read from usb
	//usb_send_replay(uint8_t *data, const size_t len, const int rlen, const uint32_t timeout, uint8_t *result) {
	char fw_version[MAX_LEN];
	uint8_t cmd[4] = {WQCAN_CMD_READ_FW_VERSION, 0, 0, 0};
	uint8_t result[16] = {0};
	uint8_t rbytes = 0;
	usb_send_replay(cmd, 4, 6, 300, result, rbytes);
	snprintf(fw_version, MAX_LEN, "STM32CAN v%d.%d", result[3], result[4]);

	if (fw_version[0] != 0)
		strcpy(pFirmwareVersion, fw_version);
	else 
		strcpy(pFirmwareVersion, "unavailable");

	strcpy(pDllVersion, J2534_DLL_VERSION);
	strcpy(pApiVersion, J2534_APIVER_NOVEMBER_2004);

	return STATUS_NOERROR;
}

/*
 * Gets the text description of the last error.
 */
long PTAPI PassThruGetLastError(char *pErrorDescription) {
	if (pErrorDescription == NULL) {
		return ERR_NULL_PARAMETER;
	}

	memcpy(pErrorDescription, LAST_ERROR, MAX_LEN);
	log_trace("GetLastError: LAST_ERROR: %s", LAST_ERROR);
	return STATUS_NOERROR;
}

/*
 * General I/O control functions for reading and writing
 * protocol configuration parameters (e.g. initialization,
 * baud rates, programming voltages, etc.).
 */
long PTAPI PassThruIoctl(unsigned long ChannelID, unsigned long ioctlID, void *pInput, void *pOutput) {
	log_trace("PassThruIoctl ChannelID: %lu, ioctlID: 0x%x", ChannelID, ioctlID);
	long err = ERR_NOT_SUPPORTED;
	switch(ioctlID) {
		case GET_CONFIG:
			const SCONFIG_LIST *inputlist = pInput;
			break;
		case SET_CONFIG:
			//const SCONFIG_LIST *inputlist = pInput;
			break;
		case READ_VBATT:
			uint16_t voltage = 0;
			uint8_t cmd[4] = {WQCAN_CMD_READ_VBAT, 0, 0, 0};
			uint8_t result[16] = {0};
			int rbyte = 0;
			if (usb_send_replay(cmd, 4, 6, 300, result, rbyte) != LIBUSB_SUCCESS) {
				err = ERR_NOT_SUPPORTED;
				break;
			}
			memcpy(&voltage, result+3, 2);
			log_trace("Voltage: %d", voltage);
			*(uint32_t*)pOutput = voltage;
			err = STATUS_NOERROR;
			break;
		case FIVE_BAUD_INIT:
		case FAST_INIT:
		case CLEAR_TX_BUFFER:
		case CLEAR_RX_BUFFER:
		case CLEAR_PERIODIC_MSGS:
		case CLEAR_MSG_FILTERS:
		case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
		case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
		case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
		case READ_PROG_VOLTAGE:
			break;
		
	}
	snprintf(LAST_ERROR, MAX_LEN, "PassThruIoctl is not implemented");
	return err;
}
