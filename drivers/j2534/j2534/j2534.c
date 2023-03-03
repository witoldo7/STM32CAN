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

#define DEVICE_ID 12345
#define WQCAN_VENDOR_ID	    0xFFFF                      /* USB vendor ID used by the device */
#define WQCAN_PRODUCT_ID	0x0005                      /* USB product ID used by the device */
#define WQCAN_USB_EP_IN	    (LIBUSB_ENDPOINT_IN  | 2)   /* endpoint address */
#define WQCAN_USB_EP_OUT	(LIBUSB_ENDPOINT_OUT | 2)   /* endpoint address */
#define WQCAN_TIMEOUT	    3000                        /* Connection timeout (in ms) */

/* WQCAN command id */
#define WQCAN_CMD_READ_FW_VERSION 0x20



#if defined(_MSC_VER)
#define snprintf _snprintf
#endif

#if defined(PLATFORM_POSIX)
#include <fcntl.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

#define THREAD_RETURN_VALUE	NULL
typedef sem_t * semaphore_t;
typedef pthread_t thread_t;

static inline semaphore_t semaphore_create(void)
{
	sem_t *semaphore;
	char name[50];

	snprintf(name, sizeof(name), "/org.libusb.example.dpfp_threaded:%d", (int)getpid());
	semaphore = sem_open(name, O_CREAT | O_EXCL, 0, 0);
	if (semaphore == SEM_FAILED)
		return NULL;
	/* Remove semaphore so that it does not persist after process exits */
	(void)sem_unlink(name);
	return semaphore;
}

static inline void semaphore_give(semaphore_t semaphore)
{
	(void)sem_post(semaphore);
}

static inline void semaphore_take(semaphore_t semaphore)
{
	(void)sem_wait(semaphore);
}

static inline void semaphore_destroy(semaphore_t semaphore)
{
	(void)sem_close(semaphore);
}

static inline int thread_create(thread_t *thread,
	void *(*thread_entry)(void *arg), void *arg)
{
	return pthread_create(thread, NULL, thread_entry, arg) == 0 ? 0 : -1;
}

static inline void thread_join(thread_t thread)
{
	(void)pthread_join(thread, NULL);
}
#elif defined(PLATFORM_WINDOWS)
#define THREAD_RETURN_VALUE	0
typedef HANDLE semaphore_t;
typedef HANDLE thread_t;

#if defined(__CYGWIN__)
typedef DWORD thread_return_t;
#else
#include <process.h>
typedef unsigned thread_return_t;
#endif

static inline semaphore_t semaphore_create(void)
{
	return CreateSemaphore(NULL, 0, 1, NULL);
}

static inline void semaphore_give(semaphore_t semaphore)
{
	(void)ReleaseSemaphore(semaphore, 1, NULL);
}

static inline void semaphore_take(semaphore_t semaphore)
{
	(void)WaitForSingleObject(semaphore, INFINITE);
}

static inline void semaphore_destroy(semaphore_t semaphore)
{
	(void)CloseHandle(semaphore);
}

static inline int thread_create(thread_t *thread,
	thread_return_t (__stdcall *thread_entry)(void *arg), void *arg)
{
#if defined(__CYGWIN__)
	*thread = CreateThread(NULL, 0, thread_entry, arg, 0, NULL);
#else
	*thread = (HANDLE)_beginthreadex(NULL, 0, thread_entry, arg, 0, NULL);
#endif
	return *thread != NULL ? 0 : -1;
}

static inline void thread_join(thread_t thread)
{
	(void)WaitForSingleObject(thread, INFINITE);
	(void)CloseHandle(thread);
}
#endif

static libusb_context *ctx = NULL;
static libusb_device_handle *handle;

static uint8_t receiveBuf[64];
uint8_t transferBuf[64];

uint8_t covertPacketToBuffer(packet_t *packet, uint8_t *buffer) {
  uint8_t size = 0;
  if (packet != (packet_t*)0x0) {
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
  return size;
}

static int usb_send_replay(uint8_t *data, const size_t len, const int rlen, const uint32_t timeout, uint8_t *result, int bytes_read) {
	int bytes_written = 0, r = LIBUSB_ERROR_OTHER;

	if (len > 0) {
		r = libusb_bulk_transfer(handle, WQCAN_USB_EP_OUT, data, (int)len, &bytes_written, timeout);
	}
	if (r != LIBUSB_SUCCESS) {
		last_error("USB data transfer error sending %d bytes: %s", (int)len, libusb_error_name(r));
	}
	else {
		if (timeout > 0 && result != NULL) {
			r = libusb_bulk_transfer(handle, WQCAN_USB_EP_IN, result, rlen, &bytes_read, timeout);

			if (r != LIBUSB_SUCCESS) {
				last_error("USB data transfer error: %s", libusb_error_name(r));
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
	*pDeviceID = DEVICE_ID;
	log_trace("PassThruOpen: Device Name: %s, ID: %lu", (char*)pName, *pDeviceID);
	
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
    r = libusb_claim_interface(handle, 1);
	if (r < 0) {
		log_trace("usb_claim_interface error %d", r);
		libusb_close(handle);
		libusb_exit(NULL);
		return ERR_DEVICE_NOT_CONNECTED;
	}

	last_error("PassThruOpen Interface claimed");
	return STATUS_NOERROR;
}

/*
 * Terminate a connection with a Pass-Thru device.
 */
long PTAPI PassThruClose(unsigned long DeviceID) {
	log_trace("PassThruClose: DeviceID:  %lu", DeviceID);
	
	libusb_close(handle);
	libusb_exit(NULL);
	
	last_error("PassThruClose partial impl");
	return STATUS_NOERROR;
}

/*
 * Establish a connection using a protocol channel.
 */
long PTAPI PassThruConnect(unsigned long DeviceID, unsigned long protocolID, 
						   unsigned long flags, unsigned long baud, unsigned long *pChannelID) {
	log_trace("PassThruConnect: DeviceID: %lu, protocolID: %lu, flags: %08lX, baud: %lu",
			  DeviceID, protocolID, flags, baud);
	unsigned long err = ERR_NOT_SUPPORTED;
	unsigned long channelID = 0;
	uint8_t buff[24] = {0}, buffer[32] = {0}, result[24] = {0}, size = 0;
	int rbyte = 0;
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_connect, .data_len = 24};

	memcpy(buff, &protocolID, sizeof(protocolID));
	memcpy(buff + 8, &flags, sizeof(flags));
	memcpy(buff + 16, &baud, sizeof(baud));

	size = covertPacketToBuffer(&txPacket, buffer);

	if (usb_send_replay(buffer, size, 20, 200, result, rbyte) != LIBUSB_SUCCESS) {
		return ERR_NOT_SUPPORTED;
	}

	memcpy(&err, result + 3, sizeof(err));
	memcpy(&channelID, result + 11, sizeof(channelID));

	*pChannelID = channelID;
	log_trace("PassThruConnect: cid: %lu, err:%lu", *pChannelID, err);
	last_error("PassThruConnect: connected");
	return err;
}

/*
 * Terminate a connection with a protocol channel.
 */
long PTAPI PassThruDisconnect(unsigned long ChannelID) {
	log_trace("PassThruDisconnect: ChannelID: %lu", ChannelID);
	unsigned long err = ERR_NOT_SUPPORTED;

	uint8_t buff[24] = {0}, buffer[12] = {0}, result[24] = {0}, size = 0;
	int rbyte = 0;
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_disconnect, .data_len = 8};

	memcpy(buff, &ChannelID, sizeof(ChannelID));
	size = covertPacketToBuffer(&txPacket, buffer);

	if (usb_send_replay(buffer, size, 20, 200, result, rbyte) != LIBUSB_SUCCESS) {
		return ERR_NOT_SUPPORTED;
	}

	memcpy(&err, result + 3, sizeof(err));

	last_error("PassThruDisconnect");
	return err;
}

/*
 * Read message(s) from a protocol channel.
 */
long PTAPI PassThruReadMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg, 
							unsigned long *pNumMsgs, unsigned long Timeout) {
	log_trace("PassThruReadMsgs: ChannelID:%lu, Timeout: %u msec", ChannelID, Timeout);
	PASSTHRU_MSG* msg_out = &pMsg[0];
	CANRxFrame rxMsg = {0};
	unsigned long msg_cnt = 0;
	*pNumMsgs = msg_cnt;
	unsigned long err = ERR_NOT_SUPPORTED;
	uint8_t buff[16] = {0}, buffer[24] = {0}, result[120] = {0}, size = 0;
	int rbyte = 0;
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_read_message, .data_len = 16};
	memcpy(buff, &ChannelID, sizeof(ChannelID));
	memcpy(buff + 8, &Timeout, sizeof(Timeout));

	size = covertPacketToBuffer(&txPacket, buffer);
	
	if (Timeout < 50)
		Timeout = 50;

	if (usb_send_replay(buffer, size, 76, Timeout, result, rbyte) != LIBUSB_SUCCESS) {
		return ERR_NOT_SUPPORTED;
	}

	uint16_t len = (uint16_t)((result[1] & 0xffffU) << 8) | (uint16_t)result[2];
	if (len < 72) {
		log_trace("empty buffer, len: %d", len);
		memcpy(&err, result + 3, sizeof(err));
		return err;
	}
	if (len != 72) {
		return ERR_INVALID_MSG;
	}
	memcpy(&rxMsg, result + 3, sizeof(CANRxFrame));
	msg_cnt = 1;
	*pNumMsgs = msg_cnt;

	msg_out->ProtocolID = CAN;
	msg_out->DataSize = rxMsg.DLC + 4;
	uint32_t id = 0;
	if (rxMsg.common.XTD)
		id = rxMsg.ext.EID & 0x1FFFFFFF;
	else
		id = rxMsg.std.SID & 0x7FF;

	msg_out->Data[0] = (uint8_t)(id >> 24);
	msg_out->Data[1] = (uint8_t)(id >> 16);
	msg_out->Data[2] = (uint8_t)(id >> 8);
	msg_out->Data[3] = (uint8_t)id;
	memcpy(msg_out->Data + 4, rxMsg.data8, rxMsg.DLC);
	msg_out->Timestamp = rxMsg.RXTS +10000000;
	msg_out->RxStatus = (rxMsg.common.XTD ? CAN_29BIT_ID : 0) | TX_MSG_TYPE;
	msg_out->ExtraDataIndex = 0;
	msg_out->TxFlags = TX_MSG_TYPE;

	log_trace("%s", parsemsg(msg_out));
	return STATUS_NOERROR;
}

bool PASSTHRU_MSG_To_CANTxFrame(PASSTHRU_MSG *pMsg, CANTxFrame *canTx) {
	bool xtd = (pMsg->TxFlags) & CAN_29BIT_ID;
	canTx->common.XTD = xtd;
	if (pMsg->DataSize < 4)
		return false;;
	uint32_t dlc = pMsg->DataSize - 4;
	uint32_t id = pMsg->Data[0] << 24 | pMsg->Data[1] << 16 | pMsg->Data[2] << 8 | pMsg->Data[3];

	if (xtd) {
		canTx->ext.EID = id & 0x1FFFFFFF;;
	} else {
		canTx->std.SID = id & 0x7FF;
	}
	canTx->DLC = dlc;
	memcpy(canTx->data8, pMsg->Data + 4, dlc);
	return true;
}

/*
 * Write message(s) to a protocol channel.
 */
long PTAPI PassThruWriteMsgs(unsigned long ChannelID, PASSTHRU_MSG *pMsg,
							 unsigned long *pNumMsgs, unsigned long timeInterval) {
	log_trace("PassThruWriteMsgs: ChannelID: %lu, NumMsg: %lu, Interval: %lu msec\r\n MSG: %s", ChannelID, *pNumMsgs, timeInterval, parsemsg(pMsg));
	unsigned long err = ERR_NOT_SUPPORTED;
	uint8_t buff[100] = {0}, buffer[100] = {0}, result[12] = {0}, size = 0;
	int rbyte = 0;
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_write_message, .data_len = 88};

	memcpy(buff, &ChannelID, sizeof(ChannelID));
	memcpy(buff + 8 , pNumMsgs, sizeof(pNumMsgs));

	switch (ChannelID) {
		case CAN:
		case CAN_PS:
		case ISO15765:
			CANTxFrame tx = {0};
			PASSTHRU_MSG_To_CANTxFrame(&pMsg[0], &tx);
			memcpy(buff + 16, &tx, sizeof(CANTxFrame));
			break;
		case SW_CAN_PS:
			break;
		default:
			break;
	}

	size = covertPacketToBuffer(&txPacket, buffer);

	if (usb_send_replay(buffer, size, 12, 600, result, rbyte) != LIBUSB_SUCCESS) {
		return ERR_NOT_SUPPORTED;
	}

	memcpy(&err, result + 3, sizeof(err));
	return err;;
}

/*
 * Start sending a message at a specified time interval on a protocol channel.
 */
long PTAPI PassThruStartPeriodicMsg(unsigned long ChannelID, PASSTHRU_MSG *pMsg, 
									unsigned long *pMsgID, unsigned long timeInterval) {
	log_trace("PassThruStartPeriodicMsg, ChannelID: %lu, Interval: %lu msec", ChannelID, timeInterval);
	
	last_error("PassThruStartPeriodicMsg is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Stop a periodic message.
 */
long PTAPI PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long msgID) {
	log_trace("PassThruStopPeriodicMsg, ChannelID: %lu", ChannelID, msgID);
	last_error("PassThruStopPeriodicMsg is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Start filtering incoming messages on a protocol channel.
 */
long PTAPI PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType, PASSTHRU_MSG *pMaskMsg,
								  PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, unsigned long *pMsgID) {
	log_trace("PassThruStartMsgFilter: ChannelID: %lu, FilterType: %lu, \n pMaskMsg: %s \n pPatternMsg: %s \n pFlowControlMsg: %s",
			  ChannelID, FilterType, parsemsg(pMaskMsg), parsemsg(pPatternMsg), parsemsg(pFlowControlMsg));
	if (pMaskMsg == NULL || pPatternMsg == NULL)
	{
		last_error("Error: PASSTHRU_MSG* must not be NULL");
		return ERR_NULL_PARAMETER;
	}
	if (pMaskMsg->DataSize > 12 || pPatternMsg->DataSize > 12)
	{
		last_error("Error: PASSTHRU_MSG invalid data length");
		return ERR_INVALID_MSG;
	}
	if (pMaskMsg->DataSize != pPatternMsg->DataSize)
	{
		last_error("Error: Mask and Pattern have differnt data lenghts");
		return ERR_INVALID_MSG;
	}
	if (pMaskMsg->TxFlags != pPatternMsg->TxFlags)
	{
		last_error("Error: Mask and Pattern have differnt TX flags");
		return ERR_INVALID_MSG;
	}
	if ((FilterType == PASS_FILTER || FilterType == BLOCK_FILTER) && pFlowControlMsg)
	{
		last_error("Error: FilterType, FlowControlMsg mismatch");
		return ERR_INVALID_MSG;
	}
	unsigned long err = ERR_NOT_SUPPORTED;

	uint8_t buff[24] = {0}, buffer[28] = {0}, result[24] = {0}, size = 0;
	int rbyte = 0;
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_filter, .data_len = 8};

	memcpy(buff, &ChannelID, sizeof(ChannelID));
	size = covertPacketToBuffer(&txPacket, buffer);

	if (usb_send_replay(buffer, size, 12, 200, result, rbyte) != LIBUSB_SUCCESS) {
		return ERR_NOT_SUPPORTED;
	}

	memcpy(&err, result + 3, sizeof(err));

	/*
	switch (FilterType) {
		case PASS_FILTER:
			switch (pMaskMsg->ProtocolID) {
				case CAN:
				case CAN_PS:
					uint8_t cmd_filter[5] = {WQCAN_CMD_CAN_FILTER, 0, 1, 0, 0};
					uint8_t result[4] = {0};
					int rbyte = 0;
					//TODO:
					if (usb_send_replay(cmd_filter, 5, 4, 400, result, rbyte) != LIBUSB_SUCCESS) {
						return ERR_NOT_SUPPORTED;
					}
					if (result[3] != 0) {
						log_trace("Set filter failed.");
						return ERR_NOT_SUPPORTED;
					}
					return STATUS_NOERROR;
				case SW_CAN_PS:
					break;
				default:
					break;
			}
			break;
		case BLOCK_FILTER:
			break;
		case FLOW_CONTROL_FILTER:
			break;
		default:
			break;
	}
	*/
	last_error("PassThruStartMsgFilter TODO");
	return err;
}

/*
 * Stops filtering incoming messages on a protocol channel.
 */
long PTAPI PassThruStopMsgFilter(unsigned long ChannelID, unsigned long msgID) {
	log_trace("PassThruStopMsgFilter: ChannelID: lu, msgID: %lu", ChannelID, msgID);

	last_error("PassThruStopMsgFilter is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Set a programming voltage on a specific pin.
 */
long PTAPI PassThruSetProgrammingVoltage(unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage) {
	log_trace("PassThruSetProgrammingVoltage: DeviceID: %lu, pinNumber: %lu, voltage: %lu", DeviceID, PinNumber, Voltage);

	last_error("PassThruSetProgrammingVoltage is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Reads the version information of the firmware, this DLL and API implmentation.
 */
long PTAPI PassThruReadVersion(unsigned long DeviceID, char *pFirmwareVersion, char *pDllVersion, char *pApiVersion) {
	log_trace("PassThruReadVersion: DeviceID: %lu", DeviceID);

	if (pFirmwareVersion == NULL || pDllVersion == NULL || pApiVersion == NULL) {
		last_error("Error: Version* must not be NULL");
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
	memcpy(pErrorDescription, getLastError(), MAX_LEN);
	return STATUS_NOERROR;
}

/*
 * General I/O control functions for reading and writing
 * protocol configuration parameters (e.g. initialization,
 * baud rates, programming voltages, etc.).
 */
long PTAPI PassThruIoctl(unsigned long ChannelID, unsigned long ioctlID, void *pInput, void *pOutput) {
	log_trace("PassThruIoctl ChannelID: %lu, ioctlID: 0x%x", ChannelID, ioctlID);
	unsigned long err = STATUS_NOERROR;//ERR_NOT_SUPPORTED;
	uint8_t buff[24] = {0}, buffer[32] = {0}, result[24] = {0};
	int rbyte = 0, size = 0;
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_ioctl};
	memcpy(buff, &ChannelID, sizeof(ChannelID));
	memcpy(buff + 8, &ioctlID, sizeof(ioctlID));

	switch(ioctlID) {
		case GET_CONFIG:
			//const SCONFIG_LIST *inputlist = pInput;
			last_error("PassThruIoctl ioctlID: %x is not implemented", ioctlID);
			break;
		case SET_CONFIG:
			last_error("PassThruIoctl ioctlID: %x is not implemented", ioctlID);
			break;
		case READ_VBATT:
			txPacket.data_len = 16;
			size = covertPacketToBuffer(&txPacket, buffer);
			if (usb_send_replay(buffer, size, 14, 200, result, rbyte) != LIBUSB_SUCCESS) {
				return ERR_NOT_SUPPORTED;
			}
			uint16_t voltage = 0;
			memcpy(&err, result + 3, sizeof(err));
			memcpy(&voltage, result + 11, sizeof(voltage));
			log_trace("Voltage: %d", voltage);
			*(uint32_t*)pOutput = voltage;
			break;
		case FIVE_BAUD_INIT:
		case FAST_INIT:
		case CLEAR_TX_BUFFER:
			last_error("PassThruIoctl ioctlID: %x is not implemented", ioctlID);
			break;
		case CLEAR_RX_BUFFER:
			txPacket.data_len = 16;
			size = covertPacketToBuffer(&txPacket, buffer);
			if (usb_send_replay(buffer, size, 12, 200, result, rbyte) != LIBUSB_SUCCESS) {
				return ERR_NOT_SUPPORTED;
			}
			memcpy(&err, result + 3, sizeof(err));
			break;
		case CLEAR_PERIODIC_MSGS:
		case CLEAR_MSG_FILTERS:
		case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
		case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
		case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
		case READ_PROG_VOLTAGE:
			last_error("PassThruIoctl ioctlID: %x is not implemented", ioctlID);
			break;
		default:
			last_error("PassThruIoctl, unknown ioctlID: %x", ioctlID);
	}
	return err;
}
