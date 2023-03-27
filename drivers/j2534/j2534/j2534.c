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
#include <errno.h>
#include <signal.h>
#include "log.h"
#include "utils.h"

#define DEVICE_ID           12345
#define WQCAN_VENDOR_ID	    0xFFFF                      /* USB vendor ID used by the device */
#define WQCAN_PRODUCT_ID    0x0005                      /* USB product ID used by the device */
#define WQCAN_USB_EP_IN	    (LIBUSB_ENDPOINT_IN  | 2)   /* endpoint address */
#define WQCAN_USB_EP_OUT    (LIBUSB_ENDPOINT_OUT | 2)   /* endpoint address */
#define WQCAN_TIMEOUT	    100                         /* Connection timeout (in ms) */

static libusb_device_handle *handle;
static uint8_t databuf[128];
static struct libusb_transfer *data_transfer = NULL;

uint8_t data[32];
packet_t tx_packet = {.data = data};

uint8_t rspdata[128];
packet_t resp_packet = {.data = rspdata};

static bool isOpen = false;
static volatile sig_atomic_t do_exit = 0;
static semaphore_t wait_for_response_semaphore, txSem, rxSem;
semaphore_t slock;
static thread_t poll_thread;

static void request_exit() {
	do_exit = 1;
	libusb_cancel_transfer(data_transfer);
	semaphore_destroy(wait_for_response_semaphore);
	semaphore_destroy(rxSem);
	semaphore_destroy(txSem);
	semaphore_destroy(slock);
	log_trace("poll thread shutting down");
	thread_join(poll_thread);
	if (handle)
		libusb_release_interface(handle, 1);
	if (handle)
		libusb_close(handle);
	libusb_exit(NULL);
}

#if defined(_MSC_VER)
static thread_return_t __stdcall poll_thread_main(void *arg)
#else
static void *poll_thread_main(void *arg)
#endif
{
	(void)arg;
	log_trace("poll thread running");

	while (!do_exit) {
		struct timeval tv = { 1, 0 };
		int r;

		r = libusb_handle_events_timeout(NULL, &tv);

		if (r < 0) {
			log_error("poll thread exit r: %d", r);
			request_exit();
			break;
		}
	}
	return THREAD_RETURN_VALUE;
}

static void sighandler(int signum) {
	(void)signum;
	log_error("sighandler: %d", signum);
	request_exit();
}

static void setup_signals(void) {
#if defined(_MSC_VER)
	(void)signal(SIGINT, sighandler);
	(void)signal(SIGTERM, sighandler);
#else
	struct sigaction sigact;

	sigact.sa_handler = sighandler;
	sigemptyset(&sigact.sa_mask);
	sigact.sa_flags = 0;
	(void)sigaction(SIGINT, &sigact, NULL);
	(void)sigaction(SIGTERM, &sigact, NULL);
	(void)sigaction(SIGQUIT, &sigact, NULL);
#endif
}

static void LIBUSB_CALL cb_data(struct libusb_transfer *transfer) {
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		log_error("transfer status: %d", transfer->status);
		goto err_free_transfer;
	}

	uint16_t data_len = (uint16_t)((transfer->buffer[1] & 0xffffU) << 8) | (uint16_t)transfer->buffer[2];

	if ((data_len + 4) != transfer->actual_length) {
		log_error("Packet incomplete, transfer size: %d, packet size %d", transfer->actual_length, (data_len + 4));
		if (libusb_submit_transfer(data_transfer) < 0)
			goto err_free_transfer;
		return;
	}

	switch (transfer->buffer[0]) {
	case cmd_j2534_connect:
	case cmd_j2534_disconnect:
	case cmd_j2534_ioctl:
	case cmd_j2534_filter:
	case cmd_j2534_periodic_message:
	case cmd_j2534_misc:
		resp_packet.cmd_code = transfer->buffer[0];
		resp_packet.data_len = data_len;
		resp_packet.term = transfer->buffer[transfer->actual_length - 1];
		memcpy(resp_packet.data, transfer->buffer + 3, data_len);
		semaphore_give(wait_for_response_semaphore);
		break;
	case cmd_j2534_write_message:
		tx_packet.cmd_code = transfer->buffer[0];
		tx_packet.data_len = data_len;
		tx_packet.term = transfer->buffer[transfer->actual_length - 1];
		memcpy(tx_packet.data, transfer->buffer + 3, data_len);
		semaphore_give(wait_for_response_semaphore);
		break;
	case cmd_j2534_read_message:
		log_trace("new rx");
		PASSTHRU_MSG pMsg = { 0 };
		convertPacketToPMSG(transfer->buffer +3, data_len, &pMsg);
		enQueue(pMsg);
		break;
	default:
		break;
	}

	if (libusb_submit_transfer(data_transfer) < 0)
		goto err_free_transfer;

	return;

err_free_transfer:
	log_error("err_free_transfer");
	if (transfer != NULL) {
		libusb_free_transfer(transfer);
		data_transfer = NULL;
	}
}

static int alloc_transfers(void) {
	data_transfer = libusb_alloc_transfer(0);
	if (!data_transfer) {
		errno = ENOMEM;
		return -1;
	}

	libusb_fill_bulk_transfer(data_transfer, handle, WQCAN_USB_EP_IN, databuf,
		sizeof(databuf), cb_data, NULL, 0);

	return 0;
}

static int usb_send_packet(packet_t packet, unsigned int timeout) {
	int bytes_written = 0, r = LIBUSB_ERROR_OTHER;
	uint8_t buff[1024] ={0};
	uint16_t len = covertPacketToBuffer(&packet, buff);
	if (len > 0) {
		r = libusb_bulk_transfer(handle, WQCAN_USB_EP_OUT, buff, (int)len, &bytes_written, timeout);
	}
	if (r != LIBUSB_SUCCESS) {
		last_error("USB data transfer error sending %d bytes: %s", (int)len, libusb_error_name(r));
	}
	return r;
}

/*
 * Establish a connection with a PassThru device.
 */
uint32_t PTAPI PassThruOpen(void *pName, uint32_t *pDeviceID) {
	check_debug_log();
	*pDeviceID = DEVICE_ID;
	log_trace("PassThruOpen: Device Name: %s, ID: %lu", (char*)pName, *pDeviceID);
	libusb_init(NULL);
    //libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, 4);

    //Open Device with VendorID and ProductID
	handle = libusb_open_device_with_vid_pid(NULL, WQCAN_VENDOR_ID, WQCAN_PRODUCT_ID);
	if (!handle) {
		last_error("device not found");
		libusb_exit(NULL);
		return ERR_DEVICE_NOT_CONNECTED;
	}

    if (libusb_kernel_driver_active(handle, 1) == 1) {
        int r = libusb_detach_kernel_driver(handle, 1);
    }
	int r = 1;
	//Claim Interface 1 from the device
    r = libusb_claim_interface(handle, 1);
	if (r < 0) {
		last_error("usb_claim_interface error %d", r);
		libusb_close(handle);
		libusb_exit(NULL);
		return ERR_DEVICE_NOT_CONNECTED;
	}

	r = alloc_transfers();
	if (r < 0) {
		last_error("alloc_transfers error %d", r);
		goto out_deinit;
	}

	setup_signals();

	wait_for_response_semaphore = semaphore_create("resp");
	slock = semaphore_create("slock");
	txSem = semaphore_create("txSem");
	rxSem = semaphore_create("rxSem");

	if (!wait_for_response_semaphore || !slock || !txSem || !rxSem) {
		last_error("failed to initialize semaphore");
		goto out_deinit;
	}

	r = thread_create(&poll_thread, poll_thread_main, NULL);
	if (r) {
		last_error("failed to initialize poll thread");
		goto out_deinit;
	}

	r = libusb_submit_transfer(data_transfer);
	if (r < 0) {
		last_error("libusb_submit_transfer error %d", r);
	}

	last_error("PassThruOpen Interface claimed");
	isOpen = true;
	return STATUS_NOERROR;

out_deinit:
	if (data_transfer)
		libusb_free_transfer(data_transfer);
	return ERR_FAILED;
}

/*
 * Terminate a connection with a Pass-Thru device.
 */
uint32_t PTAPI PassThruClose(uint32_t DeviceID) {
	log_trace("PassThruClose: DeviceID:  %lu", DeviceID);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	isOpen = false;
	request_exit();
	log_trace("PassThruClose");

	return STATUS_NOERROR;
}

/*
 * Establish a connection using a protocol channel.
 */
uint32_t PTAPI PassThruConnect(uint32_t DeviceID, uint32_t protocolID,
						   uint32_t flags, uint32_t baud, uint32_t *pChannelID) {
	log_trace("PassThruConnect: DeviceID: %lu, protocolID: %lu, flags: %08lX, baud: %lu",
			  DeviceID, protocolID, flags, baud);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	uint32_t err = ERR_NOT_SUPPORTED;
	uint32_t channelID = 0;
	uint8_t buff[12] = {0};
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_connect, .data_len = 12};

	memcpy(buff, &protocolID, sizeof(protocolID));
	memcpy(buff + sizeof(protocolID), &flags, sizeof(flags));
	memcpy(buff + sizeof(protocolID) + sizeof(flags), &baud, sizeof(baud));

	if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
		last_error("PassThruConnect: Tx USB failed");
		return ERR_FAILED;
	}

	while (!do_exit & (resp_packet.cmd_code != cmd_j2534_connect))
		semaphore_take(wait_for_response_semaphore);

	memcpy(&err, resp_packet.data, sizeof(err));
	memcpy(&channelID, resp_packet.data +  sizeof(err), sizeof(channelID));

	*pChannelID = channelID;
	log_trace("PassThruConnect: cid: %lu, err:%lu", *pChannelID, err);
	return err;
}

/*
 * Terminate a connection with a protocol channel.
 */
uint32_t PTAPI PassThruDisconnect(uint32_t ChannelID) {
	log_trace("PassThruDisconnect: ChannelID: %lu", ChannelID);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	uint32_t err = ERR_NOT_SUPPORTED;
	uint8_t buff[24] = {0};
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_disconnect, .data_len = 4};

	memcpy(buff, &ChannelID, sizeof(ChannelID));

	if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
		last_error("PassThruConnect: Tx USB failed");
		return ERR_FAILED;
	}

	while (!do_exit & (resp_packet.cmd_code != cmd_j2534_disconnect))
		semaphore_take(wait_for_response_semaphore);

	memcpy(&err, resp_packet.data, sizeof(err));

	log_trace("PassThruDisconnect, err: %lu", err);
	return err;
}

uint32_t coppyMessages(PASSTHRU_MSG* pMsg, uint32_t pNumMsgs) {
	if (pNumMsgs == 0) {
		return ERR_BUFFER_EMPTY;
	}
	for (uint32_t i = 0; i < pNumMsgs; i++) {
		PASSTHRU_MSG msg = {0};
		bool status = deQueue(&msg);
		if (!status)
			return ERR_BUFFER_EMPTY;

		PASSTHRU_MSG* dest = &pMsg[i];
		memcpy(dest, &msg, sizeof(PASSTHRU_MSG));
		log_trace("RX: %s", parsemsg(&msg));
	}
	return STATUS_NOERROR;
}

/*
 * Read message(s) from a protocol channel.
 */
static uint32_t rx_counter=0;
uint32_t PTAPI PassThruReadMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout) {
	Lock(rxSem, rx_counter);
	log_trace("PassThruReadMsgs: ChannelID:%lu, Timeout: %u msec, numMsg: %lu", ChannelID, Timeout, *pNumMsgs);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	uint32_t queueSize = sizeQueue();
	if ((Timeout == 0) && (queueSize == 0)) {
		*pNumMsgs = 0;
		last_error("Zero messages received");
		return ERR_BUFFER_EMPTY;
	}

	if ((*pNumMsgs > 0) && (queueSize >= *pNumMsgs)) {
		return coppyMessages(pMsg, *pNumMsgs);
	}

	uint32_t err = ERR_NOT_SUPPORTED;
	uint32_t tm = Timeout*100;
	if (Timeout > 0) {
		if (Timeout < 10)
			tm = 1000;

		bool exit = false;
		for (uint32_t i = 0; (i < tm) & !do_exit & !exit; i++) {
		//while(!do_exit & !exit) {
			queueSize = sizeQueue();
			if ((*pNumMsgs == 0) && (queueSize > 0)) {
				exit = true;
				break;
			}
			if ((queueSize >= *pNumMsgs) && (*pNumMsgs != 0)) {
				exit = true;
				break;
			}
			sleep_ms(10);
		}

		if (queueSize == 0) {
			last_error("Zero messages received");
			*pNumMsgs = 0;
			return ERR_BUFFER_EMPTY; //Why not timeout?
		}
	}

	uint32_t msgCnt = 0;
	if (*pNumMsgs > 0) {
		msgCnt = queueSize > *pNumMsgs ? *pNumMsgs : queueSize;
	}
	else {
		msgCnt = queueSize;
	}

	err = coppyMessages(pMsg, msgCnt);
	*pNumMsgs = msgCnt;
	log_trace("PassThruReadMsgs err: %lu", err);
	Unlock(rxSem, rx_counter);
	return STATUS_NOERROR;
}

/*
 * Write message(s) to a protocol channel.
 */
static uint32_t tx_counter=0;
uint32_t PTAPI PassThruWriteMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t timeInterval) {
	Lock(txSem, tx_counter);
	log_trace("PassThruWriteMsgs: ChannelID: %lu, NumMsg: %lu, Interval: %lu msec\r\n MSG: %s", ChannelID, *pNumMsgs, timeInterval, parsemsg(pMsg));
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	if (*pNumMsgs > MAX_J2534_MESSAGES)
		return ERR_EXCEEDED_LIMIT;

	uint32_t err = ERR_NOT_SUPPORTED;
	uint8_t buff[100] = { 0 };
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_write_message};
	uint8_t offset = 0;
	memcpy(buff, &ChannelID, sizeof(ChannelID));
	offset += 4;
	memcpy(buff + offset, &timeInterval, sizeof(timeInterval));
	offset += 4;

	for (uint8_t i = 0; i < *pNumMsgs; i++) {
		switch (ChannelID) {
			case CAN:
			case CAN_PS:
			case SW_CAN_PS:
				CANTxFrame tx = {0};
				PASSTHRU_MSG_To_CANTxFrame(&pMsg[i], &tx);
				memcpy(buff + offset, &tx, 8 + tx.DLC);
				txPacket.data_len = offset + 8 + tx.DLC;
				break;
			case ISO15765:
			default:
				break;
		}

		if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
			last_error("PassThruWriteMsgs: Tx USB failed");
			return ERR_FAILED;
		}

		while (!do_exit & (tx_packet.cmd_code != cmd_j2534_write_message))
			semaphore_take(wait_for_response_semaphore);

		memcpy(&err, tx_packet.data, sizeof(err));
	}
	log_trace("PassThruWriteMsgs err: %lu", err);
	Unlock(txSem, tx_counter);
	return STATUS_NOERROR;
}

/*
 * Start sending a message at a specified time interval on a protocol channel.
 */
uint32_t PTAPI PassThruStartPeriodicMsg(uint32_t ChannelID, PASSTHRU_MSG *pMsg,
									uint32_t *pMsgID, uint32_t timeInterval) {
	log_trace("PassThruStartPeriodicMsg, ChannelID: %lu, Interval: %lu msec", ChannelID, timeInterval);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	last_error("PassThruStartPeriodicMsg is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Stop a periodic message.
 */
uint32_t PTAPI PassThruStopPeriodicMsg(uint32_t ChannelID, uint32_t msgID) {
	log_trace("PassThruStopPeriodicMsg, ChannelID: %lu", ChannelID, msgID);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;
	last_error("PassThruStopPeriodicMsg is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Start filtering incoming messages on a protocol channel.
 */
uint32_t PTAPI PassThruStartMsgFilter(uint32_t ChannelID, uint32_t FilterType, PASSTHRU_MSG *pMaskMsg,
								  PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, uint32_t *pMsgID) {
	log_trace("PassThruStartMsgFilter: ChannelID: %lu, FilterType: %lu, \n pMaskMsg: %s \n pPatternMsg: %s \n pFlowControlMsg: %s",
			  ChannelID, FilterType, parsemsg(pMaskMsg), parsemsg(pPatternMsg), parsemsg(pFlowControlMsg));
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	if (pMaskMsg == NULL || pPatternMsg == NULL) {
		last_error("Error: PASSTHRU_MSG* must not be NULL");
		return ERR_NULL_PARAMETER;
	}
	if (pMaskMsg->DataSize > 12 || pPatternMsg->DataSize > 12) {
		last_error("Error: PASSTHRU_MSG invalid data length");
		return ERR_INVALID_MSG;
	}
	if (pMaskMsg->DataSize != pPatternMsg->DataSize) {
		last_error("Error: Mask and Pattern have different data lengths");
		return ERR_INVALID_MSG;
	}
	if (pMaskMsg->TxFlags != pPatternMsg->TxFlags) {
		last_error("Error: Mask and Pattern have different TX flags");
		return ERR_INVALID_MSG;
	}
	if ((FilterType == PASS_FILTER || FilterType == BLOCK_FILTER) && pFlowControlMsg) {
		last_error("Error: FilterType, FlowControlMsg mismatch");
		return ERR_INVALID_MSG;
	}

	uint32_t err = ERR_NOT_SUPPORTED;
	uint8_t buff[48] = { 0 };
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_filter, .data_len = 44};
	uint32_t size = pMaskMsg->DataSize;
	memcpy(buff, &ChannelID, sizeof(ChannelID));
	memcpy(buff + 4, &pMaskMsg->TxFlags, 8);
	memcpy(buff + 8, &pMaskMsg->ProtocolID, 2);
	buff[10] = (uint8_t)FilterType;
	buff[11] = size;

	for (uint8_t i = 0; i < size; i++) {
		buff[12 + i] = pMaskMsg->Data[size - 1 - i];
		buff[24 + i] = pPatternMsg->Data[size - 1 - i];
	}

	if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
		last_error("PassThruWriteMsgs: Tx USB failed");
		return ERR_FAILED;
	}

	while (!do_exit & (resp_packet.cmd_code != cmd_j2534_filter))
		semaphore_take(wait_for_response_semaphore);

	memcpy(&err, resp_packet.data, sizeof(err));
	*pMsgID = resp_packet.data[4];

	log_trace("PassThruStartMsgFilter err: %lu, filterID: %lu", err, *pMsgID);
	return err;
}

/*
 * Stops filtering incoming messages on a protocol channel.
 */
uint32_t PTAPI PassThruStopMsgFilter(uint32_t ChannelID, uint32_t msgID) {
	log_trace("PassThruStopMsgFilter: ChannelID: lu, msgID: %lu", ChannelID, msgID);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	last_error("PassThruStopMsgFilter is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Set a programming voltage on a specific pin.
 */
uint32_t PTAPI PassThruSetProgrammingVoltage(uint32_t DeviceID, uint32_t PinNumber, uint32_t Voltage) {
	log_trace("PassThruSetProgrammingVoltage: DeviceID: %lu, pinNumber: %lu, voltage: %lu", DeviceID, PinNumber, Voltage);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	last_error("PassThruSetProgrammingVoltage is not implemented");
	return ERR_NOT_SUPPORTED;
}

/*
 * Reads the version information of the firmware, this DLL and API implmentation.
 */
uint32_t PTAPI PassThruReadVersion(uint32_t DeviceID, char *pFirmwareVersion, char *pDllVersion, char *pApiVersion) {
	log_trace("PassThruReadVersion: DeviceID: %lu", DeviceID);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	if (pFirmwareVersion == NULL || pDllVersion == NULL || pApiVersion == NULL) {
		last_error("Error: Version* must not be NULL");
		return ERR_NULL_PARAMETER;
	}

	char dll_ver[MAX_LEN];

#if defined(LIBUSB_API_VERSION) || defined(LIBUSBX_API_VERSION) || defined(LIBUSB1010)
	const struct libusb_version *ver = libusb_get_version();
	_snprintf_s(dll_ver, MAX_LEN, MAX_LEN, "%s (libusb-%u.%u.%u.%u%s)", J2534_DLL_VERSION,
		ver->major, ver->minor, ver->micro, ver->nano, ver->rc);
#else
	snprintf(dll_ver, MAX_LEN, "%s", DLL_VERSION);
#endif

	char fw_version[MAX_LEN];
	packet_t version_req = {
			.cmd_code = cmd_j2534_misc,
			.data_len = 0,
			.term = cmd_j2534_ack
	};

	if (usb_send_packet(version_req, WQCAN_TIMEOUT)) {
		last_error("PassThruReadVersion: Tx USB failed");
		return ERR_FAILED;
	}

	while (resp_packet.cmd_code != cmd_j2534_misc)
		semaphore_take(wait_for_response_semaphore);

	_snprintf_s(fw_version, MAX_LEN, MAX_LEN, "STM32CAN v%d.%d", resp_packet.data[0], resp_packet.data[1]);

	if (fw_version[0] != 0)
		strcpy_s(pFirmwareVersion, sizeof(fw_version), fw_version);
	else 
		strcpy_s(pFirmwareVersion, 11, "unavailable");

	strcpy_s(pDllVersion, 5, J2534_DLL_VERSION);
	strcpy_s(pApiVersion, 5, J2534_APIVER_NOVEMBER_2004);

	return STATUS_NOERROR;
}

/*
 * Gets the text description of the last error.
 */
uint32_t PTAPI PassThruGetLastError(char *pErrorDescription) {
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	if (pErrorDescription == NULL)
		return ERR_NULL_PARAMETER;

	memcpy(pErrorDescription, getLastError(), MAX_LEN);
	return STATUS_NOERROR;
}

/*
 * General I/O control functions for reading and writing
 * protocol configuration parameters (e.g. initialization,
 * baud rates, programming voltages, etc.).
 */
uint32_t PTAPI PassThruIoctl(uint32_t ChannelID, uint32_t ioctlID, void *pInput, void *pOutput) {
	log_trace("PassThruIoctl ChannelID: %lu, ioctlID: 0x%2X", ChannelID, ioctlID);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	uint32_t err = ERR_NOT_SUPPORTED;
	uint8_t buff[24] = { 0 };
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_ioctl};
	memcpy(buff, &ChannelID, sizeof(ChannelID));
	memcpy(buff + 4, &ioctlID, sizeof(ioctlID));

	switch(ioctlID) {
		case GET_CONFIG:
			//const SCONFIG_LIST *inputlist = pInput;
			last_error("PassThruIoctl ioctlID: %x is not implemented", ioctlID);
			break;
		case SET_CONFIG:
			last_error("PassThruIoctl ioctlID: %x is not implemented", ioctlID);
			break;
		case READ_VBATT:
			txPacket.data_len = 8;
			uint16_t voltage = 0;
			if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
				last_error("PassThruIoctl: Tx USB failed");
				return ERR_FAILED;
			}

			while (!do_exit & (resp_packet.cmd_code != cmd_j2534_ioctl))
				semaphore_take(wait_for_response_semaphore);

			memcpy(&err, resp_packet.data, sizeof(err));
			memcpy(&voltage, resp_packet.data + sizeof(err), sizeof(voltage));
			log_trace("Voltage: %d", voltage);
			*(uint32_t*)pOutput = voltage;
			err = STATUS_NOERROR;
			break;
		case FIVE_BAUD_INIT:
		case FAST_INIT:
		case CLEAR_TX_BUFFER:
			log_trace("Clear TX queue");
			err = STATUS_NOERROR;
			break;
		case CLEAR_RX_BUFFER:
			log_trace("Clear RX queue");
			clearQueue();
			err = STATUS_NOERROR;
			break;
		case CLEAR_PERIODIC_MSGS:
		case CLEAR_MSG_FILTERS:
			log_trace("Clear Msg filters");
			txPacket.data_len = 8;
			if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
				last_error("PassThruIoctl: Tx USB failed");
				return ERR_FAILED;
			}

			while (!do_exit & (resp_packet.cmd_code != cmd_j2534_ioctl))
				semaphore_take(wait_for_response_semaphore);

			memcpy(&err, resp_packet.data, sizeof(err));
			break;
		case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
		case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
		case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
		case READ_PROG_VOLTAGE:
			last_error("PassThruIoctl ioctlID: 0x%2X is not implemented", ioctlID);
			break;
		default:
			last_error("PassThruIoctl, unknown ioctlID: %2X", ioctlID);
	}
	log_trace("PassThruIoctl err %x", err);
	return err;
}
