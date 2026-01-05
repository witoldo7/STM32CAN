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
#include "j2534translate.h"

#define DEVICE_ID           12345
#define WQCAN_VENDOR_ID	    0xFFFF                      /* USB vendor ID used by the device */
#define WQCAN_PRODUCT_ID    0x0005                      /* USB product ID used by the device */
#define WQCAN_USB_EP_IN	    (LIBUSB_ENDPOINT_IN  | 2)   /* endpoint address */
#define WQCAN_USB_EP_OUT    (LIBUSB_ENDPOINT_OUT | 2)   /* endpoint address */
#define WQCAN_TIMEOUT	    100                         /* Connection timeout (in ms) */
#define ASYNC_TRANSFERS     4

static libusb_device_handle *handle;
static uint8_t databuf[ASYNC_TRANSFERS][128];
static struct libusb_transfer *data_transfer[ASYNC_TRANSFERS] = {NULL};
static bool isOpen = false;

uint8_t data[32];
packet_t tx_packet = {.data = data};
uint8_t rspdata[128];
packet_t resp_packet = {.data = rspdata};
sig_atomic_t do_exit = 0;
semaphore_t wait_for_response_semaphore, wTxSem, txSem, rxSem, slock;
thread_t poll_thread;
uint32_t rx_counter=0;
uint32_t tx_counter=0;
libusb_transfer_cb_fn transfer[4] = {NULL};

static void request_exit() {
	int r;
	do_exit = 1;
	for (uint8_t i = 0; i < ASYNC_TRANSFERS; i++) {
		libusb_cancel_transfer(data_transfer[i]);
	}
	semaphore_destroy(wait_for_response_semaphore);
	semaphore_destroy(rxSem);
	semaphore_destroy(txSem);
	semaphore_destroy(wTxSem);
	semaphore_destroy(slock);
	log_trace("poll thread shutting down");
	thread_join(poll_thread);
	if (handle) {
		r = libusb_release_interface(handle, 1);
		log_trace("libusb_release_interface: %s", libusb_error_name(r));
	}
	if (handle)
		libusb_close(handle);
	libusb_exit(NULL);
	handle = NULL;
	for (uint8_t i = 0; i < ASYNC_TRANSFERS; i++) {
		data_transfer[i] = NULL;
	}
	isOpen = false;
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
			log_error("poll thread exit : %s", libusb_error_name(r));
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

static void LIBUSB_CALL cb_process(struct libusb_transfer *transfer, struct libusb_transfer *data) {
	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		log_warn("transfer status: %d", transfer->status);
		goto err_free_transfer;
	}

	uint16_t data_len = (uint16_t)((transfer->buffer[1] & 0xffffU) << 8) | (uint16_t)transfer->buffer[2];

	if ((data_len + 4) != transfer->actual_length) {
		log_error("Packet incomplete, transfer size: %d, packet size %d", transfer->actual_length, (data_len + 4));
		if (libusb_submit_transfer(data) < 0)
			goto err_free_transfer;
		return;
	}
    //log_trace("usb_recv_packet cmd: %02x", transfer->buffer[0]);
	switch (transfer->buffer[0]) {
	case cmd_j2534_connect:
	case cmd_j2534_disconnect:
	case cmd_j2534_ioctl:
	case cmd_j2534_filter:
	case cmd_j2534_start_periodic_message:
	case cmd_j2534_stop_periodic_message:
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
		semaphore_give(wTxSem);
		break;
	case cmd_j2534_read_message:
		PASSTHRU_MSG pMsg = { 0 };
		convertPacketToPMSG(transfer->buffer +3, data_len, &pMsg);
		enQueue(pMsg);
		break;
	default:
		log_error("unsupported cmd: 0x%x", transfer->buffer[0]);
		break;
	}

	if (libusb_submit_transfer(data) < 0)
		goto err_free_transfer;

	return;

err_free_transfer:
	log_warn("free_transfer");
	if (data != NULL) {
		libusb_free_transfer(data);
		data = NULL;
	}
}

static void LIBUSB_CALL cb_data(struct libusb_transfer *transfer) {
	cb_process(transfer, data_transfer[0]);
}
static void LIBUSB_CALL cb_data1(struct libusb_transfer *transfer) {
	cb_process(transfer, data_transfer[1]);
}
static void LIBUSB_CALL cb_data2(struct libusb_transfer *transfer) {
	cb_process(transfer, data_transfer[2]);
}
static void LIBUSB_CALL cb_data3(struct libusb_transfer *transfer) {
	cb_process(transfer, data_transfer[3]);
}

static int alloc_transfers(void) {
	transfer[0] = cb_data;
	transfer[1] = cb_data1;
	transfer[2] = cb_data2;
	transfer[3] = cb_data3;
	for (uint8_t i = 0; i < ASYNC_TRANSFERS; i++) {
		data_transfer[i] = libusb_alloc_transfer(0);
		if (!data_transfer) {
			errno = ENOMEM;
			return -1;
		}

		libusb_fill_bulk_transfer(data_transfer[i], handle, WQCAN_USB_EP_IN, databuf[i],
			sizeof(databuf)/ASYNC_TRANSFERS, transfer[i], NULL, 0);
	}
	return 0;
}

static int usb_send_packet(packet_t packet, unsigned int timeout) {
	int bytes_written = 0, r = LIBUSB_ERROR_OTHER;
	uint8_t buff[128] ={0};
	uint16_t len = covertPacketToBuffer(&packet, buff);
	if (len > 0) {
		r = libusb_bulk_transfer(handle, WQCAN_USB_EP_OUT, buff, (int)len, &bytes_written, 0);
		//log_trace("usb_send_packet cmd: %02x", buff[0]);
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
	*pDeviceID = DEVICE_ID;
	if (isOpen) {
		log_trace("PassThruOpen: already opened");
		return STATUS_NOERROR;
	}
	check_debug_log();
    do_exit = 0;
	log_trace("PassThruOpen: Device Name: %s, ID: %lu", (char*)pName, *pDeviceID);
	libusb_init(NULL);
//  libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, 4);

    //Open Device with VendorID and ProductID
	handle = libusb_open_device_with_vid_pid(NULL, WQCAN_VENDOR_ID, WQCAN_PRODUCT_ID);
	if (!handle) {
		last_error("device not found");
		libusb_exit(NULL);
		return ERR_DEVICE_NOT_CONNECTED;
	}
	int r = 1;
    if (libusb_kernel_driver_active(handle, 1) == 1) {
        r = libusb_detach_kernel_driver(handle, 1);
    }

	//Claim Interface 1 from the device
    r = libusb_claim_interface(handle, 1);
	if (r < 0) {
		last_error("usb_claim_interface error %s", libusb_error_name(r));
		libusb_close(handle);
		libusb_exit(NULL);
		return ERR_DEVICE_NOT_CONNECTED;
	}

	r = alloc_transfers();
	if (r < 0) {
		last_error("alloc_transfers error %s", libusb_error_name(r));
		goto out_deinit;
	}

	setup_signals();

	wait_for_response_semaphore = semaphore_create("resp");
	wTxSem = semaphore_create("wTxSem");
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
	for (uint8_t i = 0; i < ASYNC_TRANSFERS; i++) {
		r = libusb_submit_transfer(data_transfer[i]);
		if (r < 0) {
			last_error("libusb_submit_transfer error %s", libusb_error_name(r));
			goto out_deinit;
		}
	}
	last_error("PassThruOpen Interface claimed");
	isOpen = true;
	return STATUS_NOERROR;

out_deinit:
	for (uint8_t i = 0; i < ASYNC_TRANSFERS; i++) {
		if (data_transfer[i])
			libusb_free_transfer(data_transfer[i]);
	}

	return ERR_FAILED;
}

/*
 * Terminate a connection with a Pass-Thru device.
 */
uint32_t PTAPI PassThruClose(uint32_t DeviceID) {
	log_trace("PassThruClose: DeviceID:  %lu", DeviceID);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	request_exit();
	log_trace("PassThruClose");

	return STATUS_NOERROR;
}

/*
 * Establish a connection using a protocol channel.
 */
uint32_t PTAPI PassThruConnect(uint32_t DeviceID, uint32_t protocolID,
						   uint32_t flags, uint32_t baud, uint32_t *pChannelID) {
	log_trace("PassThruConnect: DeviceID: %lu, protocolID: %s, flags: %08lX, baud: %lu",
			  DeviceID, translateProtocol(protocolID), flags, baud);
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
	log_trace("PassThruConnect: cid: %lu, err: %s", *pChannelID, translateError(err));
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

	log_trace("PassThruDisconnect: err: %s", translateError(err));
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
uint32_t PTAPI PassThruReadMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout) {
	Lock(rxSem, rx_counter);
	log_trace("PassThruReadMsgs: ChannelID:%lu, Timeout: %u msec, numMsg: %lu", ChannelID, Timeout, *pNumMsgs);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;
/*
	uint8_t buff[16] = { 0 };
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_read_message};
	uint8_t offset = 0;
	memcpy(buff, &ChannelID, sizeof(ChannelID));
	offset += 4;
	memcpy(buff + offset, &Timeout, sizeof(Timeout));
	offset += 4;
	memcpy(buff + offset, pNumMsgs, sizeof(uint32_t));
	offset += 4;
	txPacket.data_len = offset;
	if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
		last_error("PassThruReadMsgs: Tx USB failed");
		return ERR_FAILED;
	}
*/
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
		    sleep_us(10);
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
	log_trace("PassThruReadMsgs err: %s", translateError(err));
	Unlock(rxSem, rx_counter);
	return err;
}

/*
 * Write message(s) to a protocol channel.
 */
uint32_t PTAPI PassThruWriteMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout) {
	Lock(txSem, tx_counter);
	log_trace("PassThruWriteMsgs: ChannelID: %lu, NumMsg: %lu, Timeout: %lu msec\r\n MSG: %s", ChannelID, *pNumMsgs, Timeout, parsemsg(pMsg));
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
	memcpy(buff + offset, &Timeout, sizeof(Timeout));
	offset += 4;

	for (uint8_t i = 0; i < *pNumMsgs; i++) {
		switch (ChannelID) {
			case CAN:
			case CAN_PS:
			case SW_CAN_PS:
			case ISO15765:
				CANTxFrame tx = {0};
				PASSTHRU_MSG_To_CANTxFrame(&pMsg[i], &tx);
				memcpy(buff + offset, &tx, 8 + tx.DLC);
				txPacket.data_len = offset + 8 + tx.DLC;
				break;
			case ISO14230:
			case ISO9141:
				memcpy(buff + offset, &pMsg[i].Data, pMsg[i].DataSize);
				txPacket.data_len = offset + pMsg[i].DataSize;
				break;
			default:
				break;
		}

		if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
			last_error("PassThruWriteMsgs: Tx USB failed");
			return ERR_FAILED;
		}

		if(semaphore_take_timeout(wTxSem, 1000) == -1) {
			return ERR_TIMEOUT;
		}
		sleep_us(150);

		memcpy(&err, tx_packet.data, sizeof(err));
	}

	log_trace("PassThruWriteMsgs err: %s", translateError(err));
	Unlock(txSem, tx_counter);
	return err;
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
								  PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, uint32_t *pFilterID) {
	char msgMaskBuff[512] = {0};
	char msgPatternBuff[512] = {0};

	log_trace("PassThruStartMsgFilter: ChannelID: %lu, FilterType: %s, \n pMaskMsg: %s \n pPatternMsg: %s \n pFlowControlMsg: %s",
			  ChannelID, translateFilterType(FilterType), parsemsgb(pMaskMsg, msgMaskBuff), parsemsgb(pPatternMsg, msgPatternBuff), parsemsg(pFlowControlMsg));
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
	memcpy(buff + 4, &pMaskMsg->TxFlags, 4);
	memcpy(buff + 8, &pMaskMsg->ProtocolID, 2);
	buff[10] = (uint8_t)FilterType;
	buff[11] = size;

	for (uint8_t i = 0; i < size; i++) {
		buff[12 + i] = pMaskMsg->Data[size - 1 - i];
		buff[24 + i] = pPatternMsg->Data[size - 1 - i];
	}

	if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
		last_error("PassThruStartMsgFilter: Tx USB failed");
		return ERR_FAILED;
	}

	while (!do_exit & (resp_packet.cmd_code != cmd_j2534_filter))
		semaphore_take(wait_for_response_semaphore);

	memcpy(&err, resp_packet.data, sizeof(err));
	*pFilterID = resp_packet.data[4];

	log_trace("PassThruStartMsgFilter err: %s, filterID: %lu", translateError(err), *pFilterID);
	return err;
}

/*
 * Stops filtering incoming messages on a protocol channel.
 */
uint32_t PTAPI PassThruStopMsgFilter(uint32_t ChannelID, uint32_t FilterID) {
	log_trace("PassThruStopMsgFilter: ChannelID: lu, filterID: %lu", ChannelID, FilterID);
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	uint32_t err = ERR_NOT_SUPPORTED;
	uint8_t buff[8] = { 0 };
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_stop_filter, .data_len = 8};
	memcpy(buff, &ChannelID, sizeof(ChannelID));
	memcpy(buff + sizeof(ChannelID), &FilterID, sizeof(FilterID));

	if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
		last_error("PassThruStopMsgFilter: Tx USB failed");
		return ERR_FAILED;
	}

	while (!do_exit & (resp_packet.cmd_code != cmd_j2534_stop_filter))
		semaphore_take(wait_for_response_semaphore);

	memcpy(&err, resp_packet.data, sizeof(err));

	last_error("PassThruStopMsgFilter err: %s", translateError(err));
	return err;
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
			.term = cmd_term_ack
	};

	if (usb_send_packet(version_req, WQCAN_TIMEOUT)) {
		last_error("PassThruReadVersion: Tx USB failed");
		return ERR_FAILED;
	}
	while (resp_packet.cmd_code != cmd_j2534_misc)
		semaphore_take(wait_for_response_semaphore);
	_snprintf_s(fw_version, MAX_LEN, MAX_LEN, "STM32CAN v%d.%d", resp_packet.data[0], resp_packet.data[1]);
	if (fw_version[0] != 0)
		strcpy(pFirmwareVersion, fw_version);
	else 
		strcpy(pFirmwareVersion, "unavailable");

	strcpy(pDllVersion, J2534_DLL_VERSION);
	strcpy(pApiVersion, J2534_APIVER_NOVEMBER_2004);

	log_trace("PassThruReadVersion: pFirmwareVersion: %s, pDllVersion: %s pApiVersion: %s", pFirmwareVersion, pDllVersion, pApiVersion);
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
	log_trace("PassThruIoctl ChannelID: %lu, ioctlID: %x ( %s ) ", ChannelID, ioctlID, translateIoctl(ioctlID));
	if (!isOpen)
		return ERR_INVALID_DEVICE_ID;

	uint32_t err = ERR_NOT_SUPPORTED;
	uint8_t buff[256] = { 0 };
	packet_t txPacket = {.data = buff, .cmd_code = cmd_j2534_ioctl};
	uint8_t offset = 0;
	memcpy(buff + offset, &ChannelID, sizeof(ChannelID));
	offset += sizeof(ChannelID);
	memcpy(buff + offset, &ioctlID, sizeof(ioctlID));
	offset += sizeof(ioctlID);

	switch(ioctlID) {
		case GET_CONFIG:
			const SCONFIG_LIST* inputlist = pInput;
			memcpy(buff + offset, &inputlist->NumOfParams, sizeof(inputlist->NumOfParams));
			offset += sizeof(inputlist->NumOfParams);
			uint8_t gsize = sizeof(SCONFIG) * inputlist->NumOfParams;
			memcpy(buff + offset, inputlist->ConfigPtr, gsize);

			txPacket.data_len = offset + gsize;
			if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
				last_error("PassThruIoctl: Tx USB failed");
				return ERR_FAILED;
			}

			while (!do_exit & (resp_packet.cmd_code != cmd_j2534_ioctl))
				semaphore_take(wait_for_response_semaphore);

			memcpy(&err, resp_packet.data, sizeof(err));

			if (err != STATUS_NOERROR) {
				for (uint8_t i = 0; i < inputlist->NumOfParams; i++) {
					SCONFIG* cfg = &inputlist->ConfigPtr[i];
					log_trace("GET_CONFIG: num: %d, parameter: %x ( %s )", i, cfg->Parameter, translateParam(cfg->Parameter));
				}
				log_trace("Received only error: %x", err);
				return err;
			}

			memcpy(inputlist->ConfigPtr, resp_packet.data + 8, gsize);


			for (uint8_t i = 0; i < inputlist->NumOfParams; i++) {
				SCONFIG* cfg = &inputlist->ConfigPtr[i];
				log_trace("GET_CONFIG: num: %d, parameter: %s value %x", i, translateParam(cfg->Parameter), cfg->Value);
			}
			break;
		case SET_CONFIG:
			const SCONFIG_LIST* list = pInput;
			for (uint8_t i = 0; i < list->NumOfParams; i++) {
				SCONFIG *cfg = &list->ConfigPtr[i];
				log_trace("SET_CONFIG: num: %d, parameter: %s value %x", i, translateParam(cfg->Parameter), cfg->Value);
			}
			memcpy(buff + offset, &list->NumOfParams, sizeof(list->NumOfParams));
			offset += sizeof(list->NumOfParams);
			uint8_t size = sizeof(SCONFIG) * list->NumOfParams;
			memcpy(buff + offset, list->ConfigPtr, size);
			txPacket.data_len = offset + size;

			if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
				last_error("PassThruIoctl: Tx USB failed");
				return ERR_FAILED;
			}

			while (!do_exit & (resp_packet.cmd_code != cmd_j2534_ioctl))
				semaphore_take(wait_for_response_semaphore);

			memcpy(&err, resp_packet.data, sizeof(err));
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
			break;
		case CLEAR_TX_BUFFER:
			err = STATUS_NOERROR;
			break;
		case CLEAR_RX_BUFFER:
			clearQueue();
			err = STATUS_NOERROR;
			break;
		case CLEAR_MSG_FILTERS:
			txPacket.data_len = 8;
			if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
				last_error("PassThruIoctl: Tx USB failed");
				return ERR_FAILED;
			}

			while (!do_exit & (resp_packet.cmd_code != cmd_j2534_ioctl))
				semaphore_take(wait_for_response_semaphore);

			memcpy(&err, resp_packet.data, sizeof(err));
			break;
		case FIVE_BAUD_INIT:
		{
			const SBYTE_ARRAY* init = pInput;
			uint8_t dataSize = (uint8_t)init->NumOfBytes;
			memcpy(buff + offset, &dataSize, 1);
			offset += 1;
			memcpy(buff + offset, init->BytePtr, dataSize);
			txPacket.data_len = offset + dataSize;

			if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
				last_error("PassThruIoctl: Tx USB failed");
				return ERR_FAILED;
			}

			while (!do_exit & (resp_packet.cmd_code != cmd_j2534_ioctl))
				semaphore_take(wait_for_response_semaphore);

			memcpy(&err, resp_packet.data, sizeof(err));
			if (resp_packet.data_len < 5)
				return err;

			uint8_t outbuff[32] = {0};
			SBYTE_ARRAY outMsg = {.BytePtr = outbuff, .NumOfBytes = resp_packet.data[4]};
			memcpy(outMsg.BytePtr, resp_packet.data + 5, outMsg.NumOfBytes);
			*(SBYTE_ARRAY*)pOutput = outMsg;
			break;
		}
		case FAST_INIT:
			const PASSTHRU_MSG* initMSG = pInput;
			log_trace("%s", parsemsg(pInput));
			uint8_t dataSize = (uint8_t)initMSG->DataSize;
			memcpy(buff + offset, &dataSize, 1);
			offset += 1;
			memcpy(buff + offset, initMSG->Data, dataSize);
			txPacket.data_len = offset + dataSize;

			if (usb_send_packet(txPacket, WQCAN_TIMEOUT)) {
				last_error("PassThruIoctl: Tx USB failed");
				return ERR_FAILED;
			}

			while (!do_exit & (resp_packet.cmd_code != cmd_j2534_ioctl))
				semaphore_take(wait_for_response_semaphore);

			memcpy(&err, resp_packet.data, sizeof(err));
			if (resp_packet.data_len < 5) {
				log_trace("resp len: %d\r\n", resp_packet.data_len);
				break;
			}

			PASSTHRU_MSG msg = {0};
			msg.ProtocolID = initMSG->ProtocolID;
			msg.DataSize = resp_packet.data[4];
			memcpy(msg.Data, resp_packet.data + 5, msg.DataSize);
			*(PASSTHRU_MSG*)pOutput = msg;
			log_trace("%s", parsemsg(&msg));
			clearQueue();
			break;
		case CLEAR_PERIODIC_MSGS:
		case CLEAR_FUNCT_MSG_LOOKUP_TABLE:
		case ADD_TO_FUNCT_MSG_LOOKUP_TABLE:
		case DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE:
		case READ_PROG_VOLTAGE:
			last_error("PassThruIoctl ioctlID: %s is not implemented", translateIoctl(ioctlID));
			break;
		case GET_DEVICE_INFO:
		    SPARAM_LIST* paramList = pOutput;
			log_trace("PassThruIoctl GET_DEVICE_INFO: param_num %d, param %x", paramList->NumOfParams, paramList->SParamPtr[0].Parameter);
			err = STATUS_NOERROR;
			switch (paramList->SParamPtr[0].Parameter) {
			case SERIAL_NUMBER:
				paramList->SParamPtr[0].Value = 12121212;
				paramList->SParamPtr[0].Supported = 1;
				break;
			case SHORT_TO_GND_J1962:
				paramList->SParamPtr[0].Value = 0xF;
				paramList->SParamPtr[0].Supported = 1;
				break;
			case PGM_VOLTAGE_J1962:
				paramList->SParamPtr[0].Value = 0xF;
				paramList->SParamPtr[0].Supported = 1;
				break;
			default:
				err = ERR_NOT_SUPPORTED;
				break;
			}

			break;
			
		default:
			last_error("PassThruIoctl, unknown ioctlID: 0x%X", ioctlID);
	}
	log_trace("PassThruIoctl err: %s", translateError(err));
	return err;
}

uint32_t PTAPI PassThruGetNextCarDAQ(char **name, unsigned long *version, char **addr) {
	*name = "WQCAN";
	*version = 2323;
	addr = NULL;
	return STATUS_NOERROR;
}