#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include "combi.h"
#include "usbcfg.h"

static semaphore_t rxSem;
static semaphore_t processSem;
uint8_t receiveBuf[OUT_PACKETSIZE];
uint8_t transferBuf[IN_PACKETSIZE];
packet_t rx_packet = {};
packet_t tx_packet = {};
volatile bool is_transmiting = false;

void usb_send(USBDriver *usbp, usbep_t ep, const uint8_t *buf, size_t n) {
	while (is_transmiting) {
		chThdSleepMicroseconds(1);
	}
	is_transmiting = true;
	osalSysLock();
	usbStartTransmitI(usbp, ep, buf, n);
	osalSysUnlock();
}

static bool start_receive(USBDriver *usbp, usbep_t ep, uint8_t *buf, size_t n) {
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
 * data Transmitted Callback
 */
void dataTransmitted(USBDriver *usbp, usbep_t ep){
    (void) usbp;
    (void) ep;
    is_transmiting = false;
}

/*
 * data Received Callback
 */
void dataReceived(USBDriver *usbp, usbep_t ep){
    (void) usbp;
    (void) ep;
    chSysLockFromISR();
    chSemSignalI(&rxSem);
    chSysUnlockFromISR();
}

static THD_WORKING_AREA(usb_rx_wa, 2048);
static THD_FUNCTION(usb_rx, arg) {
	(void) arg;
	uint8_t buff[16] = {0};
	rx_packet.data = buff;
	chRegSetThreadName("usbReceiver");
	while (TRUE){
		chSemWait(&rxSem);
		if (!(((USBDriver *)&USBD1)->state == USB_ACTIVE)) {
			continue;
		}
		if (*(receiveBuf) != 0) {
			rx_packet.cmd_code = receiveBuf[0];
			uint16_t len = (uint16_t)((receiveBuf[1] & 0xffffU) << 8) | (uint16_t)receiveBuf[2];
			rx_packet.data_len = len;
		    memcpy(rx_packet.data, receiveBuf + 3, len);
			rx_packet.term = receiveBuf[len + 3];
			osalSysLock();
			chSemSignalI(&processSem);
			osalSysUnlock();
	    }
		start_receive(&USBD1, EP_OUT, receiveBuf, IN_PACKETSIZE);
	}
}

static THD_WORKING_AREA(combi_wa, 4096);
static THD_FUNCTION(combi, arg) {
	(void) arg;
	bool ret = false;
	uint8_t size = 0;
	chRegSetThreadName("combi");
	while (TRUE){
		chSemWait(&processSem);
		if (rx_packet.term == cmd_term_ack) {
			switch (rx_packet.cmd_code & 0xe0) {
				case 0x20: //Board utility.
					ret = exec_cmd_board(&rx_packet, &tx_packet);
					break;
				case 0x40: //BDM utility.
					ret = exec_cmd_bdm(&rx_packet, &tx_packet);
					break;
				case 0x80:  //CAN utility.
					ret = exec_cmd_can(&rx_packet, &tx_packet);
					break;
				default:
					continue;
			}
			if (ret != true) {
				CombiSendReplyPacket(&tx_packet, &rx_packet, 0, 0, cmd_term_nack);
			}

			size = CombiSendPacket(&tx_packet, transferBuf);
			usb_send(&USBD1, EP_IN, transferBuf, size);
		}
	}
}

static THD_WORKING_AREA(can_rx_wa, 4096);
static THD_FUNCTION(can_rx, p) {
	(void)p;
	event_listener_t el;
	CANRxFrame rxmsg = {};
	uint8_t size = 0;
	uint8_t buffer[IN_PACKETSIZE] = {0};
	uint8_t packetbuff[16] = {0};
	packet_t tx_packet = {.data = packetbuff, .cmd_code = cmd_can_frame, .data_len = 15};
	chRegSetThreadName("can receiver");
	chEvtRegister(&CAND1.rxfull_event, &el, 0);
	while (true) {
		if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0) {
			continue;
		}
		while (canReceive(&CAND1, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE) == MSG_OK) {
			packetbuff[0] = rxmsg.SID & 0xFF;
			packetbuff[1] = (rxmsg.SID >> 8) & 0xFF;
			packetbuff[2] = (rxmsg.SID >> 16) & 0xFF;
			packetbuff[3] = (rxmsg.SID >> 24) & 0xFF;
			packetbuff[12] = rxmsg.SID;
			memcpy(packetbuff + 4, rxmsg.data8, rxmsg.DLC);
			size = CombiSendPacket(&tx_packet, buffer);
			usb_send(&USBD1, EP_IN, buffer, size);
		}
	}
	chEvtUnregister(&CAND1.rxfull_event, &el);
}

int main(void) {
	halInit();
	chSysInit();
	chSemObjectInit(&rxSem, 1);
	chSemObjectInit(&processSem, 1);

	usbDisconnectBus(&USBD1);
	chThdSleepMilliseconds(100);
	usbStart(&USBD1, &usb_config);
	usbConnectBus(&USBD1);

	chThdCreateStatic(usb_rx_wa, sizeof(usb_rx_wa), NORMALPRIO + 7, usb_rx, NULL);
	chThdCreateStatic(combi_wa, sizeof(combi_wa), NORMALPRIO + 7, combi, NULL);
	chThdCreateStatic(can_rx_wa, sizeof(can_rx_wa), NORMALPRIO + 20, can_rx, NULL);

	bool usbinit = false; //fixme
	while (TRUE) {

		while(is_ready() & usbinit){
			chThdSleepMilliseconds(100);
		}
		chThdSleepMilliseconds(100);
		start_receive(&USBD1, EP_OUT, receiveBuf, IN_PACKETSIZE);
		//usbinit = true;
	}
}
