// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
 */

#include <stdint.h>
#include "j2534def.h"

char* translateError(uint8_t err) {
	switch (err) {
	case 0x00: return "STATUS_NOERROR";
	case 0x01: return "ERR_NOT_SUPPORTED";
	case 0x02: return "ERR_INVALID_CHANNEL_ID";
	case 0x03: return "ERR_INVALID_PROTOCOL_ID";
	case 0x04: return "ERR_NULL_PARAMETER";
	case 0x05: return "ERR_INVALID_IOCTL_VALUE";
	case 0x06: return "ERR_INVALID_FLAGS";
	case 0x07: return "ERR_FAILED";
	case 0x08: return "ERR_DEVICE_NOT_CONNECTED";
	case 0x09: return "ERR_TIMEOUT";
	case 0x0A: return "ERR_INVALID_MSG";
	case 0x0B: return "ERR_INVALID_TIME_INTERVAL";
	case 0x0C: return "ERR_EXCEEDED_LIMIT";
	case 0x0D: return "ERR_INVALID_MSG_ID";
	case 0x0E: return "ERR_DEVICE_IN_USE";
	case 0x0F: return "ERR_INVALID_IOCTL_ID";
	case 0x10: return "ERR_BUFFER_EMPTY";
	case 0x11: return "ERR_BUFFER_FULL";
	case 0x12: return "ERR_BUFFER_OVERFLOW";
	case 0x13: return "ERR_PIN_INVALID";
	case 0x14: return "ERR_CHANNEL_IN_USE";
	case 0x15: return "ERR_MSG_PROTOCOL_ID";
	case 0x16: return "ERR_INVALID_FILTER_ID";
	case 0x17: return "ERR_NO_FLOW_CONTROL";
	case 0x18: return "ERR_NOT_UNIQUE";
	case 0x19: return "ERR_INVALID_BAUDRATE";
	case 0x1A: return "ERR_INVALID_DEVICE_ID";
	default: return "Unknown";
	}
}

char* translateErrorDetail(uint32_t err) {
	switch (err) {
	case 0x00: return "Function call successful.";
	case 0x01: return "Device cannot support requested functionality mandated in J2534. Device is not fully SAE J2534 compliant.";
	case 0x02: return "Invalid ChannelID value.";
	case 0x03: return "Invalid or unsupported ProtocolID, or there is a resource conflict (i.e. trying to connect to multiple mutually exclusive protocols such as J1850PWM and J1850VPW, or CAN and SCI, etc.).";
	case 0x04: return "NULL pointer supplied where a valid pointer is required.";
	case 0x05: return "Invalid value for Ioctl parameter.";
	case 0x06: return "Invalid flag values.";
	case 0x07: return "Undefined error, use PassThruGetLastError() for text description.";
	case 0x08: return "Unable to communicate with device.";
	case 0x09: return "Read or write timeout.";
	case 0x0A: return "Invalid message structure pointed to by pMsg.";
	case 0x0B: return "Invalid TimeInterval value.";
	case 0x0C: return "Exceeded maximum number of message IDs or allocated space.";
	case 0x0D: return "Invalid MsgID value.";
	case 0x0E: return "Device is currently open.";
	case 0x0F: return "Invalid IoctlID value.";
	case 0x10: return "Protocol message buffer empty, no messages available to read.";
	case 0x11: return "Protocol message buffer full. All the messages specified may not have been transmitted.";
	case 0x12: return "Indicates a buffer overflow occurred and messages were lost.";
	case 0x13: return "Invalid pin number, pin number already in use, or voltage already applied to a different pin.";
	case 0x14: return "Channel number is currently connected.";
	case 0x15: return "Protocol type in the message does not match the protocol associated with the Channel ID.";
	case 0x16: return "Invalid Filter ID value.";
	case 0x17: return "No flow control filter set or matched (for ProtocolID ISO15765 only).";
	case 0x18: return "A CAN ID in pPatternMsg or pFlowControlMsg matches either ID in an existing FLOW_CONTROL_FILTER.";
	case 0x19: return "The desired baud rate cannot be achieved within the tolerance specified in SAE J2534-1 Section 6.5.";
	case 0x1A: return "Device ID invalid.";
	default: return "Unknown";
	}
}

char* translateParam(uint32_t param) {
	switch (param) {
	case 0x01: return "DATA_RATE";
	case 0x03: return "LOOPBACK";
	case 0x04: return "NODE_ADDRESS";
	case 0x05: return "NETWORK_LINE";
	case 0x06: return "P1_MIN";
	case 0x07: return "P1_MAX";
	case 0x08: return "P2_MIN";
	case 0x09: return "P2_MAX";
	case 0x0A: return "P3_MIN";
	case 0x0B: return "P3_MAX";
	case 0x0C: return "P4_MIN";
	case 0x0D: return "P4_MAX";
	case 0x19: return "W0";
	case 0x0E: return "W1";
	case 0x0F: return "W2";
	case 0x10: return "W3";
	case 0x11: return "W4";
	case 0x12: return "W5";
	case 0x13: return "TIDLE";
	case 0x14: return "TINIL";
	case 0x15: return "TWUP";
	case 0x16: return "PARITY";
	case 0x17: return "BIT_SAMPLE_POINT";
	case 0x18: return "SYNC_JUMP_WIDTH";
	case 0x1A: return "T1_MAX";
	case 0x1B: return "T2_MAX";
	case 0x24: return "T3_MAX";
	case 0x1C: return "T4_MAX";
	case 0x1D: return "T5_MAX";
	case 0x1E: return "ISO15765_BS";
	case 0x1F: return "ISO15765_STMIN";
	case 0x22: return "ISO15765_BS_TX";
	case 0x23: return "ISO15765_STMIN_TX";
	case 0x20: return "DATA_BITS";
	case 0x21: return "FIVE_BAUD_MOD";
	case 0x25: return "ISO15765_WFT_MAX";
	case 0x00008000: return "CAN_MIXED_FORMAT";
	case 0x00008001: return "J1962_PINS";
	case 0x00008010: return "SW_CAN_HS_DATA_RATE";
	case 0x00008011: return "SW_CAN_SPEEDCHANGE_ENABLE";
	case 0x00008012: return "SW_CAN_RES_SWITCH";
	case 0x00008020: return "ACTIVE_CHANNELS";
	case 0x00008021: return "SAMPLE_RATE";
	case 0x00008022: return "SAMPLES_PER_READING";
	case 0x00008023: return "READINGS_PER_MSG";
	case 0x00008024: return "AVERAGING_METHOD";
	case 0x00008025: return "SAMPLE_RESOLUTION";
	case 0x00008026: return "INPUT_RANGE_LOW";
	case 0x00008027: return "INPUT_RANGE_HIGH";
	case DT_ISO_INIT_BAUD: return "DT_ISO_INIT_BAUD";
	case DT_ISO_CHECKSUM_TYPE: return "DT_ISO_CHECKSUM_TYPE";
	default: return "Unknown";
	}
}

char* translateIoctl(uint32_t ioctl) {
	switch (ioctl) {
	case 0x01: return "GET_CONFIG";
	case 0x02: return "SET_CONFIG";
	case 0x03: return "READ_VBATT";
	case 0x04: return "FIVE_BAUD_INIT";
	case 0x05: return "FAST_INIT";
	case 0x07: return "CLEAR_TX_BUFFER";
	case 0x08: return "CLEAR_RX_BUFFER";
	case 0x09: return "CLEAR_PERIODIC_MSGS";
	case 0x0A: return "CLEAR_MSG_FILTERS";
	case 0x0B: return "CLEAR_FUNCT_MSG_LOOKUP_TABLE";
	case 0x0C: return "ADD_TO_FUNCT_MSG_LOOKUP_TABLE";
	case 0x0D: return "DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE";
	case 0x0E: return "READ_PROG_VOLTAGE";
	case 0x00008000: return "SW_CAN_HS";
	case 0x00008001: return "SW_CAN_NS";
	case 0x00008002: return "SET_POLL_RESPONSE";
	case 0x00008003: return "BECOME_MASTER";
	case GET_DEVICE_INFO: return "GET_DEVICE_INFO";
	default: return "Unknown";
	}
}

char* translateProtocol(uint32_t protocol) {
	switch (protocol) {
	case 0x1: return "J1850VPW";
	case 0x2: return "J1850PWM";
	case 0x3: return "ISO9141";
	case 0x4: return "ISO14230";
	case 0x5: return "CAN";
	case 0x6: return "ISO15765";
	case 0x7: return "SCI_A_ENGINE";
	case 0x8: return "SCI_A_TRANS";
	case 0x9: return "SCI_B_ENGINE";
	case 0xA: return "SCI_B_TRANS";
	case 0x001000C: return "ISO15765_FD_PS";
	case 0x001000D: return "CAN_FD_PS";
	case 0x0008000: return "J1850VPW_PS";
	case 0x0008001: return "J1850PWM_PS";
	case 0x0008002: return "ISO9141_PS";
	case 0x0008003: return "ISO14230_PS";
	case 0x0008004: return "CAN_PS";
	case 0x0008005: return "ISO15765_PS";
	case 0x0008006: return "J2610_PS";
	case 0x0008007: return "SW_ISO15765_PS";
	case 0x0008008: return "SW_CAN_PS";
	case 0x0008009: return "GM_UART_PS";
	case 0x0009000: return "CAN_CH1";
	case (CAN_CH1+1): return "CAN_CH2";
	case (CAN_CH1+127): return "CAN_CH128";
	case 0x0009080: return "J1850VPW_CH1";
	case (J1850VPW_CH1+1): return "J1850VPW_CH2";
	case (J1850VPW_CH1+127): return "J1850VPW_CH128";
	case 0x0009160: return "J1850PWM_CH1";
	case (J1850PWM_CH1+1): return "J1850PWM_CH2";
	case (J1850PWM_CH1+127): return "J1850PWM_CH128";
	case 0x0009240: return "ISO9141_CH1";
	case (ISO9141_CH1+1): return "ISO9141_CH2";
	case (ISO9141_CH1+127): return "ISO9141_CH128";
	case 0x0009320: return "ISO14230_CH1";
	case (ISO14230_CH1+1): return "ISO14230_CH2";
	case (ISO14230_CH1+127): return "ISO14230_CH128";
	case 0x0009400: return "ISO15765_CH1";
	case (ISO15765_CH1+1): return "ISO15765_CH2";
	case (ISO15765_CH1+127): return "ISO15765_CH128";
	case 0x0009480: return "SW_CAN_CAN_CH1";
	case (SW_CAN_CAN_CH1+1): return "SW_CAN_CAN_CH2";
	case (SW_CAN_CAN_CH1+127): return "SW_CAN_CAN_CH128";
	case 0x0009560: return "SW_CAN_ISO15765_CH1";
	case (SW_CAN_ISO15765_CH1+1): return "SW_CAN_ISO15765_CH2";
	case (SW_CAN_ISO15765_CH1+127): return "SW_CAN_ISO15765_CH128";
	case 0x0009640: return "J2610_CH1";
	case (J2610_CH1+1): return "J2610_CH2";
	case (J2610_CH1+127): return "J2610_CH128";
	case 0x000C000: return "ANALOG_IN_CH1";
	case 0x000C001: return "ANALOG_IN_CH2";
	case 0x000C01F: return "ANALOG_IN_CH32";
	default: return "Unknown";
	}
}

char* translateFilterType(uint32_t type) {
	switch (type) {
	case 0x00000001: return "PASS_FILTER";
	case 0x00000002: return "BLOCK_FILTER";
	case 0x00000003: return "FLOW_CONTROL_FILTER";
	default: return "Unknown";
	}
}

char* translate(uint32_t type) {
	switch (type) {
	default: return "Unknown";
	}
}
