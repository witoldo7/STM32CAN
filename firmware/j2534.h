/*
 * j2534.h
 *
 *  Created on: 20 lut 2023
 *      Author: witold
 */

#ifndef J2534_H_
#define J2534_H_

#include "utils.h"

//Error
#define STATUS_NOERROR                  0x00    // Function call successful.
#define ERR_NOT_SUPPORTED               0x01    // Device cannot support requested functionality mandated in J2534. Device is not fully SAE J2534 compliant.
#define ERR_INVALID_CHANNEL_ID          0x02    // Invalid ChannelID value.
#define ERR_INVALID_PROTOCOL_ID         0x03    // Invalid or unsupported ProtocolID, or there is a resource conflict (i.e. trying to connect to multiple mutually exclusive protocols such as J1850PWM and J1850VPW, or CAN and SCI, etc.).
#define ERR_NULL_PARAMETER              0x04    // NULL pointer supplied where a valid pointer is required.
#define ERR_INVALID_IOCTL_VALUE         0x05    // Invalid value for Ioctl parameter.
#define ERR_INVALID_FLAGS               0x06    // Invalid flag values.
#define ERR_FAILED                      0x07    // Undefined error, use PassThruGetLastError() for text description.
#define ERR_DEVICE_NOT_CONNECTED        0x08    // Unable to communicate with device.
#define ERR_TIMEOUT                     0x09    // Read or write timeout:
#define ERR_INVALID_MSG                 0x0A    // Invalid message structure pointed to by pMsg.
#define ERR_INVALID_TIME_INTERVAL       0x0B    // Invalid TimeInterval value.
#define ERR_EXCEEDED_LIMIT              0x0C    // Exceeded maximum number of message IDs or allocated space.
#define ERR_INVALID_MSG_ID              0x0D    // Invalid MsgID value.
#define ERR_DEVICE_IN_USE               0x0E    // Device is currently open.
#define ERR_INVALID_IOCTL_ID            0x0F    // Invalid IoctlID value.
#define ERR_BUFFER_EMPTY                0x10    // Protocol message buffer empty, no messages available to read.
#define ERR_BUFFER_FULL                 0x11    // Protocol message buffer full. All the messages specified may not have been transmitted.
#define ERR_BUFFER_OVERFLOW             0x12    // Indicates a buffer overflow occurred and messages were lost.
#define ERR_PIN_INVALID                 0x13    // Invalid pin number, pin number already in use, or voltage already applied to a different pin.
#define ERR_CHANNEL_IN_USE              0x14    // Channel number is currently connected.
#define ERR_MSG_PROTOCOL_ID             0x15    // Protocol type in the message does not match the protocol associated with the Channel ID
#define ERR_INVALID_FILTER_ID           0x16    // Invalid Filter ID value.
#define ERR_NO_FLOW_CONTROL             0x17    // No flow control filter set or matched (for ProtocolID ISO15765 only).
#define ERR_NOT_UNIQUE                  0x18    // A CAN ID in pPatternMsg or pFlowControlMsg matches either ID in an existing FLOW_CONTROL_FILTER
#define ERR_INVALID_BAUDRATE            0x19    // The desired baud rate cannot be achieved within the tolerance specified in SAE J2534-1 Section 6.5
#define ERR_INVALID_DEVICE_ID           0x1A    // Device ID invalid.

//Protocols
#define CAN                             0x05
#define ISO15765                        0x06
#define CAN_PS                          0x00008004
#define SW_CAN_PS                       0x00008008

//Ioctl commands
#define GET_CONFIG                              0x01    // pInput = SCONFIG_LIST, pOutput = NULL
#define SET_CONFIG                              0x02    // pInput = SCONFIG_LIST, pOutput = NULL
#define READ_VBATT                              0x03    // pInput = NULL, pOutput = unsigned long
#define FIVE_BAUD_INIT                          0x04    // pInput = SBYTE_ARRAY, pOutput = SBYTE_ARRAY
#define FAST_INIT                               0x05    // pInput = PASSTHRU_MSG, pOutput = PASSTHRU_MSG
#define CLEAR_TX_BUFFER                         0x07    // pInput = NULL, pOutput = NULL
#define CLEAR_RX_BUFFER                         0x08    // pInput = NULL, pOutput = NULL
#define CLEAR_PERIODIC_MSGS                     0x09    // pInput = NULL, pOutput = NULL
#define CLEAR_MSG_FILTERS                       0x0A    // pInput = NULL, pOutput = NULL
#define CLEAR_FUNCT_MSG_LOOKUP_TABLE            0x0B    // pInput = NULL, pOutput = NULL
#define ADD_TO_FUNCT_MSG_LOOKUP_TABLE           0x0C    // pInput = SBYTE_ARRAY, pOutput = NULL
#define DELETE_FROM_FUNCT_MSG_LOOKUP_TABLE      0x0D    // pInput = SBYTE_ARRAY, pOutput = NULL
#define READ_PROG_VOLTAGE                       0x0E    // pInput = NULL, pOutput = unsigned long

//Filter
#define PASS_FILTER                         0x01
#define BLOCK_FILTER                        0x02
#define FLOW_CONTROL_FILTER                 0x03

#define ISO15765_FRAME_PAD          0x0040
#define ISO15765_ADDR_TYPE          0x0080      // Defined above
#define CAN_29BIT_ID                0x0100      // Defined above
#define WAIT_P3_MIN_ONLY            0x0200
#define SCI_MODE                    0x400000
#define SCI_TX_VOLTAGE              0x800000
#define CAN_FD_BRS                  0x1000000   // 0 - Data with arbitration speed, 1 = data with CAN_FD_DATA_PHASE_RATE
#define CAN_FD_FORMAT               0x2000000   // 0 - CAN 2.0, 1 - CAN FD format

typedef struct {
  uint64_t Parameter;
  uint64_t Value;
} SCONFIG;

typedef struct {
  uint64_t NumOfParams;
  SCONFIG* ConfigPtr;
} SCONFIG_LIST;

typedef struct {
  uint64_t NumOfBytes;     // Number of bytes in the array
  unsigned char*  BytePtr;        // Array of bytes
} SBYTE_ARRAY;


typedef struct {
  uint64_t ProtocolID;
  uint64_t RxStatus;
  uint64_t TxFlags;
  uint64_t Timestamp;
  uint64_t DataSize;
  uint64_t ExtraDataIndex;
  uint8_t Data[24];
} PASSTHRU_MSG;

enum j2534_command_t {
  cmd_j2534_connect = 0xA0,
  cmd_j2534_disconnect = 0xA1,
  cmd_j2534_ioctl = 0xA2,
  cmd_j2534_filter = 0xA3,
  cmd_j2534_read_message = 0xA4,
  cmd_j2534_write_message = 0xA5,
  cmd_j2534_periodic_message = 0xA6,
};

enum j2534_term_t {
  cmd_j2534_ack = 0x00,
  cmd_j2534_nack = 0xFF
};

extern bool exec_cmd_j2534(packet_t *rx_packet, packet_t *tx_packet);

#endif /* J2534_H_ */
