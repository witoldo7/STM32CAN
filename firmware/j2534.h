// SPDX-License-Identifier: Apache-2.0

/*
 * STM32CAN Firmware.
 * Copyright (c) 2023 Witold Olechowski
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

//param
#define DATA_RATE               0x01    // 5-500000
#define LOOPBACK                0x03    // 0 (OFF), 1 (ON) [0]
#define NODE_ADDRESS            0x04    // J1850PWM: 0x00-0xFF
#define NETWORK_LINE            0x05    // J1850PWM: 0 (BUS_NORMAL), 1 (BUS_PLUS), 2 (BUS_MINUS) [0]
#define P1_MIN                  0x06    // ISO9141 or ISO14230: Not used by interface
#define P1_MAX                  0x07    // ISO9141 or ISO14230: 0x1-0xFFFF (.5 ms per bit) [40 (20ms)]
#define P2_MIN                  0x08    // ISO9141 or ISO14230: Not used by interface
#define P2_MAX                  0x09    // ISO9141 or ISO14230: Not used by interface
#define P3_MIN                  0x0A    // ISO9141 or ISO14230: 0x0-0xFFFF (.5 ms per bit) [110 (55ms)]
#define P3_MAX                  0x0B    // ISO9141 or ISO14230: Not used by interface
#define P4_MIN                  0x0C    // ISO9141 or ISO14230: 0x0-0xFFFF (.5 ms per bit) [10 (5ms)]
#define P4_MAX                  0x0D    // ISO9141 or ISO14230: Not used by interface
#define W0                      0x19    // ISO9141: 0x0-0xFFFF (1 ms per bit) [300]
#define W1                      0x0E    // ISO9141 or ISO14230: 0x0-0xFFFF (1 ms per bit) [300]
#define W2                      0x0F    // ISO9141 or ISO14230: 0x0-0xFFFF (1 ms per bit) [20]
#define W3                      0x10    // ISO9141 or ISO14230: 0x0-0xFFFF (1 ms per bit) [20]
#define W4                      0x11    // ISO9141 or ISO14230: 0x0-0xFFFF (1 ms per bit) [50]
#define W5                      0x12    // ISO9141 or ISO14230: 0x0-0xFFFF (1 ms per bit) [300]
#define TIDLE                   0x13    // ISO9141 or ISO14230: 0x0-0xFFFF (1 ms per bit) [300]
#define TINIL                   0x14    // ISO9141 or ISO14230: 0x0-0xFFFF (1 ms per bit) [25]
#define TWUP                    0x15    // ISO9141 or ISO14230: 0x0-0xFFFF (1 ms per bit) [50]
#define PARITY                  0x16    // ISO9141 or ISO14230: 0 (NO_PARITY), 1 (ODD_PARITY), 2 (EVEN_PARITY) [0]
#define BIT_SAMPLE_POINT        0x17    // CAN: 0-100 (1% per bit) [80]
#define SYNC_JUMP_WIDTH         0x18    // CAN: 0-100 (1% per bit) [15]
#define T1_MAX                  0x1A    // SCI: 0x0-0xFFFF (1 ms per bit) [20]
#define T2_MAX                  0x1B    // SCI: 0x0-0xFFFF (1 ms per bit) [100]
#define T3_MAX                  0x24    // SCI: 0x0-0xFFFF (1 ms per bit) [50]
#define T4_MAX                  0x1C    // SCI: 0x0-0xFFFF (1 ms per bit) [20]
#define T5_MAX                  0x1D    // SCI: 0x0-0xFFFF (1 ms per bit) [100]
#define ISO15765_BS             0x1E    // ISO15765: 0x0-0xFF [0]
#define ISO15765_STMIN          0x1F    // ISO15765: 0x0-0xFF [0]
#define ISO15765_BS_TX          0x22    // ISO15765: 0x0-0xFF,0xFFFF [0xFFFF]
#define ISO15765_STMIN_TX       0x23    // ISO15765: 0x0-0xFF,0xFFFF [0xFFFF]
#define DATA_BITS               0x20    // ISO9141 or ISO14230: 0 (8 data bits), 1 (7 data bits) [0]
#define FIVE_BAUD_MOD           0x21    // ISO9141 or ISO14230: 0 (ISO 9141-2/14230-4), 1 (Inv KB2), 2 (Inv Addr), 3 (ISO 9141) [0]
#define ISO15765_WFT_MAX        0x25    // ISO15765: 0x0-0xFF [0]

// J2534-2 Configuration Parameter Values
#define CAN_MIXED_FORMAT                0x00008000  // See #defines below. [0]
#define J1962_PINS                      0x00008001  // 0xPPSS PP: 0x00-0x10 SS: 0x00-0x10 PP!=SS, except 0x0000. Exclude pins 4, 5, and 16. [0]
#define SW_CAN_HS_DATA_RATE             0x00008010  // SWCAN: 5-500000 [83333]
#define SW_CAN_SPEEDCHANGE_ENABLE       0x00008011  // SWCAN: 0 (DISABLE_SPDCHANGE), 1 (ENABLE_SPDCHANGE) [0]
#define SW_CAN_RES_SWITCH               0x00008012  // SWCAN: 0 (DISCONNECT_RESISTOR), 1 (CONNECT_RESISTOR), 2 (AUTO_ RESISTOR) [0]
#define ACTIVE_CHANNELS                 0x00008020  // ANALOG: 0-0xFFFFFFFF
#define SAMPLE_RATE                     0x00008021  // ANALOG: 0-0xFFFFFFFF [0] (high bit changes meaning from samples/sec to seconds/sample)
#define SAMPLES_PER_READING             0x00008022  // ANALOG: 1-0xFFFFFFFF [1]
#define READINGS_PER_MSG                0x00008023  // ANALOG: 1-0x00000408 (1 - 1032) [1]
#define AVERAGING_METHOD                0x00008024  // ANALOG: 0-0xFFFFFFFF [0]
#define SAMPLE_RESOLUTION               0x00008025  // ANALOG READ-ONLY: 0x1-0x20 (1 - 32)
#define INPUT_RANGE_LOW                 0x00008026  // ANALOG READ-ONLY: 0x80000000-0x7FFFFFFF (-2147483648-2147483647)
#define INPUT_RANGE_HIGH                0x00008027  // ANALOG READ-ONLY: 0x80000000-0x7FFFFFFF (-2147483648-2147483647)

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

//User def.
#define FILTER_NBR 16

typedef struct {
  uint32_t Parameter;
  uint32_t Value;
} SCONFIG;

typedef struct {
  uint32_t NumOfParams;
  SCONFIG* ConfigPtr;
} SCONFIG_LIST;

typedef struct {
    uint32_t    NumOfBytes;     // Number of bytes in the array
    uint8_t*    BytePtr;        // Array of bytes
} SBYTE_ARRAY;

typedef struct {
  CAN_Filter filter[FILTER_NBR];
  CAN_RamAddress *msgRam;
  uint8_t index;
} CAN_FILTER;

typedef struct {
  CANDriver* canp;
  CANConfig* canCfg;
  CANRamConfig* ramCfg;
  CAN_RamAddress* ramAdr;
  CAN_FILTER* canFilter;
  bool (*cb)(CANRxFrame*, packet_t*);
  uint32_t protocol;
  uint32_t flags;
  uint32_t bitRate;
  uint32_t bitSamplePoint;
  uint32_t syncJumpWidth;
  uint32_t iso15766bs;
  uint32_t iso15766bsTx;
  uint32_t iso15766stmin;
  uint32_t iso15766stminTx;
  uint32_t iso15766wtfMax;
  uint32_t canMixedFormat;
  uint32_t J1962Pins;
  uint32_t swCanHsDataRate;
  uint32_t swCanSpeedChangeEnable;
  uint32_t swCanResSwitch;
  bool loopback;
} j2534_conn;

enum j2534_command_t {
  cmd_j2534_connect                = 0xA0,
  cmd_j2534_disconnect             = 0xA1,
  cmd_j2534_ioctl                  = 0xA2,
  cmd_j2534_filter                 = 0xA3,
  cmd_j2534_stop_filter            = 0xA4,
  cmd_j2534_read_message           = 0xA5,
  cmd_j2534_write_message          = 0xA6,
  cmd_j2534_start_periodic_message = 0xA7,
  cmd_j2534_stop_periodic_message  = 0xA8,
  cmd_j2534_misc                   = 0x20,
};

enum j2534_term_t {
  cmd_j2534_ack = 0x00,
  cmd_j2534_nack = 0xFF
};

extern bool exec_cmd_j2534(packet_t *rx_packet, packet_t *tx_packet);

#endif /* J2534_H_ */
