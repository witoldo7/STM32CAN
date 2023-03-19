// SPDX-License-Identifier: MIT

/* 
 * WQCAN J2534 API Library.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#ifndef __UTILS_H
#define __UTILS_H

#include <stdbool.h>
#include "j2534.h"

#define MAX_LEN	80
enum j2534_command_t {
  cmd_j2534_connect = 0xA0,
  cmd_j2534_disconnect = 0xA1,
  cmd_j2534_ioctl = 0xA2,
  cmd_j2534_filter = 0xA3,
  cmd_j2534_read_message = 0xA4,
  cmd_j2534_write_message = 0xA5,
  cmd_j2534_periodic_message = 0xA6,
  cmd_j2534_misc = 0x20,
};

enum j2534_term_t {
  cmd_j2534_ack = 0x00,
  cmd_j2534_nack = 0xFF
};

typedef struct {
  uint8_t cmd_code; // command code
  uint16_t data_len;    // data block length
  uint8_t *data;        // optional data block
  uint8_t term;     // terminator
} packet_t;

typedef struct {
  union {
    struct {
      union {
        struct {
          uint32_t          EID:29;     /**< @brief Extended identifier.    */
        } ext;
        struct {
          uint32_t          _R1:18;     /**< @brief Reserved for offset.    */
          uint32_t          SID:11;     /**< @brief Standard identifier.    */
        } std;
        struct {
          uint32_t          _R1:29;     /**< @brief Reserved for offset.    */
          uint32_t          RTR:1;      /**< @brief Remote transmit request.*/
          uint32_t          XTD:1;      /**< @brief Extended identifier.    */
          uint32_t          ESI:1;      /**< @brief Error state indicator.  */
        } common;
      };
      uint32_t              _R2:16;
      uint32_t              DLC:4;      /**< @brief Data length code.       */
      uint32_t              BRS:1;      /**< @brief Accepted non-matching
                                                    frame.                  */
      uint32_t              FDF:1;      /**< @brief FDCAN frame format.     */
      uint32_t              _R3:1;
      uint32_t              EFC:1;      /**< @brief Event FIFO control.     */
      uint32_t              MM:8;       /**< @brief Message event marker.   */
    };
    uint32_t                header32[2];
  };
  /**
   * @brief   Frame data.
   */
  union {
    uint8_t                 data8[64];
    uint16_t                data16[64 / 2];
    uint32_t                data32[64 / 4];
  };
} CANTxFrame;

typedef struct {
  /**
   * @brief   Frame header.
   */
  union {
    struct {
      union {
        struct {
          uint32_t          EID:29;     /**< @brief Extended identifier.    */
        } ext;
        struct {
          uint32_t          _R1:18;
          uint32_t          SID:11;     /**< @brief Standard identifier.    */
        } std;
        struct {
          uint32_t          _R1:29;     /**< @brief Reserved for offset.    */
          uint32_t          RTR:1;      /**< @brief Remote transmit request.*/
          uint32_t          XTD:1;      /**< @brief Extended identifier.    */
          uint32_t          ESI:1;      /**< @brief Error state indicator.  */
        } common;
      };
      uint32_t              RXTS:16;    /**< @brief TX time stamp.          */
      uint32_t              DLC:4;      /**< @brief Data length code.       */
      uint32_t              BRS:1;      /**< @brief Bit rate switch.        */
      uint32_t              FDF:1;      /**< @brief FDCAN frame format.     */
      uint32_t              _R2:2;
      uint32_t              FIDX:7;     /**< @brief Filter index.           */
      uint32_t              ANMF:1;     /**< @brief Accepted non-matching
                                                    frame.                  */
    };
    uint32_t                header32[2];
  };
  /**
   * @brief   Frame data.
   */
  union {
    uint8_t                 data8[64];
    uint16_t                data16[64 / 2];
    uint32_t                data32[64 / 4];
  };
} CANRxFrame;

void sleep_ms(int milliseconds);
void enQueue(PASSTHRU_MSG msg);
PASSTHRU_MSG deQueue(void);
void clearQueue(void);
int sizeQueue(void);
char* parsemsg(PASSTHRU_MSG *msg);
void check_debug_log(void);
void last_error(const char *fmt, ...);
char* getLastError();
uint16_t covertPacketToBuffer(packet_t *packet, uint8_t *buffer);
void convertPacketToPMSG(packet_t* packet, PASSTHRU_MSG* pMsg);
bool PASSTHRU_MSG_To_CANTxFrame(PASSTHRU_MSG *pMsg, CANTxFrame *canTx);
#endif // __UTILS_H
