// SPDX-License-Identifier: MIT

/* 
 * WQCAN J2534 API Library.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#ifndef __UTILS_H
#define __UTILS_H

#include <stdbool.h>
#include <libusb.h>
#include "j2534drv.h"

#if defined(_MSC_VER)
#define snprintf _snprintf
#define THREAD_RETURN_VALUE	0
typedef HANDLE semaphore_t;
typedef HANDLE thread_t;

#if defined(__CYGWIN__)
typedef DWORD thread_return_t;
#else
#include <process.h>
typedef unsigned thread_return_t;
#endif

semaphore_t semaphore_create(char* semname) ;
void semaphore_give(semaphore_t semaphore);
void semaphore_take(semaphore_t semaphore);
void semaphore_destroy(semaphore_t semaphore);
int thread_create(thread_t *thread,
	thread_return_t (__stdcall *thread_entry)(void *arg), void *arg);
void thread_join(thread_t thread);
#else
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>

#define _snprintf_s(a,b,c,...) snprintf(a,b,__VA_ARGS__)
#define strcpy_s(a,b,...) strcpy(a,__VA_ARGS__)

#define THREAD_RETURN_VALUE	NULL
typedef pthread_t thread_t;
typedef sem_t * semaphore_t;

semaphore_t semaphore_create(char* semname);
void semaphore_give(semaphore_t semaphore);
void semaphore_take(semaphore_t semaphore);
int semaphore_take_timeout(semaphore_t semaphore, uint32_t timeout);
void semaphore_destroy(semaphore_t semaphore);
int thread_create(thread_t *thread,
	void *(*thread_entry)(void *arg), void *arg);
void thread_join(thread_t thread);
#endif

#define MAX_LEN	80

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

enum term_command_t {
  cmd_term_ack = 0x00,
  cmd_term_nack = 0xFF
};

extern semaphore_t slock;

void UnlockRx();
void LockRx();
void Lock(semaphore_t sem, uint32_t counter);
void Unlock(semaphore_t sem, uint32_t counter);
void sleep_us(int usec);
void enQueue(PASSTHRU_MSG msg);
bool deQueue(PASSTHRU_MSG* msg);
void clearQueue(void);
uint32_t sizeQueue(void);
char* parsemsg(PASSTHRU_MSG *msg);
void check_debug_log(void);
void last_error(const char *fmt, ...);
char* getLastError();
uint16_t covertPacketToBuffer(packet_t *packet, uint8_t *buffer);
void convertPacketToPMSG(uint8_t* data, uint16_t len, PASSTHRU_MSG* pMsg);
bool PASSTHRU_MSG_To_CANTxFrame(PASSTHRU_MSG *pMsg, CANTxFrame *canTx);
#endif // __UTILS_H
