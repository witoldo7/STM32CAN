// SPDX-License-Identifier: MIT

/* 
 * WQCAN J2534 API Library.
 * Copyright (c) 2022 Witold Olechowski.
 * 
 */ 

#include <errno.h>
#include <libusb.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "utils.h"
#include "log.h"
#ifdef _MSC_VER
#include <windows.h>
#elif _POSIX_C_SOURCE >= 199309L
#include <time.h>   // for nanosleep
#else
#include <unistd.h> // for usleep
#endif

void sleep_ms(int milliseconds) {
#ifdef _MSC_VER
    Sleep(milliseconds);
#elif _POSIX_C_SOURCE >= 199309L
    struct timespec ts;
    ts.tv_sec =0;// milliseconds / 1000;
    ts.tv_nsec = 100000;// milliseconds* 1000;//1000000;
    nanosleep(&ts, NULL);
#else
    if (milliseconds >= 1000)
      sleep(milliseconds / 1000);
    usleep((milliseconds % 1000) * 1000);
#endif
}

#ifdef _MSC_VER
BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) { 
    return TRUE; 
}
#endif


#if defined(_MSC_VER)
semaphore_t semaphore_create(char* semname) {
	(void)semname;
	return CreateSemaphore(NULL, 0, 1, semname);
}

void semaphore_give(semaphore_t semaphore) {
	(void)ReleaseSemaphore(semaphore, 1, NULL);
}

void semaphore_take(semaphore_t semaphore) {
	(void)WaitForSingleObject(semaphore, INFINITE);
}

void semaphore_destroy(semaphore_t semaphore) {
	(void)CloseHandle(semaphore);
}

int thread_create(thread_t *thread,
	thread_return_t (__stdcall *thread_entry)(void *arg), void *arg)
{
#if defined(__CYGWIN__)
	*thread = CreateThread(NULL, 0, thread_entry, arg, 0, NULL);
#else
	*thread = (HANDLE)_beginthreadex(NULL, 0, thread_entry, arg, 0, NULL);
#endif
	return *thread != NULL ? 0 : -1;
}

void thread_join(thread_t thread) {
	(void)WaitForSingleObject(thread, INFINITE);
	(void)CloseHandle(thread);
}

#else

semaphore_t semaphore_create(char* semname) {
	sem_t *semaphore;
	char name[50];

	snprintf(name, sizeof(name), "/org.libusb.j2534.%s:%d", semname, (int)getpid());
	semaphore = sem_open(name, O_CREAT | O_EXCL, 0, 0);
	if (semaphore == SEM_FAILED)
		return NULL;
	/* Remove semaphore so that it does not persist after process exits */
	(void)sem_unlink(name);
	return semaphore;
}

void semaphore_give(semaphore_t semaphore) {
	(void)sem_post(semaphore);
}

void semaphore_take(semaphore_t semaphore) {
	(void)sem_wait(semaphore);
}

void semaphore_destroy(semaphore_t semaphore) {
	(void)sem_close(semaphore);
}

int thread_create(thread_t *thread,
	void *(*thread_entry)(void *arg), void *arg) {
	return pthread_create(thread, NULL, thread_entry, arg) == 0 ? 0 : -1;
}

void thread_join(thread_t thread) {
	(void)pthread_join(thread, NULL);
}
#endif

char LAST_ERROR[MAX_LEN];
char msgBuff[4528];

static uint32_t MAX_SIZE = 512;
static volatile PASSTHRU_MSG queue[512] = {0};
static uint32_t front = 0;
static uint32_t rear = 0;
static uint32_t queue_size = 0;
static uint32_t m_counter=0;

void LockRx() {
	if (m_counter++ > 1) {
		semaphore_take(slock);
	}
}

void UnlockRx() {
	if (m_counter-- > 0) {
		semaphore_give(slock);
	}
}

void Lock(semaphore_t sem, uint32_t counter) {
	if (counter++ > 1) {
		semaphore_take(sem);
	}
}

void Unlock(semaphore_t sem, uint32_t counter) {
	if (counter-- > 0) {
		semaphore_give(sem);
	}
}


void enQueue(PASSTHRU_MSG msg) {
	LockRx();
	uint32_t next_rear = (rear + 1) % MAX_SIZE;
    if (next_rear == front) {
    	UnlockRx();
        return;
    }
    queue[rear] = msg;
    rear = next_rear;
    queue_size++;
    UnlockRx();
}

bool deQueue(PASSTHRU_MSG *msg) {
	LockRx();
    if (front == rear) {
    	UnlockRx();
        return false;
    }
    *msg = queue[front];
    front = (front + 1) % MAX_SIZE;
    queue_size--;
    UnlockRx();
	return true;
}

void clearQueue(void) {
	LockRx();
    front = 0;
    rear = 0;
    queue_size = 0;
    UnlockRx();
}

uint32_t sizeQueue(void) {
	//LockRx();
	uint32_t q = queue_size;
    //UnlockRx();
    return q;
}

void last_error(const char *fmt, ...) {
	memset(LAST_ERROR, 0, MAX_LEN);
	snprintf(LAST_ERROR, MAX_LEN, fmt);
	log_trace(fmt);
}

char *getLastError() {
	return LAST_ERROR;
}

char* parsemsg(PASSTHRU_MSG* msg) {
	if (msg == NULL)
		return "NULL";
	unsigned int length = 0;
	length = sprintf(msgBuff,
		"\tMSG: %p\n"
		"\t\tProtocolID:\t%lu\n"
		"\t\tRxStatus:\t%08lX\n"
		"\t\tTxFlags:\t%08lX\n"
		"\t\tTimeStamp:\t0x%08lX (%lu \xC2\xB5sec)\n"
		"\t\tDataSize:\t%lu\n"
		"\t\tExtraData:\t%lu\n"
		"\t\tData:\t",
		msg, msg->ProtocolID, msg->RxStatus, msg->TxFlags, msg->Timestamp,
		msg->Timestamp, msg->DataSize, msg->ExtraDataIndex);

		for (unsigned int i = 0; i < msg->DataSize; i++)
			sprintf(msgBuff + length + i*4, "%02X, ", (uint8_t)msg->Data[i]);

	return msgBuff;
}

void check_debug_log(void) {
	const char* env = getenv("J2534_DEBUG");
	if (env == NULL) {
		log_set_quiet(1);
		return;
	}
	
	if (env[0] == 0x31) //J2534_DEBUG=1
		return;
	
	FILE* file = fopen(env, "a");
		
	if (file == NULL)
		return;

	log_add_fp(file, LOG_TRACE);
}

uint16_t covertPacketToBuffer(packet_t *packet, uint8_t *buffer) {
  uint16_t size = 0;
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

void convertPacketToPMSG(uint8_t *data, uint16_t len, PASSTHRU_MSG* pMsg) {
	uint16_t protocol = 0;
	memcpy(&protocol, data, sizeof(protocol));
	switch (protocol) {
	case CAN:
	case CAN_PS:
	case SW_CAN_PS:
		CANRxFrame rxMsg = {0};
		memcpy(&rxMsg, data + 2, len - 2);
		pMsg->ProtocolID = protocol;
		pMsg->DataSize = rxMsg.DLC + 4;
		pMsg->ExtraDataIndex = 0;
		uint32_t id = 0;
		if (rxMsg.common.XTD)
			id = rxMsg.ext.EID & 0x1FFFFFFF;
		else
			id = rxMsg.std.SID & 0x7FF;

		pMsg->Data[0] = (uint8_t)(id >> 24);
		pMsg->Data[1] = (uint8_t)(id >> 16);
		pMsg->Data[2] = (uint8_t)(id >> 8);
		pMsg->Data[3] = (uint8_t)id;
		memcpy(pMsg->Data + 4, rxMsg.data8, rxMsg.DLC);
		pMsg->Timestamp = rxMsg.RXTS;
		pMsg->RxStatus = (rxMsg.common.XTD ? CAN_29BIT_ID : 0) | TX_MSG_TYPE;
		pMsg->TxFlags = 0;
		break;
	default:
		log_error("convertPacketToPMSG: not supported protocol: %02X", protocol);
		break;
	}
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
