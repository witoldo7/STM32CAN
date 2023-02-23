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
BOOL APIENTRY DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved) { 
    return TRUE; 
}
#endif

char LAST_ERROR[MAX_LEN];
char msgBuff[4528];

void last_error(const char *fmt, ...) {
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
