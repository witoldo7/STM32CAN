// SPDX-License-Identifier: MIT
/* 
 * SocketCAN driver for CombiAdapter/WQCAN.
 * Copyright (c) 2022 Witold Olechowski
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

char* parsemsg(PASSTHRU_MSG *msg) {
	if (msg == NULL) 
		return "NULL";
	char *buff;
	unsigned int length = 0;
	length = sprintf(buff,
		"\tMSG: %p\n"
		"\t\tProtocolID:\t%lu\n"
		"\t\tRxStatus:\t%08lX\n"
		"\t\tTxFlags:\t%08lX\n"
		"\t\tTimeStamp:\t0x%08lX (%lu \xC2\xB5sec)\n"
		"\t\tDataSize:\t%lu\n"
		"\t\tExtraData:\t%lu\n"
		"\t\tData:\n\t\t\t",
		msg, msg->ProtocolID, msg->RxStatus, msg->TxFlags, msg->Timestamp,
		msg->Timestamp, msg->DataSize, msg->ExtraDataIndex);
	for (unsigned int i=0; i <  msg->DataSize; i++)
		length = sprintf(buff + length, "%02X ", (uint8_t)msg->Data[i]);
	sprintf(buff + length, "\n");
	return buff;
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
