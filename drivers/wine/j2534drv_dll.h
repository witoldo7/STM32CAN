/*
 * j2534drv.dll
 *
 * Generated from j2534drv.dll by winedump.
 *
 * DO NOT SEND GENERATED DLLS FOR INCLUSION INTO WINE !
 *
 */
#ifndef __WINE_J2534DRV_DLL_H
#define __WINE_J2534DRV_DLL_H

#include "windef.h"
#include "wine/debug.h"
#include "winbase.h"
#include "winnt.h"


APIENTRY __stdcall J2534DRV_DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved);
void __stdcall J2534DRV_Lock(semaphore_t sem, uint32_t counter);
void __stdcall J2534DRV_LockRx(void);
bool __stdcall J2534DRV_PASSTHRU_MSG_To_CANTxFrame(PASSTHRU_MSG * pMsg, CANTxFrame * canTx);
PTAPI __stdcall J2534DRV_PassThruClose(uint32_t DeviceID);
PTAPI __stdcall J2534DRV_PassThruConnect(uint32_t DeviceID, uint32_t protocolID, uint32_t flags, uint32_t baud, uint32_t * pChannelID);
PTAPI __stdcall J2534DRV_PassThruDisconnect(uint32_t ChannelID);
PTAPI __stdcall J2534DRV_PassThruGetLastError(char * pErrorDescription);
PTAPI __stdcall J2534DRV_PassThruIoctl(uint32_t ChannelID, uint32_t ioctlID, void * pInput, void * pOutput);
PTAPI __stdcall J2534DRV_PassThruOpen(void);
PTAPI __stdcall J2534DRV_PassThruReadMsgs(uint32_t ChannelID, PASSTHRU_MSG * pMsg, uint32_t * pNumMsgs, uint32_t Timeout);
PTAPI __stdcall J2534DRV_PassThruReadVersion(uint32_t DeviceID, char * pFirmwareVersion, char * pDllVersion, char * pApiVersion);
PTAPI __stdcall J2534DRV_PassThruSetProgrammingVoltage(uint32_t DeviceID, uint32_t PinNumber, uint32_t Voltage);
PTAPI __stdcall J2534DRV_PassThruStartMsgFilter(uint32_t ChannelID, uint32_t FilterType, PASSTHRU_MSG * pMaskMsg, PASSTHRU_MSG * pPatternMsg, PASSTHRU_MSG * pFlowControlMsg, uint32_t * pFilterID);
PTAPI __stdcall J2534DRV_PassThruStartPeriodicMsg(uint32_t ChannelID, PASSTHRU_MSG * pMsg, uint32_t * pMsgID, uint32_t timeInterval);
PTAPI __stdcall J2534DRV_PassThruStopMsgFilter(uint32_t ChannelID, uint32_t FilterID);
PTAPI __stdcall J2534DRV_PassThruStopPeriodicMsg(uint32_t ChannelID, uint32_t msgID);
PTAPI __stdcall J2534DRV_PassThruWriteMsgs(uint32_t ChannelID, PASSTHRU_MSG * pMsg, uint32_t * pNumMsgs, uint32_t Timeout);
void __stdcall J2534DRV_Unlock(semaphore_t sem, uint32_t counter);
void __stdcall J2534DRV_UnlockRx(void);
/* __stdcall J2534DRV___local_stdio_printf_options(); */
/* __stdcall J2534DRV__snprintf_s(); */
/* __stdcall J2534DRV__vfprintf_l(); */
/* __stdcall J2534DRV__vsnprintf_l(); */
/* __stdcall J2534DRV__vsnprintf_s_l(); */
/* __stdcall J2534DRV__vsprintf_l(); */
void __stdcall J2534DRV_check_debug_log(void);
void __stdcall J2534DRV_clearQueue(void);
void __stdcall J2534DRV_convertPacketToPMSG(uint8_t * data, uint16_t len, PASSTHRU_MSG * pMsg);
uint32_t __stdcall J2534DRV_coppyMessages(PASSTHRU_MSG * pMsg, uint32_t pNumMsgs);
uint16_t __stdcall J2534DRV_covertPacketToBuffer(packet_t * packet, uint8_t * buffer);
bool __stdcall J2534DRV_deQueue(PASSTHRU_MSG * msg);
/* __stdcall J2534DRV_do_exit(); */
void __stdcall J2534DRV_enQueue(PASSTHRU_MSG msg);
/* __stdcall J2534DRV_fprintf(); */
char * __stdcall J2534DRV_getLastError(void);
void __stdcall J2534DRV_last_error(char * fmt, ...);
int __stdcall J2534DRV_log_add_callback(log_LogFn fn, void * udata, int level);
int __stdcall J2534DRV_log_add_fp(FILE * fp, int level);
char * __stdcall J2534DRV_log_level_string(int level);
void __stdcall J2534DRV_log_log(int level, char * file, int line, char * fmt, ...);
void __stdcall J2534DRV_log_set_level(int level);
void __stdcall J2534DRV_log_set_lock(log_LockFn fn, void * udata);
void __stdcall J2534DRV_log_set_quiet(bool enable);
char * __stdcall J2534DRV_parsemsg(PASSTHRU_MSG * msg);
char * __stdcall J2534DRV_parsemsgb(PASSTHRU_MSG * msg, char * buff);
/* __stdcall J2534DRV_resp_packet(); */
/* __stdcall J2534DRV_rx_counter(); */
semaphore_t __stdcall J2534DRV_semaphore_create(char * semname);
void __stdcall J2534DRV_semaphore_destroy(semaphore_t semaphore);
void __stdcall J2534DRV_semaphore_give(semaphore_t semaphore);
void __stdcall J2534DRV_semaphore_take(semaphore_t semaphore);
int __stdcall J2534DRV_semaphore_take_timeout(semaphore_t semaphore, uint32_t timeout);
uint32_t __stdcall J2534DRV_sizeQueue(void);
void __stdcall J2534DRV_sleep_us(int usec);
/* __stdcall J2534DRV_sprintf(); */
/* __stdcall J2534DRV_thread_create(); */
void __stdcall J2534DRV_thread_join(thread_t thread);
/* __stdcall J2534DRV_transfer(); */
char * __stdcall J2534DRV_translate(uint32_t type);
char * __stdcall J2534DRV_translateError(uint8_t err);
char * __stdcall J2534DRV_translateErrorDetail(uint32_t err);
char * __stdcall J2534DRV_translateFilterType(uint32_t type);
char * __stdcall J2534DRV_translateIoctl(uint32_t ioctl);
char * __stdcall J2534DRV_translateParam(uint32_t param);
char * __stdcall J2534DRV_translateProtocol(uint32_t protocol);
/* __stdcall J2534DRV_tx_counter(); */
/* __stdcall J2534DRV_tx_packet(); */
/* __stdcall J2534DRV_vfprintf(); */
/* __stdcall J2534DRV_vsnprintf(); */



#endif	/* __WINE_J2534DRV_DLL_H */
