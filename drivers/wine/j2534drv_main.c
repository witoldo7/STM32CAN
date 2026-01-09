/*
 * j2534drv.dll
 *
 * Generated from j2534drv.dll by winedump.
 *
 * DO NOT SUBMIT GENERATED DLLS FOR INCLUSION INTO WINE!
 *
 */

#include "config.h"

#include <stdarg.h>

#include "windef.h"
#include "winbase.h"
#include "j2534drv_dll.h"
#include "wine/debug.h"

WINE_DEFAULT_DEBUG_CHANNEL(j2534drv);

/******************************************************************
 *		DllMain (J2534DRV.1)
 *
 *
 */
APIENTRY __stdcall J2534DRV_DllMain(HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	FIXME(":stub\n");
	return (APIENTRY) 0;
}
/******************************************************************
 *		Lock (J2534DRV.2)
 *
 *
 */
void __stdcall J2534DRV_Lock(semaphore_t sem, uint32_t counter)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		LockRx (J2534DRV.3)
 *
 *
 */
void __stdcall J2534DRV_LockRx(void)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		PASSTHRU_MSG_To_CANTxFrame (J2534DRV.4)
 *
 *
 */
bool __stdcall J2534DRV_PASSTHRU_MSG_To_CANTxFrame(PASSTHRU_MSG * pMsg, CANTxFrame * canTx)
{
	FIXME(":stub\n");
	return (bool) 0;
}
/******************************************************************
 *		PassThruClose (J2534DRV.5)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruClose(uint32_t DeviceID)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruConnect (J2534DRV.6)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruConnect(uint32_t DeviceID, uint32_t protocolID, uint32_t flags, uint32_t baud, uint32_t * pChannelID)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruDisconnect (J2534DRV.7)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruDisconnect(uint32_t ChannelID)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruGetLastError (J2534DRV.8)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruGetLastError(char * pErrorDescription)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruIoctl (J2534DRV.9)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruIoctl(uint32_t ChannelID, uint32_t ioctlID, void * pInput, void * pOutput)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruOpen (J2534DRV.10)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruOpen(void)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruReadMsgs (J2534DRV.11)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruReadMsgs(uint32_t ChannelID, PASSTHRU_MSG * pMsg, uint32_t * pNumMsgs, uint32_t Timeout)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruReadVersion (J2534DRV.12)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruReadVersion(uint32_t DeviceID, char * pFirmwareVersion, char * pDllVersion, char * pApiVersion)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruSetProgrammingVoltage (J2534DRV.13)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruSetProgrammingVoltage(uint32_t DeviceID, uint32_t PinNumber, uint32_t Voltage)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruStartMsgFilter (J2534DRV.14)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruStartMsgFilter(uint32_t ChannelID, uint32_t FilterType, PASSTHRU_MSG * pMaskMsg, PASSTHRU_MSG * pPatternMsg, PASSTHRU_MSG * pFlowControlMsg, uint32_t * pFilterID)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruStartPeriodicMsg (J2534DRV.15)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruStartPeriodicMsg(uint32_t ChannelID, PASSTHRU_MSG * pMsg, uint32_t * pMsgID, uint32_t timeInterval)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruStopMsgFilter (J2534DRV.16)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruStopMsgFilter(uint32_t ChannelID, uint32_t FilterID)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruStopPeriodicMsg (J2534DRV.17)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruStopPeriodicMsg(uint32_t ChannelID, uint32_t msgID)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		PassThruWriteMsgs (J2534DRV.18)
 *
 *
 */
PTAPI __stdcall J2534DRV_PassThruWriteMsgs(uint32_t ChannelID, PASSTHRU_MSG * pMsg, uint32_t * pNumMsgs, uint32_t Timeout)
{
	FIXME(":stub\n");
	return (PTAPI) 0;
}
/******************************************************************
 *		Unlock (J2534DRV.19)
 *
 *
 */
void __stdcall J2534DRV_Unlock(semaphore_t sem, uint32_t counter)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		UnlockRx (J2534DRV.20)
 *
 *
 */
void __stdcall J2534DRV_UnlockRx(void)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		__local_stdio_printf_options (J2534DRV.21)
 *
 *
 */
#if 0
__stdcall J2534DRV___local_stdio_printf_options()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		_snprintf_s (J2534DRV.22)
 *
 *
 */
#if 0
__stdcall J2534DRV__snprintf_s()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		_vfprintf_l (J2534DRV.23)
 *
 *
 */
#if 0
__stdcall J2534DRV__vfprintf_l()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		_vsnprintf_l (J2534DRV.24)
 *
 *
 */
#if 0
__stdcall J2534DRV__vsnprintf_l()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		_vsnprintf_s_l (J2534DRV.25)
 *
 *
 */
#if 0
__stdcall J2534DRV__vsnprintf_s_l()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		_vsprintf_l (J2534DRV.26)
 *
 *
 */
#if 0
__stdcall J2534DRV__vsprintf_l()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		check_debug_log (J2534DRV.27)
 *
 *
 */
void __stdcall J2534DRV_check_debug_log(void)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		clearQueue (J2534DRV.28)
 *
 *
 */
void __stdcall J2534DRV_clearQueue(void)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		convertPacketToPMSG (J2534DRV.29)
 *
 *
 */
void __stdcall J2534DRV_convertPacketToPMSG(uint8_t * data, uint16_t len, PASSTHRU_MSG * pMsg)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		coppyMessages (J2534DRV.30)
 *
 *
 */
uint32_t __stdcall J2534DRV_coppyMessages(PASSTHRU_MSG * pMsg, uint32_t pNumMsgs)
{
	FIXME(":stub\n");
	return (uint32_t) 0;
}
/******************************************************************
 *		covertPacketToBuffer (J2534DRV.31)
 *
 *
 */
uint16_t __stdcall J2534DRV_covertPacketToBuffer(packet_t * packet, uint8_t * buffer)
{
	FIXME(":stub\n");
	return (uint16_t) 0;
}
/******************************************************************
 *		deQueue (J2534DRV.32)
 *
 *
 */
bool __stdcall J2534DRV_deQueue(PASSTHRU_MSG * msg)
{
	FIXME(":stub\n");
	return (bool) 0;
}
/******************************************************************
 *		do_exit (J2534DRV.33)
 *
 *
 */
#if 0
__stdcall J2534DRV_do_exit()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		enQueue (J2534DRV.34)
 *
 *
 */
void __stdcall J2534DRV_enQueue(PASSTHRU_MSG msg)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		fprintf (J2534DRV.35)
 *
 *
 */
#if 0
__stdcall J2534DRV_fprintf()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		getLastError (J2534DRV.36)
 *
 *
 */
char * __stdcall J2534DRV_getLastError(void)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		last_error (J2534DRV.37)
 *
 *
 */
void __stdcall J2534DRV_last_error(char * fmt, ...)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		log_add_callback (J2534DRV.38)
 *
 *
 */
int __stdcall J2534DRV_log_add_callback(log_LogFn fn, void * udata, int level)
{
	FIXME(":stub\n");
	return (int) 0;
}
/******************************************************************
 *		log_add_fp (J2534DRV.39)
 *
 *
 */
int __stdcall J2534DRV_log_add_fp(FILE * fp, int level)
{
	FIXME(":stub\n");
	return (int) 0;
}
/******************************************************************
 *		log_level_string (J2534DRV.40)
 *
 *
 */
char * __stdcall J2534DRV_log_level_string(int level)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		log_log (J2534DRV.41)
 *
 *
 */
void __stdcall J2534DRV_log_log(int level, char * file, int line, char * fmt, ...)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		log_set_level (J2534DRV.42)
 *
 *
 */
void __stdcall J2534DRV_log_set_level(int level)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		log_set_lock (J2534DRV.43)
 *
 *
 */
void __stdcall J2534DRV_log_set_lock(log_LockFn fn, void * udata)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		log_set_quiet (J2534DRV.44)
 *
 *
 */
void __stdcall J2534DRV_log_set_quiet(bool enable)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		parsemsg (J2534DRV.45)
 *
 *
 */
char * __stdcall J2534DRV_parsemsg(PASSTHRU_MSG * msg)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		parsemsgb (J2534DRV.46)
 *
 *
 */
char * __stdcall J2534DRV_parsemsgb(PASSTHRU_MSG * msg, char * buff)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		resp_packet (J2534DRV.47)
 *
 *
 */
#if 0
__stdcall J2534DRV_resp_packet()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		rx_counter (J2534DRV.48)
 *
 *
 */
#if 0
__stdcall J2534DRV_rx_counter()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		semaphore_create (J2534DRV.49)
 *
 *
 */
semaphore_t __stdcall J2534DRV_semaphore_create(char * semname)
{
	FIXME(":stub\n");
	return (semaphore_t) 0;
}
/******************************************************************
 *		semaphore_destroy (J2534DRV.50)
 *
 *
 */
void __stdcall J2534DRV_semaphore_destroy(semaphore_t semaphore)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		semaphore_give (J2534DRV.51)
 *
 *
 */
void __stdcall J2534DRV_semaphore_give(semaphore_t semaphore)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		semaphore_take (J2534DRV.52)
 *
 *
 */
void __stdcall J2534DRV_semaphore_take(semaphore_t semaphore)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		semaphore_take_timeout (J2534DRV.53)
 *
 *
 */
int __stdcall J2534DRV_semaphore_take_timeout(semaphore_t semaphore, uint32_t timeout)
{
	FIXME(":stub\n");
	return (int) 0;
}
/******************************************************************
 *		sizeQueue (J2534DRV.54)
 *
 *
 */
uint32_t __stdcall J2534DRV_sizeQueue(void)
{
	FIXME(":stub\n");
	return (uint32_t) 0;
}
/******************************************************************
 *		sleep_us (J2534DRV.55)
 *
 *
 */
void __stdcall J2534DRV_sleep_us(int usec)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		sprintf (J2534DRV.56)
 *
 *
 */
#if 0
__stdcall J2534DRV_sprintf()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		thread_create (J2534DRV.57)
 *
 *
 */
#if 0
__stdcall J2534DRV_thread_create()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		thread_join (J2534DRV.58)
 *
 *
 */
void __stdcall J2534DRV_thread_join(thread_t thread)
{
	FIXME(":stub\n");
}
/******************************************************************
 *		transfer (J2534DRV.59)
 *
 *
 */
#if 0
__stdcall J2534DRV_transfer()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		translate (J2534DRV.60)
 *
 *
 */
char * __stdcall J2534DRV_translate(uint32_t type)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		translateError (J2534DRV.61)
 *
 *
 */
char * __stdcall J2534DRV_translateError(uint8_t err)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		translateErrorDetail (J2534DRV.62)
 *
 *
 */
char * __stdcall J2534DRV_translateErrorDetail(uint32_t err)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		translateFilterType (J2534DRV.63)
 *
 *
 */
char * __stdcall J2534DRV_translateFilterType(uint32_t type)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		translateIoctl (J2534DRV.64)
 *
 *
 */
char * __stdcall J2534DRV_translateIoctl(uint32_t ioctl)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		translateParam (J2534DRV.65)
 *
 *
 */
char * __stdcall J2534DRV_translateParam(uint32_t param)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		translateProtocol (J2534DRV.66)
 *
 *
 */
char * __stdcall J2534DRV_translateProtocol(uint32_t protocol)
{
	FIXME(":stub\n");
	return (char *) 0;
}
/******************************************************************
 *		tx_counter (J2534DRV.67)
 *
 *
 */
#if 0
__stdcall J2534DRV_tx_counter()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		tx_packet (J2534DRV.68)
 *
 *
 */
#if 0
__stdcall J2534DRV_tx_packet()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		vfprintf (J2534DRV.69)
 *
 *
 */
#if 0
__stdcall J2534DRV_vfprintf()
{
	/* @stub in .spec */
}
#endif
/******************************************************************
 *		vsnprintf (J2534DRV.70)
 *
 *
 */
#if 0
__stdcall J2534DRV_vsnprintf()
{
	/* @stub in .spec */
}
#endif
