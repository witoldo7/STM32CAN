// SPDX-License-Identifier: MIT

/*
 * WQCAN J2534 API Library.
 * Copyright (c) 2023 Witold Olechowski.
 *
 */

#ifndef J2534DRV_H_
#define J2534DRV_H_
#include "j2534def.h"
#include "windef.h"
#include "winbase.h"
#include "winnt.h"


#define J2534DLL_API extern __declspec(dllexport)
#define PTAPI __stdcall

//
// J2534-1 v04.04 Function Prototypes
//
J2534DLL_API uint32_t PTAPI	PassThruOpen(void *pName, uint32_t *pDeviceID);
J2534DLL_API uint32_t PTAPI	PassThruClose(uint32_t DeviceID);
J2534DLL_API uint32_t PTAPI	PassThruConnect(uint32_t DeviceID, uint32_t ProtocolID, uint32_t Flags, uint32_t BaudRate, uint32_t *pChannelID);
J2534DLL_API uint32_t PTAPI	PassThruDisconnect(uint32_t ChannelID);
J2534DLL_API uint32_t PTAPI	PassThruReadMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout);
J2534DLL_API uint32_t PTAPI	PassThruWriteMsgs(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout);
J2534DLL_API uint32_t PTAPI	PassThruStartPeriodicMsg(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pMsgID, uint32_t TimeInterval);
J2534DLL_API uint32_t PTAPI	PassThruStopPeriodicMsg(uint32_t ChannelID, uint32_t MsgID);
J2534DLL_API uint32_t PTAPI	PassThruStartMsgFilter(uint32_t ChannelID, uint32_t FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, uint32_t *pFilterID);
J2534DLL_API uint32_t PTAPI	PassThruStopMsgFilter(uint32_t ChannelID, uint32_t FilterID);
J2534DLL_API uint32_t PTAPI	PassThruSetProgrammingVoltage(uint32_t DeviceID, uint32_t PinNumber, uint32_t Voltage);
J2534DLL_API uint32_t PTAPI	PassThruReadVersion(uint32_t DeviceID, char *pFirmwareVersion, char *pDllVersion, char *pApiVersion);
J2534DLL_API uint32_t PTAPI	PassThruGetLastError(char *pErrorDescription);
J2534DLL_API uint32_t PTAPI	PassThruIoctl(uint32_t ChannelID, uint32_t IoctlID, void *pInput, void *pOutput);
J2534DLL_API uint32_t PTAPI PassThruGetNextCarDAQ(char **name, unsigned long *version, char **addr);
//
// J2534-1 v04.04 Function Typedefs
// These function typedefs allow simpler use of the J2534 API by
// allowing you to do things like this:
// PTCONNECT	pPassThruConnectFunc = GetProcAddress(hModule, "PassThruConnect");
// if (pPassThruConnectFunc == NULL)
//     return FALSE;
// pPassThruConnectFunc(DeviceID, CAN, CAN_29BIT_ID, 500000, &ChannelID);
//
typedef uint32_t (PTAPI *PTOPEN)(void *pName, uint32_t *pDeviceID);
typedef uint32_t (PTAPI *PTCLOSE)(uint32_t DeviceID);
typedef uint32_t (PTAPI *PTCONNECT)(uint32_t DeviceID, uint32_t ProtocolID, uint32_t Flags, uint32_t BaudRate, uint32_t *pChannelID);
typedef uint32_t (PTAPI *PTDISCONNECT)(uint32_t ChannelID);
typedef uint32_t (PTAPI *PTREADMSGS)(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout);
typedef uint32_t (PTAPI *PTWRITEMSGS)(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pNumMsgs, uint32_t Timeout);
typedef uint32_t (PTAPI *PTSTARTPERIODICMSG)(uint32_t ChannelID, PASSTHRU_MSG *pMsg, uint32_t *pMsgID, uint32_t TimeInterval);
typedef uint32_t (PTAPI *PTSTOPPERIODICMSG)(uint32_t ChannelID, uint32_t MsgID);
typedef uint32_t (PTAPI *PTSTARTMSGFILTER)(uint32_t ChannelID, uint32_t FilterType, PASSTHRU_MSG *pMaskMsg, PASSTHRU_MSG *pPatternMsg, PASSTHRU_MSG *pFlowControlMsg, uint32_t *pFilterID);
typedef uint32_t (PTAPI *PTSTOPMSGFILTER)(uint32_t ChannelID, uint32_t FilterID);
typedef uint32_t (PTAPI *PTSETPROGRAMMINGVOLTAGE)(uint32_t DeviceID, uint32_t PinNumber, uint32_t Voltage);
typedef uint32_t (PTAPI *PTREADVERSION)(uint32_t DeviceID, char *pFirmwareVersion, char *pDllVersion, char *pApiVersion);
typedef uint32_t (PTAPI *PTGETLASTERROR)(char *pErrorDescription);
typedef uint32_t (PTAPI *PTIOCTL)(uint32_t ChannelID, uint32_t IoctlID, void *pInput, void *pOutput);
typedef uint32_t (PTAPI *PASSTRUGETNEXTCARDAQ)(char **name, unsigned long *version, char **addr);

#endif /* J2534DRV_H_ */
