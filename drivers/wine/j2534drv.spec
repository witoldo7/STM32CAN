# Generated from j2534drv.dll by winedump

1 stdcall DllMain( ptr long ptr ) J2534DRV_DllMain
2 stdcall Lock( long long ) J2534DRV_Lock
3 stdcall LockRx() J2534DRV_LockRx
4 stdcall PASSTHRU_MSG_To_CANTxFrame( ptr ptr ) J2534DRV_PASSTHRU_MSG_To_CANTxFrame
5 stdcall PassThruClose( long ) J2534DRV_PassThruClose
6 stdcall PassThruConnect( long long long long ptr ) J2534DRV_PassThruConnect
7 stdcall PassThruDisconnect( long ) J2534DRV_PassThruDisconnect
8 stdcall PassThruGetLastError( str ) J2534DRV_PassThruGetLastError
9 stdcall PassThruIoctl( long long ptr ptr ) J2534DRV_PassThruIoctl
10 stdcall PassThruOpen() J2534DRV_PassThruOpen
11 stdcall PassThruReadMsgs( long ptr ptr long ) J2534DRV_PassThruReadMsgs
12 stdcall PassThruReadVersion( long str str str ) J2534DRV_PassThruReadVersion
13 stdcall PassThruSetProgrammingVoltage( long long long ) J2534DRV_PassThruSetProgrammingVoltage
14 stdcall PassThruStartMsgFilter( long long ptr ptr ptr ptr ) J2534DRV_PassThruStartMsgFilter
15 stdcall PassThruStartPeriodicMsg( long ptr ptr long ) J2534DRV_PassThruStartPeriodicMsg
16 stdcall PassThruStopMsgFilter( long long ) J2534DRV_PassThruStopMsgFilter
17 stdcall PassThruStopPeriodicMsg( long long ) J2534DRV_PassThruStopPeriodicMsg
18 stdcall PassThruWriteMsgs( long ptr ptr long ) J2534DRV_PassThruWriteMsgs
19 stdcall Unlock( long long ) J2534DRV_Unlock
20 stdcall UnlockRx() J2534DRV_UnlockRx
21 stub __local_stdio_printf_options
22 stub _snprintf_s
23 stub _vfprintf_l
24 stub _vsnprintf_l
25 stub _vsnprintf_s_l
26 stub _vsprintf_l
27 stdcall check_debug_log() J2534DRV_check_debug_log
28 stdcall clearQueue() J2534DRV_clearQueue
29 stdcall convertPacketToPMSG( ptr long ptr ) J2534DRV_convertPacketToPMSG
30 stdcall coppyMessages( ptr long ) J2534DRV_coppyMessages
31 stdcall covertPacketToBuffer( ptr ptr ) J2534DRV_covertPacketToBuffer
32 stdcall deQueue( ptr ) J2534DRV_deQueue
33 stub do_exit
34 stdcall enQueue( ptr ) J2534DRV_enQueue
35 stub fprintf
36 stdcall getLastError() J2534DRV_getLastError
37 varargs last_error( str ) J2534DRV_last_error
38 stdcall log_add_callback( long ptr long ) J2534DRV_log_add_callback
39 stdcall log_add_fp( ptr long ) J2534DRV_log_add_fp
40 stdcall log_level_string( long ) J2534DRV_log_level_string
41 varargs log_log( long str long str ) J2534DRV_log_log
42 stdcall log_set_level( long ) J2534DRV_log_set_level
43 stdcall log_set_lock( long ptr ) J2534DRV_log_set_lock
44 stdcall log_set_quiet( long ) J2534DRV_log_set_quiet
45 stdcall parsemsg( ptr ) J2534DRV_parsemsg
46 stdcall parsemsgb( ptr str ) J2534DRV_parsemsgb
47 stub resp_packet
48 stub rx_counter
49 stdcall semaphore_create( str ) J2534DRV_semaphore_create
50 stdcall semaphore_destroy( long ) J2534DRV_semaphore_destroy
51 stdcall semaphore_give( long ) J2534DRV_semaphore_give
52 stdcall semaphore_take( long ) J2534DRV_semaphore_take
53 stdcall semaphore_take_timeout( long long ) J2534DRV_semaphore_take_timeout
54 stdcall sizeQueue() J2534DRV_sizeQueue
55 stdcall sleep_us( long ) J2534DRV_sleep_us
56 stub sprintf
57 stub thread_create
58 stdcall thread_join( long ) J2534DRV_thread_join
59 stub transfer
60 stdcall translate( long ) J2534DRV_translate
61 stdcall translateError( long ) J2534DRV_translateError
62 stdcall translateErrorDetail( long ) J2534DRV_translateErrorDetail
63 stdcall translateFilterType( long ) J2534DRV_translateFilterType
64 stdcall translateIoctl( long ) J2534DRV_translateIoctl
65 stdcall translateParam( long ) J2534DRV_translateParam
66 stdcall translateProtocol( long ) J2534DRV_translateProtocol
67 stub tx_counter
68 stub tx_packet
69 stub vfprintf
70 stub vsnprintf
