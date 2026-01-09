# STM32CAN - WQCAN
The idea of the project is to provide CAN and BDM functionality for Saab Trionic 8 ECU. 

## Currently supported:
* CombiBeckend - For CAN operation work fine.
* SocketCAN - Most of the driver impl is done.
     - can0: CAN2.0/FDCAN - various speed can be defined, loop back, one shoot, listen only, FD mode, FD_NON_ISO, TDC_AUTO
     - can1: SWCAN - 33.3k and 83.3k, loop back, one shoot, listen only.
     - TODO: BERR_REPORTING
* SAE J2534 - Support for Windows and Linux.
     - ISO14230: Tested with Tech2Win and Saab 9-5 DICE,
     - ISO15765: Tested with GenericDiagnosticTool, OpenVehicleDiag and Trionic8 ECU, SW_ISO15765_PS on pin1,
     - CAN: Tested with TrionicCanFlasher, Txloger. Read T8 ECU flash time: 38s ;)

## TODO
* BDM - Needs rework.

## License
Apache2
