Driver tested with linux 5.19 and 6.0

# Install
Open terminall and follow:
```sh
sudo cp 52-wqcan.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo make link
sudo make install
sudo modprobe wqcan
```

# Setup interface
All configuration options can be checked by:
```sh
ip link set can1 type can --help 
```

Interface can0 supports FDCAN operation mode and various speed. For example, setup Loopback mode with CAN speed 1Mbit and data 2Mbit. This mode can be used for hardware self-test.
```sh
sudo ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on loopback on 
sudo ifconfig can0 up
```

Interface can1 supports only SWCAN (GMLAN) with speed 33.3k and 83.3k. For example, speed 33.3kbit
```sh
sudo ip link set can1 up type can bitrate 33333 
sudo ifconfig can1 up
```

# Send / Receive 
Receiving in sniffer:
```sh
cansniffer can0 -t 5000 -c
```
Output:
```
32|ms | -- ID -- | data ...     < can0 # l=20 h=100 t=0 slots=35 >
00430 | ---- 0C1 | 00 00 00 00 00 00 00 00 ........
00430 | ---- 0C5 | 00 00 00 00 00 00 00 00 ........
00100 | ---- 100 | 00 00 00 00 00 00 00 00 ........
00012 | ---- 110 | 00 00 00 00 00 00 00 00 ........
00012 | ---- 120 | 00 32 03 20 00 96 00 00 .2. ....
00009 | ---- 140 | C2 03 20 FC DE 00 00 00 .. .....
00010 | ---- 180 | 20 C0 00 A0 60           ...`
00010 | ---- 188 | 0F 00 0F F5 0B 00 00    .......
00060 | ---- 1F5 | 01 00 00 00 00 00       ......
00200 | ---- 200 | 00 00 00 00 00 00 00 00 ........
00019 | ---- 280 | 00 00 00 00 00 00 00 00 ........
```

```sh
cansniffer can1 -t 5000 -c
```
Output:
```
04|ms | -- ID -- | data ...     < can1 # l=20 h=100 t=5000 slots=65 >
05299 | ---- 000 |                         
00097 | ---- 040 | 00 40 DA 30 42 80 00 00 .@.0B...
99999 | ---- 041 | 01 02 C7 C4 00 00 00 00 ........
99999 | ---- 043 | 01 01 C7 C4 00 00 00 00 ........
99999 | ---- 044 | 01 08 C7 C4 00 00 00 00 ........
00100 | ---- 060 | 00 A0 57                ..W
00999 | ---- 070 | 24 08 00 00 00 02 0F F5 $.......
03168 | ---- 090 | 17                      .
00098 | ---- 091 | 00 20 00 00 00          . ...
05753 | ---- 100 |                         
00100 | ---- 108 | 01 00 00 00 00 00 00    .......
```

Frame can be send by:
```sh
cansend can0 123\#1122334455667788
```
