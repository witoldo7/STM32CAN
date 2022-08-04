// SPDX-License-Identifier: MIT
/* 
 * SocketCAN driver for CombiAdapter/WQCAN.
 * Copyright (c) 2022 Witold Olechowski
 * 
 */ 

#ifndef __UTILS_H
#define __UTILS_H

#include "j2534.h"

#define MAX_LEN	80

char* parsemsg(PASSTHRU_MSG *msg);

void check_debug_log(void);

#endif // __UTILS_H
