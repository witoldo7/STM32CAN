/*
 * debug.h
 *
 *  Created on: 18 paź 2022
 *      Author: witold
 */

#ifndef DEBUG_H_
#define DEBUG_H_

#include "chprintf.h"

extern BaseSequentialStream   *GlobalDebugChannel;

#if !defined(DEBUG_TRACE_PRINT)
#define DEBUG_PRINT     TRUE
#endif

#if DEBUG_PRINT
#define DBG_PRNT(fmt, ...)  chprintf(GlobalDebugChannel, fmt, ##__VA_ARGS__)
#else
#define DBG_PRNT(fmt, ...)  do {} while(0)
#endif

#endif /* DEBUG_H_ */
