/*******************************************************************************

sizedefs.h - definitions of types used in Just4Trionic by Just4pLeisure
(c) 2010 by Sophie Dexter
portions (c) 2009, 2010 by Janis Silins (johnc)

A derivative work based on:  
 * sizedefs.h - define parameter types for BDM package
 * Copyright (C) 1992 by Scott Howard, all rights reserved
 * Permission is hereby granted to freely copy and use this code or derivations thereof
 * as long as no charge is made to anyone for its use
 *
 * this file defines the types BYTE, WORD, and LONG to declare types
 * that match the data sizes used in the target microcontroller(s)
 * BYTE is one byte, WORD is two bytes, LONG is 4 bytes
 * these are all unsigned quantities
 * change these definitions if you are using a compiler with different default sizes
 */

/*******************************************************************************

WARNING: Use at your own risk, sadly this software comes with no guarantees.
This software is provided 'free' and in good faith, but the author does not
accept liability for any damage arising from its use.

*******************************************************************************/

#ifndef __SIZEDEFS_H__
#define __SIZEDEFS_H__

#define BYTE unsigned char
// For MBED / ARM Cortex3 use 'unsigned short' for WORD (2 bytes)
#define WORD unsigned short
#define LONG unsigned long

#define uint8_t unsigned char
#define uint16_t unsigned short
#define uint32_t unsigned long

#endif

/* end of sizedefs.h */
