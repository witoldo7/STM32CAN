/*******************************************************************************

common.h - information and definitions needed by parts of Just4Trionic
(c) by 2010 Sophie Dexter
portions (c) 2009, 2010 by Janis Silins (johnc)

********************************************************************************

WARNING: Use at your own risk, sadly this software comes with no guarantees.
This software is provided 'free' and in good faith, but the author does not
accept liability for any damage arising from its use.

*******************************************************************************/

#ifndef __BDM_H__
#define __BDM_H__

#include "ch.h"
#include "sizedefs.h"
#include "strings.h"

// build configuration
#define IGNORE_VCC_PIN            ///< uncomment to ignore the VCC pin

// MCU status macros
#ifndef IGNORE_VCC_PIN
#define IS_CONNECTED    palReadLine(LINE_CONNECTED)
#else
#define IS_CONNECTED    true
#endif    // IGNORE_VCC_PIN

#define IN_BDM           palReadLine(LINE_FREZE)
#define IS_RUNNING       (palReadLine(LINE_RESET) && !IN_BDM)

// bit macros
#define SETBIT(x,y)         (x |= (y))                ///< set bit y in byte x
#define CLEARBIT(x,y)       (x &= (~(y)))            ///< clear bit y in byte x
#define CHECKBIT(x,y)       (((x) & (y)) == (y))    ///< check bit y in byte x

// command return flags and character constants
#define TERM_OK             13            ///< command terminator or success flag
#define TERM_ERR            7            ///< error flag
#define TERM_BREAK          0x1b        ///< command break flag
#define ERR_COUNT           255            ///< maximum error cycles

// structure for command address/value pairs
struct mempair_t {
    uint32_t addr;            ///< target address
    uint16_t val;            ///< word value
};

// word write algorithm (29Fxxx)
static const struct mempair_t am29_write [] = {
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0xa0a0},
};

// chip erase algorithms
static const struct mempair_t am29_erase [] = {
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0x8080},
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0x1010}
};

// reset algorithms
static const struct mempair_t am29_reset [] = {
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0xf0f0},
};

// chip id algorithms
static const struct mempair_t am29_id [] = {
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0x9090},
};

// ;-)
static const struct mempair_t flash_tag [] = {
    {0x7fe00, 0xFF4A}, {0x7fe02, 0x7573}, {0x7fe04, 0x7434}, {0x7fe06, 0x704C},
    {0x7fe08, 0x6569}, {0x7fe0a, 0x7375}, {0x7fe0c, 0x7265}, {0x7fe0e, 0x3B29},
};

uint8_t erase_flash(const char* flash_type, const uint32_t* start_addr,
                    const uint32_t* end_addr);
bool flash_am28(const uint32_t* addr, uint16_t value);
bool flash_am29(const uint32_t* addr, uint16_t value);
bool reset_am28(void);
bool reset_am29(void);

#endif              // __BDM_H__
