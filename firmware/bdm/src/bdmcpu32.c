/*******************************************************************************

 bdmcpu32.cpp
 (c) 2010 by Sophie Dexter

 Generic BDM functions for Just4Trionic by Just4pLeisure

 A derivative work based on:
 //-----------------------------------------------------------------------------
 //    CAN/BDM adapter firmware
 //    (C) Janis Silins, 2010
 //    $id$
 //-----------------------------------------------------------------------------

 ********************************************************************************

 WARNING: Use at your own risk, sadly this software comes with no guarantees.
 This software is provided 'free' and in good faith, but the author does not
 accept liability for any damage arising from its use.

 *******************************************************************************/

#include "bdmcpu32.h"

#include "../inc/bdm.h"
#include "ch.h"

// constants
#define MCU_SETTLE_TIME     10        ///< delay to let MCU switch modes, ms
#define CMD_BIT_COUNT       17        ///< command size, bits

// BDM commands
#define BDM_NOP             0x0000    ///< no-op
#define BDM_GO              0x0c00    ///< resume execution
#define BDM_WRITE           0x1800    ///< write memory
#define BDM_READ            0x1900    ///< read memory 
#define BDM_WSREG           0x2480    ///< write system register
#define BDM_RSREG           0x2580    ///< read system register
#define BDM_RDREG           0x2180    ///< read A/D register
#define BDM_WRREG           0x2080    ///< write A/D register
#define BDM_DUMP            0x1d00    ///< dump memory
#define BDM_FILL            0x1c00    ///< fill memory
#define BDM_CALL            0x0800    ///< function call
#define BDM_RST             0x0400    ///< reset

// system registers
#define SREG_RPC            0x0
#define SREG_PCC            0x1
#define SREG_SR             0xb
#define SREG_USP            0xc
#define SREG_SSP            0xd
#define SREG_SFC            0xe
#define SREG_DFC            0xf
#define SREG_ATEMP          0x8
#define SREG_FAR            0x9
#define SREG_VBR            0xa

// BDM responses
#define BDM_CMDCMPLTE       0x0000ffff    ///< command complete
#define BDM_NOTREADY        0x00010000    ///< response not ready
#define BDM_BERR            0x00010001    ///< error
#define BDM_ILLEGAL         0x0001ffff    ///< illegal command

// BDM data sizes
#define BDM_BYTESIZE        0x00        ///< byte
#define BDM_WORDSIZE        0x40        ///< word (2 bytes)
#define BDM_LONGSIZE        0x80        ///< long word (4 bytes)

static uint32_t bdm_response = 0;      ///< result of BDM read/write operation

// private functions
bool bdm_read(uint32_t *result, uint16_t cmd, const uint32_t *addr);
bool bdm_write(const uint32_t *addr, uint16_t cmd, const uint32_t *value);
void bdm_clk_slow(uint16_t value, uint8_t num_bits);
void bdm_clk_fast(uint16_t value, uint8_t num_bits);
void bdm_clk_turbo(uint16_t value, uint8_t num_bits);
void bdm_clk_nitrous(uint16_t value, uint8_t num_bits);
void (*bdm_clk)(uint16_t value, uint8_t num_bits) = bdm_clk_slow;
void bdm_clear(void);

//-----------------------------------------------------------------------------
/**
 Stops target MCU and puts into background debug mode (BDM).

 @return                        status flag
 */
uint8_t stop_chip(void) {
  // not connected
  if (!IS_CONNECTED) {
    return TERM_ERR;
  }

  // pull BKPT low to enter background mode (the pin must remain in output mode,
  // otherwise the target will pull it high and we'll lose the first DSO bit)
  palWriteLine(LINE_BKPT, 0);
  palSetLineMode(LINE_BKPT, PAL_MODE_OUTPUT_PUSHPULL);
  systime_t start = chVTGetSystemTimeX();
  systime_t end = chTimeAddX(start, TIME_MS2I(MCU_SETTLE_TIME));
  while (!IN_BDM & chTimeIsInRangeX(chVTGetSystemTimeX(), start, end)) {
    __asm("nop");
  }

  // check if succeeded
  if (!IN_BDM) {
    palSetLineMode(LINE_BKPT, PAL_MODE_INPUT);
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Forces hardware reset on target MCU and lets it run.

 @return                        status flag
 */
uint8_t reset_chip(void) {
  // not connected
  if (!IS_CONNECTED) {
    return TERM_ERR;
  }

  palSetLineMode(LINE_BKPT, PAL_MODE_INPUT);
  // push RESET low
  palWriteLine(LINE_RESET, 0);
  palSetLineMode(LINE_RESET, PAL_MODE_OUTPUT_PUSHPULL);

  // wait for MCU to settle
  chThdSleepMilliseconds(MCU_SETTLE_TIME);
  // rising edge on RESET line
  palWriteLine(LINE_RESET, 1);
  // wait for MCU to settle
  chThdSleepMilliseconds(MCU_SETTLE_TIME);
  palSetLineMode(LINE_RESET, PAL_MODE_INPUT);

  // check if succeeded
  return IS_RUNNING ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Starts target MCU from the specified address. If address is 0, execution
 begins at the current address in program counter.

 @param            addr        start address

 @return                        status flag
 */
uint8_t run_chip(const uint32_t *addr) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // set program counter
  if ((*addr > 0) && sysreg_write(SREG_RPC, addr) != TERM_OK) {
    return TERM_ERR;
  }
  // resume MCU
  bdm_clk(BDM_GO, CMD_BIT_COUNT);

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Resets target MCU and stops execution on first instruction fetch.

 @return                        status flag
 */
uint8_t restart_chip(void) {
  // not connected
  if (!IS_CONNECTED) {
    return TERM_ERR;
  }

  // pull BKPT low to enter background mode (the pin must remain an output,
  // otherwise the target will pull it high and we'll lose the first DSO bit)
  palWriteLine(LINE_BKPT, 0);
  palSetLineMode(LINE_BKPT, PAL_MODE_OUTPUT_PUSHPULL);
  // push RESET low
  palWriteLine(LINE_RESET, 0);
  palSetLineMode(LINE_RESET, PAL_MODE_OUTPUT_PUSHPULL);
  // wait for target MCU to settle
  chThdSleepMilliseconds(10*MCU_SETTLE_TIME);
  // rising edge on RESET line
  palWriteLine(LINE_RESET, 1);
  // wait for target MCU to settle
  chThdSleepMilliseconds(10*MCU_SETTLE_TIME);
  palSetLineMode(LINE_RESET, PAL_MODE_INPUT);
  // check if succeeded
  if (!IN_BDM) {
    // set BKPT back as input and fail
    palSetLineMode(LINE_BKPT, PAL_MODE_INPUT);
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Sends GO command word to target MCU, then triggers breakpoint on first
 instruction fetch.

 @return                        status flag
 */
uint8_t step_chip(void) {
  // not connected
  if (!IS_CONNECTED) {
    return TERM_ERR;
  }

  // resume MCU
  bdm_clk(BDM_GO, CMD_BIT_COUNT);

  // pull BKPT low to enter background mode (the pin must remain an output,
  // otherwise the target pulls it high and we lose the first DSO bit)
  palWriteLine(LINE_BKPT, 0);
  palSetLineMode(LINE_BKPT, PAL_MODE_OUTPUT_PUSHPULL);

  // wait for target MCU to settle
  chThdSleepMilliseconds(MCU_SETTLE_TIME);

  // check if succeeded
  if (!IN_BDM) {
    // set BKPT back as input and fail
    palSetLineMode(LINE_BKPT, PAL_MODE_INPUT);
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Returns byte from the specified memory location; MCU must be in
 background mode.

 @param        result            value (out)
 @param        addr            source address

 @return                        status flag
 */
uint8_t memread_byte(uint8_t *result, const uint32_t *addr) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // read byte
  if (!bdm_read((uint32_t*)result, BDM_READ, addr)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Returns word (2 bytes) from the specified memory location. Address must be
 word-aligned and MCU must be in background mode.

 @param        result            value (out)
 @param        addr            source address

 @return                        status flag
 */
uint8_t memread_word(uint16_t *result, const uint32_t *addr) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // read word
  if (!bdm_read((uint32_t*)result, BDM_READ + BDM_WORDSIZE, addr)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Returns long word (4 bytes) from the specified memory location. Address
 must be word-aligned and target MCU must be in background mode.

 @param            result            value
 @param            addr            source address

 @return                            status flag
 */
uint8_t memread_long(uint32_t *result, const uint32_t *addr) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  //  read long word
  if (!bdm_read(result, BDM_READ + BDM_LONGSIZE, addr)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Dumps byte from the specified memory location; MCU must be in background
 mode. Any memread_*() function must be called beforehand to set the
 initial address.

 @param        value            result (out)

 @return                        status flag
 */
uint8_t memdump_byte(uint8_t *result) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // dump byte
  if (!bdm_read((uint32_t*)result, BDM_DUMP, NULL)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 dumps word from the specified memory location; MCU must be in background
 mode. Any memread_*() function must be called beforehand to set the
 initial address.

 @param        value            result (out)

 @return                        status flag
 */
uint8_t memdump_word(uint16_t *result) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // dump word
  if (!bdm_read((uint32_t*)result, BDM_DUMP + BDM_WORDSIZE, NULL)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Dumps long word  from the specified memory location; MCU must be in
 background mode. Any memread_*() function must be called beforehand to set
 the initial address.

 @param        value            result (out)

 @return                        status flag
 */
uint8_t memdump_long(uint32_t *result) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // dump long word
  if (!bdm_read(result, BDM_DUMP + BDM_LONGSIZE, NULL)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Writes byte to the specified memory location; MCU must be in background
 mode.

 @param        addr            destination address
 @param        value            value

 @return                        status flag
 */
uint8_t memwrite_byte(const uint32_t *addr, uint8_t value) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // write byte
  if (!bdm_write(addr, BDM_WRITE, (uint32_t*)&value)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Writes word to the specified memory location. Address must be word-aligned
 and MCU must be in background mode.

 @param        addr            memory address
 @param        value            value

 @return                        status flag
 */
uint8_t memwrite_word(const uint32_t *addr, uint16_t value) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // write word
  if (!bdm_write(addr, BDM_WRITE + BDM_WORDSIZE, (uint32_t*)&value)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Writes long word to the specified memory location. Address must be
 word-aligned and target MCU must be in background mode.

 @param        addr            memory address
 @param        value            value

 @return                        status flag
 */
uint8_t memwrite_long(const uint32_t *addr, const uint32_t *value) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // write long word
  if (!bdm_write(addr, BDM_WRITE + BDM_LONGSIZE, value)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Writes byte to the current memory location; MCU must be in background
 mode. Any memwrite_*() function must be called beforehand to set the
 current address.

 @param        value            value

 @return                        status flag
 */
uint8_t memfill_byte(uint8_t value) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // fill byte
  if (!bdm_write(NULL, BDM_FILL, (uint32_t*)&value)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Writes word to the specified memory location; MCU must be in background
 mode. Any memwrite_*() function must be called beforehand to set the
 initial address.

 @param        value            value

 @return                        status flag
 */
uint8_t memfill_word(uint16_t value) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // fill word
  if (!IN_BDM || !bdm_write(NULL, BDM_FILL + BDM_WORDSIZE, (uint32_t*)&value)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Writes long word to the specified memory location; MCU must be in background
 mode. Any memwrite_*() function must be called beforehand to set the
 initial address.

 @param        value            value

 @return                        status flag
 */
uint8_t memfill_long(const uint32_t *value) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // fill long word
  if (!bdm_write(NULL, BDM_FILL + BDM_LONGSIZE, value)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Issues a read byte command to MCU.

 @param        addr        address (optional)

 @return                   status flag
 */
uint8_t memread_byte_cmd(const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;

  // write command code
  if (!bdm_command(BDM_READ + BDM_BYTESIZE))
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Issues a read word command to MCU.

 @param        addr        address (optional)

 @return                   status flag
 */
uint8_t memread_word_cmd(const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;

  // write command code
  bdm_clk(BDM_READ + BDM_WORDSIZE, CMD_BIT_COUNT);
  if (bdm_response > BDM_CMDCMPLTE)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Issues a read long command to MCU.

 @param        addr        address (optional)

 @return                   status flag
 */
uint8_t memread_long_cmd(const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;

  // write command code
  bdm_clk(BDM_READ + BDM_LONGSIZE, CMD_BIT_COUNT);
  if (bdm_response > BDM_CMDCMPLTE)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }

  return TERM_OK;
}
//-----------------------------------------------------------------------------
/**
 Issues a write byte command to MCU.

 @param        addr        address (optional)

 @return                   status flag
 */
uint8_t memwrite_byte_cmd(const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;

  // write command code
  if (!bdm_command(BDM_WRITE + BDM_BYTESIZE))
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Issues a write word command to MCU.

 @param        addr        address (optional)

 @return                   status flag
 */
uint8_t memwrite_word_cmd(const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;

  // write command code
  bdm_clk(BDM_WRITE + BDM_WORDSIZE, CMD_BIT_COUNT);
  if (bdm_response > BDM_CMDCMPLTE)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Issues a write long command to MCU.

 @param        addr        address (optional)

 @return                   status flag
 */
uint8_t memwrite_long_cmd(const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;

  // write command code
  bdm_clk(BDM_WRITE + BDM_LONGSIZE, CMD_BIT_COUNT);
  if (bdm_response > BDM_CMDCMPLTE)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Gets a byte from the MCU (follows a previously sent read or dump word cmd)
 Sends a READ_BYTE command so that commands overlap

 @param        result        read result (out)
 addr          address (optional)

 @return                     status flag
 */
uint8_t memread_read_byte(uint8_t *result, const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }
  // receive the response byte
  return (bdm_get((uint32_t*)result, BDM_BYTESIZE, BDM_READ + BDM_BYTESIZE)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Gets a byte from the MCU (follows a previously sent read or dump word cmd)
 Sends a WRITE_BYTE command so that commands overlap

 @param        result        read result (out)
 addr          address (optional)

 @return                     status flag
 */
uint8_t memread_write_byte(uint8_t *result, const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }
  // receive the response byte
  return (bdm_get((uint32_t*)result, BDM_BYTESIZE, BDM_WRITE + BDM_BYTESIZE)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Gets a byte from the MCU (follows a previously sent read or dump word cmd)
 Sends a BDM_NOP command to end a sequence of overlapping commands

 @param        result        read result (out)
 addr          address (optional)

 @return                     status flag
 */
uint8_t memread_nop_byte(uint8_t *result, const uint32_t *addr) {

  if (!IN_BDM)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }
  // receive the response byte
  return (bdm_get((uint32_t*)result, BDM_BYTESIZE, BDM_NOP)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Writes a byte to the MCU (follows a previously sent write or fill word cmd)
 Sends a WRITE_BYTE command so that commands overlap

 @param        addr          address (optional)
 value         value to write

 @return                     status flag
 */
uint8_t memwrite_write_byte(const uint32_t *addr, uint8_t value) {

  if (!IN_BDM)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }
  // write the value
  if (!bdm_put((uint32_t*)&value, BDM_BYTESIZE))
    return TERM_ERR;
  // wait until MCU responds
  return (bdm_ready(BDM_WRITE + BDM_BYTESIZE)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Writes a byte to the MCU (follows a previously sent write or fill word cmd)
 Sends a READ_BYTE command so that commands overlap

 @param        addr          address (optional)
 value         value to write

 @return                     status flag
 */
uint8_t memwrite_read_byte(const uint32_t *addr, uint8_t value) {

  if (!IN_BDM)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }
  // write the value
  if (!bdm_put((uint32_t*)&value, BDM_BYTESIZE))
    return TERM_ERR;
  // wait until MCU responds
  return (bdm_ready(BDM_READ + BDM_BYTESIZE)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Writes a byte to the MCU (follows a previously sent write or fill word cmd)
 Sends a BDM_NOP command to end a sequence of overlapping commands

 @param        addr          address (optional)
 value         value to write

 @return                     status flag
 */
uint8_t memwrite_nop_byte(const uint32_t *addr, uint8_t value) {

  if (!IN_BDM)
    return TERM_ERR;
  // write the optional address
  if (addr) {
    if (!bdm_address(addr))
      return TERM_ERR;
  }
  // write the value
  if (!bdm_put((uint32_t*)&value, BDM_BYTESIZE))
    return TERM_ERR;
  // wait until MCU responds
  return (bdm_ready(BDM_NOP)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Writes 2 words to the same address
 The BDM commands are overlapped to make things a bit faster
 A BDM_NOP command is then sent to end the sequence of overlapping commands

 @param        addr          address
 value1, 2     values to write

 @return                     status flag
 */
uint8_t memwrite_word_write_word(const uint32_t *addr, const uint16_t value1, const uint16_t value2) {
  return (IN_BDM && bdm_command(BDM_WRITE + BDM_WORDSIZE) &&        // write command code
      bdm_address(addr) &&                            // write the address
      bdm_put((uint32_t*)&value1, BDM_WORDSIZE) &&    // write the first value
      bdm_ready(BDM_WRITE + BDM_WORDSIZE) &&          // wait until MCU responds and overlap the next write command
      bdm_address(addr) &&                            // write the address (same address for second word)
      bdm_put((uint32_t*)&value2, BDM_WORDSIZE) &&    // write the second value
      bdm_ready(BDM_NOP)) ? TERM_OK : TERM_ERR;       // wait until MCU responds
}

//-----------------------------------------------------------------------------
/**
 Writes a word then reads back a result from the same address
 The BDM commands are overlapped to make things a bit faster
 A BDM_NOP command is then sent to end the sequence of overlapping commands

 @param        result        read result (out)
 addr          address
 value         value to write

 @return                     status flag
 */

uint8_t memwrite_word_read_word(uint16_t *result, const uint32_t *addr, const uint16_t value) {
  return (IN_BDM && bdm_command(BDM_WRITE + BDM_WORDSIZE) &&        // write command code
      bdm_address(addr) &&                            // write the address
      bdm_put((uint32_t*)&value, BDM_WORDSIZE) &&     // write the value
      bdm_ready(BDM_READ + BDM_WORDSIZE) &&           // wait until MCU responds and overlap the next read command
      bdm_address(addr) &&                            // write the address (same address for reading the result)
      bdm_get((uint32_t*)result, BDM_WORDSIZE, BDM_NOP)) ? TERM_OK : TERM_ERR;    // receive the response word
}

//-----------------------------------------------------------------------------
/**
 Gets a word from the MCU (follows a previously sent read or dump word cmd)
 Sends a DUMP_WORD command so that dump commands overlap

 @param        result        read result (out)

 @return                     status flag
 */
uint8_t memget_word(uint16_t *result) {

  if (!IN_BDM)
    return TERM_ERR;
  // receive the response word
  return (bdm_get((uint32_t*)result, BDM_WORDSIZE, BDM_DUMP + BDM_WORDSIZE)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Gets a long from the MCU (follows a previously sent read or dump long cmd)
 Sends a DUMP_LONG command so that dump commands overlap

 @param        result        read result (out)

 @return                     status flag
 */
uint8_t memget_long(uint32_t *result) {

  if (!IN_BDM)
    return TERM_ERR;
  // receive the response words
  return (bdm_get(result, BDM_LONGSIZE, BDM_DUMP + BDM_LONGSIZE)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
 Reads value from system register.

 @param        result            register value (out)
 @param        reg                register

 @return                        status flag
 */
uint8_t sysreg_read(uint32_t *result, uint8_t reg) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // read register
  if (!bdm_read(result, BDM_RSREG + reg, NULL)) {
    // clear the interface and fail
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Writes value to system register.

 @param        reg                register
 @param        value            register value

 @return                        status flag
 */
uint8_t sysreg_write(uint8_t reg, const uint32_t *value) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // write register
  if (!bdm_write(NULL, BDM_WSREG + reg, value)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Reads value from A/D register.

 @param        result            register value (out)
 @param        reg                register

 @return                        status flag
 */
uint8_t adreg_read(uint32_t *result, uint8_t reg) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // read register
  if (!bdm_read(result, BDM_RDREG + reg, NULL)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }
  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Writes value to A/D register.

 @param        reg                register
 @param        value            register value

 @return                        status flag
 */
uint8_t adreg_write(uint8_t reg, const uint32_t *value) {
  // check state
  if (!IN_BDM) {
    return TERM_ERR;
  }

  // write register
  if (!bdm_write(NULL, BDM_WRREG + reg, value)) {
    // clear the interface and fail
    bdm_clear();
    return TERM_ERR;
  }

  return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
 Issues a read command to MCU.

 @param        result        read result (out)
 @param        cmd            command sequence
 @param        addr        address (optional)

 @return                    succ / fail
 */
bool bdm_read(uint32_t *result, uint16_t cmd, const uint32_t *addr) {
  *result = 0;

  // write command code
  bdm_clk(cmd, CMD_BIT_COUNT);
  if (bdm_response > BDM_CMDCMPLTE) {
    return false;
  }

  // write the optional address
  if (addr) {
    // first word
    bdm_clk((uint16_t)(*addr >> 16), CMD_BIT_COUNT);
    if (bdm_response > BDM_NOTREADY) {
      return false;
    }
    // second word
    bdm_clk((uint16_t)(*addr), CMD_BIT_COUNT);
    if (bdm_response > BDM_NOTREADY) {
      return false;
    }
  }

  // receive response words
  uint8_t wait_cnt;
  for (uint8_t curr_word = 0; curr_word < ((cmd & BDM_LONGSIZE) ? 2 : 1); ++curr_word) {
    // wait while MCU prepares the response
    wait_cnt = ERR_COUNT;
    do {
      bdm_clk(BDM_NOP, CMD_BIT_COUNT);
    } while (bdm_response == BDM_NOTREADY && --wait_cnt > 0);

    // save the result
    if (bdm_response < BDM_NOTREADY) {
      (*result) <<= 16;
      (*result) |= bdm_response;
    }
    else {
      // result was not received
      return false;
    }
  }

  return true;
}

//-----------------------------------------------------------------------------
/**
 Issues a write command to MCU.

 @param        num_words        number of additional command words
 @param        cmd                command sequence

 @return                        succ / fail
 */
bool bdm_write(const uint32_t *addr, uint16_t cmd, const uint32_t *value) {
  // write command code
  bdm_clk(cmd, CMD_BIT_COUNT);
  if (bdm_response > BDM_NOTREADY) {
    return false;
  }

  // write the optional address
  if (addr) {
    // first word
    bdm_clk((uint16_t)((*addr) >> 16), CMD_BIT_COUNT);
    if (bdm_response > BDM_NOTREADY) {
      return false;
    }
    // second word
    bdm_clk((uint16_t)(*addr), CMD_BIT_COUNT);
    if (bdm_response > BDM_NOTREADY) {
      return false;
    }
  }

  // write the value
  if (cmd & BDM_LONGSIZE) {
    bdm_clk((uint16_t)((*value) >> 16), CMD_BIT_COUNT);
    if (bdm_response > BDM_NOTREADY) {
      return false;
    }
  }
  bdm_clk((uint16_t)(*value), CMD_BIT_COUNT);
  if (bdm_response > BDM_NOTREADY) {
    return false;
  }

  // wait until MCU responds
  uint8_t wait_cnt = ERR_COUNT;
  do {
    // read response
    bdm_clk(BDM_NOP, CMD_BIT_COUNT);
  } while (bdm_response == BDM_NOTREADY && --wait_cnt > 0);

  // check if command succeeded
  return (bdm_response == BDM_CMDCMPLTE);
}

bool bdm_command(uint16_t cmd) {
  // write command code
  bdm_clk(cmd, CMD_BIT_COUNT);
  return (bdm_response > BDM_NOTREADY) ? false : true;
}

bool bdm_address(const uint32_t *addr) {
  // write an address
  // first word
  bdm_clk((uint16_t)((*addr) >> 16), CMD_BIT_COUNT);
  if (bdm_response > BDM_NOTREADY) {
    return false;
  }
  // second word
  bdm_clk((uint16_t)(*addr), CMD_BIT_COUNT);
  return (bdm_response > BDM_NOTREADY) ? false : true;
}

bool bdm_get(uint32_t *result, uint8_t size, uint16_t next_cmd) {
  // receive response words
  *result = 0;
  uint8_t wait_cnt;
  for (uint8_t curr_word = 0; curr_word < ((size & BDM_LONGSIZE) ? 2 : 1); ++curr_word) {
    // wait while MCU prepares the response
    wait_cnt = ERR_COUNT;
    do {
      bdm_clk(next_cmd, CMD_BIT_COUNT);
    } while (bdm_response == BDM_NOTREADY && --wait_cnt > 0);

    // save the result
    if (bdm_response < BDM_NOTREADY) {
      (*result) <<= 16;
      (*result) |= bdm_response;
    }
    else {
      // result was not received
      return false;
    }
  }
  return true;
}

bool bdm_put(const uint32_t *value, uint8_t size) {
  // write the value
  if (size & BDM_LONGSIZE) {
    bdm_clk((uint16_t)((*value) >> 16), CMD_BIT_COUNT);
    if (bdm_response > BDM_NOTREADY) {
      return false;
    }
  }
  bdm_clk((uint16_t)(*value), CMD_BIT_COUNT);
  return (bdm_response > BDM_NOTREADY) ? false : true;
}

bool bdm_ready(uint16_t next_cmd) {
  // wait until MCU responds
  uint8_t wait_cnt = ERR_COUNT;
  do {
    // read response
    bdm_clk(next_cmd, CMD_BIT_COUNT);
  } while (bdm_response == BDM_NOTREADY && --wait_cnt > 0);

  // check if command succeeded
  return (bdm_response == BDM_CMDCMPLTE);
}

//-----------------------------------------------------------------------------
/**
 Sets the speed at which BDM data is trransferred.

 @param            mode            SLOW, FAST, TURBO, NITROUS
 */
void bdm_clk_mode(int mode) {
  switch (mode) {
  case NITROUS:
    bdm_clk = &bdm_clk_nitrous;
    break;
  case TURBO:
    bdm_clk = &bdm_clk_turbo;
    break;
  case FAST:
    bdm_clk = &bdm_clk_fast;
    break;
  case SLOW:
  default:
    bdm_clk = &bdm_clk_slow;
    break;
  }
  return;
}

//-----------------------------------------------------------------------------
/**
 Writes a word to target MCU via BDM line and gets the response.

 @param            value            value to write
 @param            num_bits        value size, bits
 */

void bdm_clk_slow(uint16_t value, uint8_t num_bits) {
  //chSysLock();
  palSetLineMode(LINE_DSI, PAL_MODE_OUTPUT_PUSHPULL);
  // clock the value via BDM
  bdm_response = ((uint32_t)value) << (32 - num_bits);
  while (num_bits--) {

    // falling edge on BKPT/DSCLK
    palWriteLine(LINE_BKPT, 0);
    // set DSI bit
    palWriteLine(LINE_DSI, bdm_response & 0x80000000);
    bdm_response <<= 1;
    // read DSO bit
    bdm_response |= palReadLine(LINE_DSO);
    // short delay
    chThdSleepMicroseconds(1);
    // rising edge on BKPT/DSCLK
    palWriteLine(LINE_BKPT, 1);
    // short delay
    chThdSleepMicroseconds(1);
  }
  palSetLineMode(LINE_DSI, PAL_MODE_INPUT);
//  chSysUnlock();
}

/**
 Clears the BDM interface after errors.
 */
void bdm_clear(void) {
  bdm_clk(BDM_NOP, CMD_BIT_COUNT);
  bdm_clk(BDM_NOP, CMD_BIT_COUNT);
  bdm_clk(BDM_NOP, CMD_BIT_COUNT);
  bdm_clk(BDM_NOP, CMD_BIT_COUNT);

  while (bdm_response > 0) {
    bdm_clk(0, 1);
  }
  while (bdm_response < 1) {
    bdm_clk(0, 1);
  }
  bdm_clk(0, 15);
}

//-----------------------------------------------------------------------------
//    EOF
//-----------------------------------------------------------------------------
