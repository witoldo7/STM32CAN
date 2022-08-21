#include "bdm.h"
#include "bdmcpu32.h"
#include <string.h>

bool erase_am29(void);
bool erase_am28(const uint32_t* start_addr, const uint32_t* end_addr);
bool flash_am28(const uint32_t* addr, uint16_t value);
//-----------------------------------------------------------------------------
/**
    Erases the flash memory chip starting from [start_addr] up to, but not
    including [end_addr] and optionally verifies the result; MCU must be in
    background mode.

    @param        flash_type        type of flash chip
    @param        start_addr        flash start address
    @param        end_addr        flash end address

    @return                        status flag
*/
uint8_t erase_flash(const char* flash_type, const uint32_t* start_addr,
                    const uint32_t* end_addr)
{
    // AM29Fxxx chips (retrofitted to Trionic 5.x; original to T7)
    if (strncmp(flash_type, "29f010", (size_t)6) == 0 ||
            strncmp(flash_type, "29f400", (size_t)6) == 0) {
        return erase_am29() ? TERM_OK : TERM_ERR;
    }

    // AM28F010 chip (Trionic 5.x original)
    if (strncmp(flash_type, "28f010", (size_t)6) == 0) {
        return erase_am28(start_addr, end_addr) ? TERM_OK : TERM_ERR;
    }

    return TERM_ERR;
}

/**
Resets an AM29Fxxx flash memory chip. MCU must be in background mode.

@param                          none

@return                         succ / fail
*/
bool reset_am29(void)
{
    // execute the algorithm
    for (uint8_t i = 0; i < 3; ++i) {
        if (memwrite_word(&am29_reset[i].addr, am29_reset[i].val) != TERM_OK) return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
/**
Erases an AM29Fxxx flash memory chip and verifies the result; MCU must be
in background mode.

@return                        succ / fail
*/
bool erase_am29(void)
{
    // reset flash
    if (!reset_am29()) {
        return false;
    }

    // execute the algorithm
    for (uint8_t i = 0; i < 6; ++i) {
        if (memwrite_word(&am29_erase[i].addr, am29_erase[i].val) != TERM_OK) {
            reset_am29();
            return false;
        }
    }

    // verify the result
    uint32_t addr = 0x0;
    uint16_t verify_value;

    systime_t start = chVTGetSystemTimeX();
    systime_t end = chTimeAddX(start, TIME_MS2I(200));
    while (chTimeIsInRangeX(chVTGetSystemTimeX(), start, end)) {
        // Typical and Maximum Chip Programming times are 9 and 27 seconds for Am29BL802C
        // Typical Chip erase time for Am29BL802C is 45 secinds, not including 0x00 programming prior to erasure.
        // Allow for at least worst case 27 seconds programming to 0x00 + 3(?) * 45 typical erase time (162 seconds)
        // Allow at least 200 seconds erase time 2,000 * (100ms + BDM memread time)
        // NOTE: 29/39F010 and 29F400 erase times are considerably lower
        if (memread_word(&verify_value, &addr) == TERM_OK && verify_value == 0xffff) {
            // erase completed normally
            reset_am29();
            return true;
        }
    }
    // erase failed
    reset_am29();
    return false;
}

//-----------------------------------------------------------------------------
/**
Writes a word to AM29Fxxx flash memory chip and optionally verifies the
result; MCU must be in background mode.

@param        addr        destination address
@param        val            value

@return                    succ / fail
*/
bool flash_am29(const uint32_t* addr, uint16_t value)
{
    // execute the algorithm
    for (uint8_t i = 0; i < 3; ++i) {
        if (memwrite_word(&am29_write[i].addr, am29_write[i].val) != TERM_OK) {
            reset_am29();
            return false;
        }
    }
    // write the value
    if (memwrite_word(addr, value) != TERM_OK) {
        reset_am29();
        return false;
    }
    // verify the result
    systime_t start = chVTGetSystemTimeX();
    systime_t end = chTimeAddX(start, TIME_US2I(500));
    while (chTimeIsInRangeX(chVTGetSystemTimeX(), start, end)) {
        // Typical and Maximum Word Programming times are 9us and 360us for Am29BL802C
        // Allow at least 500 microseconds program time 500 * (1us + BDM memread time)
        // NOTE: 29/39F010 and 29F400 programming times are considerably lower
        uint16_t verify_value;
        if ((memread_word(&verify_value, addr) == TERM_OK) &&
                (verify_value == value)) {
            // flashing successful
            return true;
        }
    }
    // writing failed
    reset_am29();
    return false;
}

//-----------------------------------------------------------------------------
/**
Resets a AM28Fxxx flash memory chip. MCU must be in background mode.

@param      start_addr      flash start address

@return                     succ / fail
*/
bool reset_am28(void)
{
    uint32_t start_addr = 0x0;
    return (memwrite_word_write_word(&start_addr, 0xffff, 0xffff) == TERM_OK);
}

//-----------------------------------------------------------------------------
/**
Erases an AM28Fxxx flash memory chip and verifies the result; MCU must be
in background mode.

@param      start_addr      flash start address
@param      end_addr        flash end address

@return                     succ / fail
*/
bool erase_am28(const uint32_t* start_addr, const uint32_t* end_addr)
{

    // check the addresses
    if (!start_addr || !end_addr) return false;

    // reset flash
    if (!reset_am28()) return false;

    // write zeroes over entire flash space
    uint32_t addr = *start_addr;

    while (addr < *end_addr) {
        if (!flash_am28(&addr, 0x0000)) return false;
        addr += 2;
        //        // feedback to host computer
        //        pc.putc(TERM_OK);
        if (!(addr % 0x80)) {
            // make the activity LED twinkle
        }
    }

    // erase flash
    addr = *start_addr;
    uint8_t verify_value;

    uint16_t pulse_cnt = 0;
    if (memwrite_byte_cmd(NULL) != TERM_OK) {
        reset_am28();
        return false;
    }
    while ((++pulse_cnt < 1000) && (addr < *end_addr)) {
        // issue the erase command
        if (memwrite_write_byte(&addr, 0x20) != TERM_OK ||
                memwrite_write_byte(&addr, 0x20) != TERM_OK) break;
        chThdSleepMilliseconds(10);

        while (addr < *end_addr) {
            // issue the verify command
            if (memwrite_read_byte(&addr, 0xa0) != TERM_OK) break;
            //            wait_us(6);
            // check the written value
            if (memread_write_byte(&verify_value, &addr) != TERM_OK) break;
            if (verify_value != 0xff) break;
            // succeeded need to check next address
            addr++;

            if (!(addr % 0x80)) {
                // make the activity LED twinkle
            }
        }
    }
    // the erase process ends with a BDM_WRITE + BDM_BYTESIZE command left in the BDM
    // it is safe to use it to put one of the FLASH chips into read mode and thereby
    // leave the BDM ready for the next command
    memwrite_nop_byte(start_addr, 0x00);

    reset_am28();
    // check for success
    return (addr == *end_addr) ? true : false;
}

//-----------------------------------------------------------------------------
/**
Writes a byte to AM28Fxxx flash memory chip and verifies the result
A so called 'mask' method checks the FLASH contents and only tries
to program bytes that need to be programmed.
MCU must be in background mode.

@param      addr        destination address
@param      val         value

@return                 succ / fail
*/
bool flash_am28(const uint32_t* addr, uint16_t value)
{

    if (!addr) return false;

    uint8_t pulse_cnt = 0;
    uint16_t verify_value = 0;
    uint16_t mask_value = 0xffff;

    // put flash into read mode and read address
    if (memwrite_word_read_word(&verify_value, addr, 0x0000) != TERM_OK)  return false;
    // return if FLASH already has the correct value - e.g. not all of the FLASH is used and is 0xff
    if (verify_value == value) return true;

    while (++pulse_cnt < 25) {

        // set a mask
        if ((uint8_t)verify_value == (uint8_t)value)
            mask_value &= 0xff00;
        if ((uint8_t)(verify_value >> 8) == (uint8_t)(value >> 8))
            mask_value &= 0x00ff;

        // write the new value
        if (memwrite_word_write_word(addr, (0x4040 & mask_value), value) != TERM_OK) break;
        // NOTE the BDM interface is slow enough that there is no need for a 10us delay before verifying
        // issue the verification command
        // NOTE the BDM interface is slow enough that there is no need for a 6us delay before reading back
        if (memwrite_word_read_word(&verify_value, addr, (0xc0c0 & mask_value)) != TERM_OK) break;
        // check if flashing was successful;
        if (verify_value == value) return true;
    }

    // something went wrong; reset the flash chip and return failed
    reset_am28();
    return false;
}

