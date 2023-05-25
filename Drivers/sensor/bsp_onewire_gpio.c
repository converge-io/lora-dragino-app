/**
 * @file bsp_onewire.c
 *
 * Routines to access devices using the Dallas Semiconductor 1-Wire(tm)
 * protocol.
 *
 * This is a port taken from esp-idf-lib of a bit-banging one wire
 * driver based on the implementation from NodeMCU.
 *
 * This, in turn, appears to have been based on the PJRC Teensy driver
 * (https://www.pjrc.com/teensy/td_libs_OneWire.html), by Jim Studt, Paul
 * Stoffregen, and a host of others.
 *
 * The original code is licensed under the MIT license.  The CRC code was taken
 * (at least partially) from Dallas Semiconductor sample code, which was licensed
 * under an MIT license with an additional clause (prohibiting inappropriate use
 * of the Dallas Semiconductor name).  See the accompanying LICENSE file for
 * details.
 */


/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include <string.h>
#include "bsp_onewire_gpio.h"
#include "bsp_board.h"
#include "bsp_onewire.h"
#include "tremo_gpio.h"


/**
 *****************************************************************************************************************************************************
 *  \section LOCAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */
#define PORT_ENTER_CRITICAL
#define PORT_EXIT_CRITICAL
#define BSP_GPIO_PULL_STATE  BSP_GPIO_PULL_DISABLED


/**
 *****************************************************************************************************************************************************
 *  \section LOCAL VARIABLES
 *****************************************************************************************************************************************************
 */
uint8_t port = BSP_BOARD_TAIL_ONE_WIRE_PORT;
uint8_t pin = BSP_BOARD_TAIL_ONE_WIRE_PIN;


/**
 *****************************************************************************************************************************************************
 *  \section FUNCTION DEFINITIONS
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  @brief Sets up the Onewire device
 *
 *  @params
 *  @return
 *****************************************************************************************************************************************************
 */
bsp_Result_t bsp_onewire_gpio_Init(void)
{
    BSP_BOARD_TAIL_CLK_ENABLE();
    gpio_init(BSP_BOARD_TAIL_ONE_WIRE_PORT, BSP_BOARD_TAIL_ONE_WIRE_PIN, GPIO_MODE_OUTPUT_INPUT_PULL_UP);
    return BSP_RESULT_OK;
}


/**
 *****************************************************************************************************************************************************
 *  @brief Sets up the Onewire device
 *
 *  @params
 *  @return
 *****************************************************************************************************************************************************
 */
bsp_Result_t bsp_onewire_gpio_Deinit(void)
{
    gpio_init(BSP_BOARD_TAIL_ONE_WIRE_PORT, BSP_BOARD_TAIL_ONE_WIRE_PIN, GPIO_MODE_OUTPUT_PP_HIGH);
    return BSP_RESULT_OK;
}


// Waits up to `max_wait` microseconds for the specified pin to go high.
// Returns true if successful, false if the bus never comes high (likely
// shorted).
static inline bool bsp_onewire_gpio_WaitForBus(int max_wait)
{
    bool state;
    //bsp_gpio_ConfigurePinSimpleInput(port, pin, BSP_GPIO_PULL_STATE);
    for (int i = 0; i < ((max_wait + 4) / 5); i++)
    {
        if (gpio_read(port, pin) == GPIO_LEVEL_HIGH)
        {
            break;
        }
        delay_us(5);
    }
    state = gpio_read(port, pin);
    // Wait an extra 1us to make sure the devices have an adequate recovery
    // time before we drive things low again.
    delay_us(1);
    return state == GPIO_LEVEL_HIGH;
}


// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return false;
//
// Returns true if a device asserted a presence pulse, false otherwise.
//
bool bsp_onewire_gpio_Reset(void)
{
    bsp_gpio_ConfigurePinSimpleInput(port, pin, BSP_GPIO_PULL_STATE);
    // wait until the wire is high... just in case
    if (!bsp_onewire_gpio_WaitForBus(250))
    {
        return false;
    }

    gpio_write(port, pin, GPIO_LEVEL_LOW);
    delay_us(480);

    PORT_ENTER_CRITICAL;
    bsp_gpio_ConfigurePinSimpleInput(port, pin, BSP_GPIO_PULL_STATE);
    delay_us(70);
    bool r = (gpio_read(port, pin) == GPIO_LEVEL_LOW);
    PORT_EXIT_CRITICAL;

    // Wait for all devices to finish pulling the bus low before returning
    if (!bsp_onewire_gpio_WaitForBus(410))
    {
        return false;
    }

    return r;
}


bsp_Result_t bsp_onewire_gpio_WriteBit(bool v)
{
    if (!bsp_onewire_gpio_WaitForBus(10))
    {
        return BSP_RESULT_ERROR;
    }
    PORT_ENTER_CRITICAL;
    if (v)
    {
        gpio_write(port, pin, GPIO_LEVEL_LOW);  // drive output low
        delay_us(10);
        //bsp_gpio_ConfigurePinSimpleInput(port, pin, BSP_GPIO_PULL_STATE);  // allow output high
        delay_us(55);
    }
    else
    {
        gpio_write(port, pin, GPIO_LEVEL_LOW);  // drive output low
        delay_us(65);
        //bsp_gpio_ConfigurePinSimpleInput(port, pin, BSP_GPIO_PULL_STATE);  // allow output high
    }
    delay_us(1);
    PORT_EXIT_CRITICAL;

    return BSP_RESULT_OK;
}


bsp_Result_t bsp_onewire_gpio_ReadBit(bool* read_bit)
{
    if (!bsp_onewire_gpio_WaitForBus(10))
    {
        return BSP_RESULT_ERROR;
    }

    PORT_ENTER_CRITICAL;
    gpio_write(port, pin, GPIO_LEVEL_LOW);
    delay_us(2);
    //bsp_gpio_ConfigurePinSimpleInput(port, pin, BSP_GPIO_PULL_STATE);  // let pin float, pull up will raise
    delay_us(11);
    *read_bit = (gpio_read(port, pin) == GPIO_LEVEL_HIGH);  // Must sample within 15us of start
    delay_us(48);
    PORT_EXIT_CRITICAL;

    return BSP_RESULT_OK;
}


// Write a byte. The writing code uses open-drain mode and expects the pullup
// resistor to pull the line high when not driven low.  If you need strong
// power after the write (e.g. DS18B20 in parasite power mode) then call
// bsp_onewire_gpio_Power() after this is complete to actively drive the line high.
//
bsp_Result_t bsp_onewire_gpio_Write(uint8_t v)
{
    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        if (bsp_onewire_gpio_WriteBit((bitMask & v)) != BSP_RESULT_OK)
        {
            return BSP_RESULT_ERROR;
        }
    }

    return BSP_RESULT_OK;
}


bsp_Result_t bsp_onewire_gpio_WriteBytes(const uint8_t* buf, size_t count)
{
    for (size_t i = 0; i < count; i++)
    {
        if (bsp_onewire_gpio_Write(buf[i]) != BSP_RESULT_OK)
        {
            return BSP_RESULT_ERROR;
        }
    }

    return BSP_RESULT_OK;
}


// Read a byte
//
bsp_Result_t bsp_onewire_gpio_Read(uint8_t* output_byte)
{
    *output_byte = 0;

    for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1)
    {
        bool bit;
        if (bsp_onewire_gpio_ReadBit(&bit) != BSP_RESULT_OK)
        {
            return BSP_RESULT_ERROR;
        }
        else if (bit)
        {
            *output_byte |= bitMask;
        }
    }
    return BSP_RESULT_OK;
}


bsp_Result_t bsp_onewire_gpio_ReadBytes(uint8_t* buf, size_t count)
{
    for (size_t i = 0; i < count; i++)
    {
        uint8_t b;
        if (bsp_onewire_gpio_Read(&b) != BSP_RESULT_OK)
        {
            return BSP_RESULT_ERROR;
        }
        *(buf + i) = b;
    }
    return BSP_RESULT_OK;
}


bsp_Result_t bsp_onewire_gpio_Select(bsp_onewire_addr_t addr)
{
    uint8_t i;

    if (bsp_onewire_gpio_Write(ONEWIRE_SELECT_ROM) != BSP_RESULT_OK)
    {
        return BSP_RESULT_ERROR;
    }

    for (i = 0; i < 8; i++)
    {
        if (bsp_onewire_gpio_Write(addr & 0xff) != BSP_RESULT_OK)
        {
            return BSP_RESULT_ERROR;
        }
        addr >>= 8;
    }

    return BSP_RESULT_OK;
}


bool bsp_onewire_gpio_SkipRom(void)
{
    return bsp_onewire_gpio_Write(ONEWIRE_SKIP_ROM) == BSP_RESULT_OK ? true : false;
}


bool bsp_onewire_gpio_Power(void)
{
    // Make sure the bus is not being held low before driving it high, or we
    // may end up shorting ourselves out.
    if (!bsp_onewire_gpio_WaitForBus(10))
    {
        return false;
    }

    gpio_write(port, pin, GPIO_LEVEL_HIGH);

    return true;
}


bool bsp_onewire_gpio_Depower(void)
{
    bsp_gpio_ConfigurePinSimpleInput(port, pin, BSP_GPIO_PULL_STATE);
    return true;
}


void bsp_onewire_gpio_SearchStart(bsp_onewire_search_t* search)
{
    // reset the search state
    memset(search, 0, sizeof(*search));
}


void bsp_onewire_gpio_SearchPrefix(bsp_onewire_search_t* search, uint8_t family_code)
{
    uint8_t i;

    search->rom_no[0] = family_code;
    for (i = 1; i < 8; i++)
    {
        search->rom_no[i] = 0;
    }
    search->last_discrepancy = 64;
    search->last_device_found = false;
}


// Perform a search. If the next device has been successfully enumerated, its
// ROM address will be returned.  If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then ONEWIRE_NONE is returned.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return 1 : device found, ROM number in ROM_NO buffer
//        0 : device not found, end of search
//
bsp_onewire_addr_t bsp_onewire_gpio_SearchNext(bsp_onewire_search_t* search)
{
    uint8_t id_bit_number;
    uint8_t last_zero, search_result;
    int rom_byte_number;
    bsp_onewire_addr_t addr;
    unsigned char rom_byte_mask;

    // initialize for search
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = 0;

    // if the last call was not the last one
    if (!search->last_device_found)
    {
        // 1-Wire reset
        if (!bsp_onewire_gpio_Reset())
        {
            // reset the search
            search->last_discrepancy = 0;
            search->last_device_found = false;
            return BSP_ONEWIRE_NONE;
        }

        // issue the search command
        bsp_onewire_gpio_Write(ONEWIRE_SEARCH);

        // loop to do the search
        do
        {
            // read a bit and its complement
            bool id_bit, cmp_id_bit;
            bsp_onewire_gpio_ReadBit(&id_bit);
            bsp_onewire_gpio_ReadBit(&cmp_id_bit);

            if ((id_bit == true) && (cmp_id_bit == true))
            {
                break;
            }
            else
            {
                bool search_direction;

                // all devices coupled have 0 or 1
                if (id_bit != cmp_id_bit)
                {
                    search_direction = id_bit;  // bit write value for search
                }
                else
                {
                    // if this discrepancy if before the Last Discrepancy
                    // on a previous next then pick the same as last time
                    if (id_bit_number < search->last_discrepancy)
                    {
                        search_direction = ((search->rom_no[rom_byte_number] & rom_byte_mask) > 0);
                    }
                    else
                    {
                        // if equal to last pick 1, if not then pick 0
                        search_direction = (id_bit_number == search->last_discrepancy);
                    }

                    // if 0 was picked then record its position in LastZero
                    if (!search_direction)
                    {
                        last_zero = id_bit_number;
                    }
                }

                // set or clear the bit in the ROM byte rom_byte_number
                // with mask rom_byte_mask
                if (search_direction)
                {
                    search->rom_no[rom_byte_number] |= rom_byte_mask;
                }
                else
                {
                    search->rom_no[rom_byte_number] &= ~rom_byte_mask;
                }

                // serial number search direction write bit
                bsp_onewire_gpio_WriteBit(search_direction);

                // increment the byte counter id_bit_number
                // and shift the mask rom_byte_mask
                id_bit_number++;
                rom_byte_mask <<= 1;

                // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
                if (rom_byte_mask == 0)
                {
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        }
        while (rom_byte_number < 8);    // loop until through all ROM bytes 0-7

        // if the search was successful then
        if (!(id_bit_number < 65))
        {
            // search successful so set last_discrepancy,last_device_found,search_result
            search->last_discrepancy = last_zero;

            // check for last device
            if (search->last_discrepancy == 0)
            {
                search->last_device_found = true;
            }

            search_result = 1;
        }
    }

    // if no device found then reset counters so next 'search' will be like a first
    if (!search_result || !search->rom_no[0])
    {
        search->last_discrepancy = 0;
        search->last_device_found = false;
        return BSP_ONEWIRE_NONE;
    }
    else
    {
        addr = 0;
        for (rom_byte_number = 7; rom_byte_number >= 0; rom_byte_number--)
        {
            addr = (addr << 8) | search->rom_no[rom_byte_number];
        }
        //printf("Ok I found something at %08x%08x...\n", (uint32_t)(addr >> 32), (uint32_t)addr);
    }
    return addr;
}


/**
 *****************************************************************************************************************************************************
 *  END OF FILE
 *****************************************************************************************************************************************************
 */
