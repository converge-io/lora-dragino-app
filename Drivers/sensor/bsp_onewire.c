/** \file
 *****************************************************************************************************************************************************
 *  \brief      bsp_onewire source file
 *  \copyright  Copyright &copy; 2019, Octagon IO Ltd t/a Converge. All Rights Reserved.
 *
 *  \author     rahul
 *  \date       4 May 2022
 *  \version    1.0
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  Use of the software contained in this file is only permitted by written agreement with Octagon IO Ltd t/a Converge.
 *  Any such agreement must identify this module by name.
 *  Similarly, use of executable code derived from this software is only allowed by licence agreement with Octagon IO Ltd t/a Converge:
 *      Converge,
 *      5.02A Mermaid House,
 *      Puddle Dock,
 *      London,
 *      EC4V 3DS
 *      https://converge.io
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include "bsp_board.h"
#include "bsp_onewire.h"

#include "log.h"

/**
 *****************************************************************************************************************************************************
 *  \section LOCAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */
#define MODULE_NAME "w1_api"

const bsp_onewire_t onewire = BSP_ONEWIRE_OBJECT;        //Defined in each bsp_board.h of the target hardware


/**
 *****************************************************************************************************************************************************
 *  \section FUNCTION DEFINITIONS
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  @brief Find the next onewire device on the line
 *  @params [in]: takes an object that has the last discrepancy and device found for the search
 *                algorithm as described in Maxim Application Note 187
 *  @param [out]: returns the ROM number of the device that was found on the bus
 *  @return true if a device was found
 *          false if no device found/no additional devices found
 *****************************************************************************************************************************************************
 */
bool bsp_onewire_Search(bsp_onewire_search_t* search)
{
    bsp_Result_t search_result = false;
    int id_bit_number;
    int last_zero, rom_byte_number; //, search_result;
    bool id_bit, cmp_id_bit;
    unsigned char rom_byte_mask;
    uint8_t crc8;

    /* initialize for search */
    id_bit_number = 1;
    last_zero = 0;
    rom_byte_number = 0;
    rom_byte_mask = 1;
    search_result = 0;
    crc8 = 0;

    /* if the last call was not the last one */
    if (!search->last_device_found)
    {
        unsigned char search_direction;

        /* 1-Wire reset */
        if (!onewire.Reset())
        {
            /* reset the search */
            search->last_discrepancy = 0;
            search->last_device_found = false;
            search->last_family_discrepancy = 0;
            logging_Info("couldn't reset the line");

            return false;
        }

        /* issue the search command */
        onewire.Write(ONEWIRE_SEARCH);

        /* loop to do the search */
        do
        {
            /* read a bit and its complement */
            onewire.ReadBit(&id_bit);
            onewire.ReadBit(&cmp_id_bit);
            /* check for no devices on 1-wire */
            if ((id_bit == 1) && (cmp_id_bit == 1))
            {
                break;
            }
            else
            {
                /* all devices coupled have 0 or 1 */
                if (id_bit != cmp_id_bit)
                {
                    search_direction = id_bit; /* bit write value for search */
                }
                else
                {
                    /* if this discrepancy if before the Last Discrepancy
                     * on a previous next then pick the same as last time */
                    if (id_bit_number < search->last_discrepancy)
                    {
                        search_direction = ((search->rom_no[rom_byte_number] & rom_byte_mask) > 0);
                    }
                    else
                    {
                        /* if equal to last pick 1, if not then pick 0 */
                        search_direction = (id_bit_number == search->last_discrepancy);
                    }
                    /* if 0 was picked then record its position in LastZero */
                    if (search_direction == 0)
                    {
                        last_zero = id_bit_number;
                        /* check for Last discrepancy in family */
                        if (last_zero < 9)
                        {
                            search->last_family_discrepancy = last_zero;
                        }
                    }
                }
                /* set or clear the bit in the ROM byte rom_byte_number with mask rom_byte_mask */
                if (search_direction == 1)
                {
                    search->rom_no[rom_byte_number] |= rom_byte_mask;
                }
                else
                {
                    search->rom_no[rom_byte_number] &= ~rom_byte_mask;
                }
                /* serial number search direction write bit */
                //OWWriteBit(search_direction);
                onewire.WriteBit(search_direction);
                /* increment the byte counter id_bit_number and shift the mask rom_byte_mask */
                id_bit_number++;
                rom_byte_mask <<= 1;
                /* if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask */
                if (rom_byte_mask == 0)
                {
                    bsp_onewire_Crc8(search->rom_no, 8); /* accumulate the CRC */
                    rom_byte_number++;
                    rom_byte_mask = 1;
                }
            }
        }
        while (rom_byte_number < 8); // loop until through all ROM bytes 0-7

        /* if the search was successful then */
        if (!((id_bit_number < 65) || (crc8 != 0)))
        {
            /* search successful so set
             * search->last_discrepancy, search->last_device_found, search_result */
            search->last_discrepancy = last_zero;
            /* check for last device */
            if (search->last_discrepancy == 0)
            {
                search->last_device_found = true;
            }
            search_result = true;
        }
    }

    /* if no device found then reset counters so next 'search' will be like a first */
    if (!search_result || !search->rom_no[0])
    {
        search->last_discrepancy = 0;
        search->last_device_found = false;
        search->last_family_discrepancy = 0;
        search_result = false;
    }

    return search_result;
}


/**
 *****************************************************************************************************************************************************
 *  @brief Uses the onewire Search function to find all the devices on the bus
 *  @params[in]: max number of devices expected
 *  @param [out]: returns a list of the addresses found, in order
 *  @return number of devices found
 *****************************************************************************************************************************************************
 */
int bsp_onewire_ScanDevices(bsp_onewire_addr_t* list, int list_len)
{
    bsp_onewire_search_t search;
    uint8_t found = 0;

    memset(&search, 0, sizeof(search));

    for (int i = 0; i < list_len; i++)
    {
        if (!bsp_onewire_Search(&search))
        {
            break;
        }
        memcpy(&list[found], search.rom_no, 8);
        found += 1;
    }

    return found;
}


/**
 *****************************************************************************************************************************************************
 *  @brief  The 1-Wire CRC scheme is described in Maxim Application Note 27:
 *          "Understanding and Using Cyclic Redundancy Checks with Maxim iButton Products"
 *          Lookup table to compute a Dallas Semiconductor 8 bit CRC
 *  @params input - pointer to array and length of the array
 *  @return 8bit CRC value
 *****************************************************************************************************************************************************
 */
static unsigned char dscrc_table[] = {
    0, 94,188,226, 97, 63,221,131,194,156,126, 32,163,253, 31, 65,
    157,195, 33,127,252,162, 64, 30, 95, 1,227,189, 62, 96,130,220,
    35,125,159,193, 66, 28,254,160,225,191, 93, 3,128,222, 60, 98,
    190,224, 2, 92,223,129, 99, 61,124, 34,192,158, 29, 67,161,255,
    70, 24,250,164, 39,121,155,197,132,218, 56,102,229,187, 89, 7,
    219,133,103, 57,186,228, 6, 88, 25, 71,165,251,120, 38,196,154,
    101, 59,217,135, 4, 90,184,230,167,249, 27, 69,198,152,122, 36,
    248,166, 68, 26,153,199, 37,123, 58,100,134,216, 91, 5,231,185,
    140,210, 48,110,237,179, 81, 15, 78, 16,242,172, 47,113,147,205,
    17, 79,173,243,112, 46,204,146,211,141,111, 49,178,236, 14, 80,
    175,241, 19, 77,206,144,114, 44,109, 51,209,143, 12, 82,176,238,
    50,108,142,208, 83, 13,239,177,240,174, 76, 18,145,207, 45,115,
    202,148,118, 40,171,245, 23, 73, 8, 86,180,234,105, 55,213,139,
    87, 9,235,181, 54,104,138,212,149,203, 41,119,244,170, 72, 22,
    233,183, 85, 11,136,214, 52,106, 43,117,151,201, 74, 20,246,168,
    116, 42,200,150, 21, 75,169,247,182,232, 10, 84,215,137,107, 53
};

uint8_t bsp_onewire_Crc8(const uint8_t* data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        crc = dscrc_table[crc ^ *(data + i)];
    }
    return crc;
}


/**
 *****************************************************************************************************************************************************
 *  @brief  Compute the 1-Wire CRC16 and compare it against the received CRC.
 *          Example usage (reading a DS2408):
 *          Put everything in a buffer so we can compute the CRC easily.
 *          uint8_t buf[13];
 *          buf[0] = 0xF0;    // Read PIO Registers
 *          buf[1] = 0x88;    // LSB address
 *          buf[2] = 0x00;    // MSB address
 *          WriteBytes(net, buf, 3);    // Write 3 cmd bytes
 *          ReadBytes(net, buf+3, 10);  // Read 6 data bytes, 2 0xFF, 2 CRC16
 *          if (!CheckCRC16(buf, 11, &buf[11])) {
 *          // Handle error.
 *          }
 *  @params input - Array of bytes to checksum.
 *  @params len - How many bytes to use.
 *  @params inverted_crc - The two CRC16 bytes in the received data.
 *                         This should just point into the received data,
 *                         not at a 16-bit integer.
 *  @params crc - The crc starting value (optional)
 *  @return 1, iff the CRC matches.
 *****************************************************************************************************************************************************
 */
bool bsp_onewire_CheckCrc16(const uint8_t* input, size_t len, const uint8_t* inverted_crc, uint16_t crc_iv)
{
    uint16_t crc = ~bsp_onewire_Crc16(input, len, crc_iv);
    return (crc & 0xFF) == inverted_crc[0] && (crc >> 8) == inverted_crc[1];
}


/**
 *****************************************************************************************************************************************************
 *  @brief  Compute a Dallas Semiconductor 16 bit CRC.  This is required to check
 *          the integrity of data received from many 1-Wire devices.  Note that the
 *          CRC computed here is *not* what you'll get from the 1-Wire network,
 *          for two reasons:
 *          1) The CRC is transmitted bitwise inverted.
 *          2) Depending on the endian-ness of your processor, the binary
 *          representation of the two-byte return value may have a different
 *          byte order than the two bytes you get from 1-Wire.
 *  @params input - Array of bytes to checksum.
 *  @params len - How many bytes to use.
 *  @params crc - The crc starting value (optional)
 *  @return The CRC16, as defined by Dallas Semiconductor.
 *****************************************************************************************************************************************************
 */
uint16_t bsp_onewire_Crc16(const uint8_t* input, size_t len, uint16_t crc_iv)
{
    uint16_t crc = crc_iv;
    static const uint8_t oddparity[16] = { 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0 };

    uint16_t i;
    for (i = 0; i < len; i++)
    {
        // Even though we're just copying a byte from the input,
        // we'll be doing 16-bit computation with it.
        uint16_t cdata = input[i];
        cdata = (cdata ^ crc) & 0xff;
        crc >>= 8;

        if (oddparity[cdata & 0x0F] ^ oddparity[cdata >> 4])
        {
            crc ^= 0xC001;
        }

        cdata <<= 6;
        crc ^= cdata;
        cdata <<= 1;
        crc ^= cdata;
    }
    return crc;
}


/**
 *****************************************************************************************************************************************************
 *  END OF FILE
 *****************************************************************************************************************************************************
 */
