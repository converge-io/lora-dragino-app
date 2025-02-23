/*!
 * \file      utilities.h
 *
 * \brief     Helper functions implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __UTILITIES_H__
#define __UTILITIES_H__

#include <stdint.h>

#include "tremo_regs.h"
#define CRITICAL_SECTION_BEGIN( )      __disable_irq();

#define CRITICAL_SECTION_END( )        __enable_irq();

#define LORAMAC_CLASSB_ENABLED  0

/*!
 * Generic definition
 */
#ifndef SUCCESS
#define SUCCESS                                     1
#endif

#ifndef FAIL
#define FAIL                                        0
#endif

/*!
 * \brief Returns the minimum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

/*!
 * \brief Returns the maximum value between a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#define MAX(a, b) (((a) > (b)) ? (a) : (b))

/*!
 * \brief Returns 2 raised to the power of n
 *
 * \param [IN] n power value
 * \retval result of raising 2 to the power n
 */
#define POW2(n) (1 << n)

/*!
 * Version
 */
typedef union Version_u
{
    struct Version_s
    {
        uint8_t Revision;
        uint8_t Patch;
        uint8_t Minor;
        uint8_t Major;
    }Fields;
    uint32_t Value;
}Version_t;

/*!
 * \brief Initializes the pseudo random generator initial value
 *
 * \param [IN] seed Pseudo random generator initial value
 */
void srand1(uint32_t seed);

int32_t rand1(void);

/*!
 * \brief Computes a random number between min and max
 *
 * \param [IN] min range minimum value
 * \param [IN] max range maximum value
 * \retval random random value in range min..max
 */
int32_t randr(int32_t min, int32_t max);

/*!
 * \brief Copies size elements of src array to dst array
 *
 * \remark STM32 Standard memcpy function only works on pointers that are aligned
 *
 * \param [OUT] dst  Destination array
 * \param [IN]  src  Source array
 * \param [IN]  size Number of bytes to be copied
 */
void memcpy1(uint8_t* dst, const uint8_t* src, uint16_t size);

/*!
 * \brief Copies size elements of src array to dst array reversing the byte order
 *
 * \param [OUT] dst  Destination array
 * \param [IN]  src  Source array
 * \param [IN]  size Number of bytes to be copied
 */
void memcpyr(uint8_t* dst, const uint8_t* src, uint16_t size);

/*!
 * \brief Set size elements of dst array with value
 *
 * \remark STM32 Standard memset function only works on pointers that are aligned
 *
 * \param [OUT] dst   Destination array
 * \param [IN]  value Default value
 * \param [IN]  size  Number of bytes to be copied
 */
void memset1(uint8_t* dst, uint8_t value, uint16_t size);

/*!
 * \brief Converts a nibble to an hexadecimal character
 *
 * \param [IN] a   Nibble to be converted
 * \retval hexChar Converted hexadecimal character
 */
int8_t Nibble2HexChar(uint8_t a);

/*!
 * \brief Computes a CCITT 32 bits CRC
 *
 * \param [IN] buffer   Data buffer used to compute the CRC
 * \param [IN] length   Data buffer length
 *
 * \retval crc          The computed buffer of length CRC
 */
uint32_t Crc32(uint8_t* buffer, uint16_t length);

/*!
 * \brief Computes the initial value of the CCITT 32 bits CRC. This function
 *        can be used with functions \ref Crc32Update and \ref Crc32Finalize.
 *
 * \retval crc          Initial crc value.
 */
uint32_t Crc32Init(void);

/*!
 * \brief Updates the value of the crc value.
 *
 * \param [IN] crcInit  Previous or initial crc value.
 * \param [IN] buffer   Data pointer.
 * \param [IN] length   Length of the data.
 *
 * \retval crc          Updated crc value.
 */
uint32_t Crc32Update(uint32_t crcInit, uint8_t* buffer, uint16_t length);

/*!
 * \brief Finalizes the crc value after the calls to \ref Crc32Update.
 *
 * \param [IN] crc      Recent crc value.
 *
 * \retval crc          Updated crc value.
 */
uint32_t Crc32Finalize(uint32_t crc);

#endif // __UTILITIES_H__
