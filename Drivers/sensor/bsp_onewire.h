/** \file
 *****************************************************************************************************************************************************
 *  \brief      bsp_onewire.h
 *  \copyright  Copyright &copy; 2019, Octagon IO Ltd t/a Converge. All Rights Reserved.
 *
 *  \author     rahul
 *  \date       23 May 2022
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


//this file describes a generic onewire interface for all devices to implement locally depending on the onewire driver used

/**
 *****************************************************************************************************************************************************
 *  \section HEADER_GUARD
 *****************************************************************************************************************************************************
 */
#ifndef BSP_ONEWIRE_INTERFACE_H_
#define BSP_ONEWIRE_INTERFACE_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "bsp.h"


/**
 *****************************************************************************************************************************************************
 *  \section GLOBAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */

/**
 * ::BSP_ONEWIRE_NONE is an invalid ROM address that will never occur in a device
 * (CRC mismatch), and so can be useful as an indicator for "no-such-device",
 * etc.
 */
#define BSP_ONEWIRE_NONE      ((bsp_onewire_addr_t)(0xffffffffffffffffLL))
#define ONEWIRE_SELECT_ROM    (0x55)
#define ONEWIRE_SKIP_ROM      (0xcc)
#define ONEWIRE_SEARCH        (0xf0)

/*
   In bsp_board.h, define the BSP_ONEWIRE_OBJECT using the macro defined below.
   Supported interfaces are:
   1. (_gpio_)
 */
#define BSP_ONEWIRE_DEFINE(interface)  \
    { \
        /* hardware specific portion of the onewire driver */ \
        .Init = bsp_onewire ## interface ## Init, \
        .Deinit = bsp_onewire ## interface ## Deinit, \
        .Reset = bsp_onewire ## interface ## Reset, \
        .Select = bsp_onewire ## interface ## Select, \
        .SkipRom = bsp_onewire ## interface ## SkipRom, \
        .WriteBit = bsp_onewire ## interface ## WriteBit, \
        .Write = bsp_onewire ## interface ## Write, \
        .WriteBytes = bsp_onewire ## interface ## WriteBytes, \
        .ReadBit = bsp_onewire ## interface ## ReadBit, \
        .Read = bsp_onewire ## interface ## Read, \
        .ReadBytes = bsp_onewire ## interface ## ReadBytes, \
        .Power = bsp_onewire ## interface ## Power, \
        .Depower = bsp_onewire ## interface ## Depower, \
    }

/**
 *****************************************************************************************************************************************************
 *  \section TYPEDEFS
 *****************************************************************************************************************************************************
 */

/**
 * Type used to hold all 1-Wire device ROM addresses (64-bit)
 */
typedef uint64_t bsp_onewire_addr_t;

/**
 * Structure to contain the current state for bsp_onewire_SearchNext(), etc
 */
typedef struct
{
    uint8_t rom_no[8];
    uint8_t last_discrepancy;
    uint8_t last_family_discrepancy;
    bool last_device_found;
} bsp_onewire_search_t;

/**
 *****************************************************************************************************************************************************
 *  \section FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */

typedef struct
{
    bsp_Result_t (* Init)(void);

    bsp_Result_t (* Deinit)(void);

    /**
     * @brief Perform a 1-Wire reset cycle.
     *
     * @return `true` if at least one device responds with a presence pulse,
     *         `false` if no devices were detected (or the bus is shorted, etc)
     */
    bool (* Reset)(void);

    /**
     * @brief Issue a 1-Wire rom select command to select a particular device.
     *
     * It is necessary to call Reset() before calling this function.
     *
     * @param[in] addr  The ROM address of the device to select
     *
     * @return `true` if the "ROM select" command could be successfully issued,
     *         `false` if there was an error.
     */
    bsp_Result_t (* Select)(const bsp_onewire_addr_t addr);

    /**
     * @brief Issue a 1-Wire "skip ROM" command to select *all* devices on the bus.
     *
     * It is necessary to call Reset() before calling this function.
     *
     * @return `true` if the "skip ROM" command could be successfully issued,
     *         `false` if there was an error.
     */
    bool (* SkipRom)(void);

    /*
     * @brief write a bit onto the onewire bus
     */
    bsp_Result_t (* WriteBit)(bool write_bit);

    /**
     * @brief Write a byte on the onewire bus.
     *
     * The writing code uses open-drain mode and expects the pullup resistor to
     * pull the line high when not driven low.  If you need strong power after the
     * write (e.g. DS18B20 in parasite power mode) then call Power() after
     * this is complete to actively drive the line high.
     *
     * @param[in] v     The byte value to write
     *
     * @return `BSP_RESULT_OK` if successful, `BSP_RESULT_ERROR` on error.
     */
    bsp_Result_t (* Write)(uint8_t value);

    /**
     * @brief Write multiple bytes on the 1-Wire bus.
     *
     * See Write() for more info.
     *
     * @param[in] buf    A pointer to the buffer of bytes to be written
     * @param[in] count  Number of bytes to write
     *
     * @return `BSP_RESULT_OK` if all bytes written successfully, `BSP_RESULT_ERROR` on error.
     */
    bsp_Result_t (* WriteBytes)(const uint8_t* buf, size_t count);

    /*
     * Read a bit from a 1-Wire device.
     */
    bsp_Result_t (* ReadBit)(bool* read_bit);


    /**
     * @brief Read a byte from a 1-Wire device.
     *
     * @return the read byte on success, negative value on error.
     */
    bsp_Result_t (* Read)(uint8_t* output_byte);

    /**
     * @brief Read multiple bytes from a 1-Wire device.
     *
     * @param[out] buf    A pointer to the buffer to contain the read bytes
     * @param[in] count  Number of bytes to read
     *
     * @return `true` on success, `false` on error.
     */
    bsp_Result_t (* ReadBytes)(uint8_t* buf, size_t count);

    /**
     * @brief Actively drive the bus high to provide extra power for certain operations
     *        of parasitically-powered devices.
     *
     * For parasitically-powered devices which need more power than can be
     * provided via the normal pull-up resistor, it may be necessary for some
     * operations to drive the bus actively high.  This function can be used to
     * perform that operation.
     *
     * The bus can be depowered once it is no longer needed by calling
     * Depower(), or it will be depowered automatically the next time
     * Reset() is called to start another command.
     *
     * Note: Make sure the device(s) you are powering will not pull more current
     * than the MCU is able to supply via its GPIO pins (this is especially
     * important when multiple devices are on the same bus and they are all
     * performing a power-intensive operation at the same time (i.e. multiple
     * DS18B20 sensors, which have all been given a "convert T" operation by using
     * bsp_onewire_gpio_SkipRom())).
     *
     * Note: This routine will check to make sure that the bus is already high
     * before driving it, to make sure it doesn't attempt to drive it high while
     * something else is pulling it low (which could cause a reset or damage the
     * MCU).

     *
     * @return `true` on success, `false` on error.
     */
    bool (* Power)(void);

    /**
     * @brief Stop forcing power onto the bus.
     *
     * You only need to do this if you previously called bsp_onewire_gpio_Power() to drive
     * the bus high and now want to allow it to float instead.  Note that
     * bsp_onewire_gpio_reset() will also automatically depower the bus first, so you do
     * not need to call this first if you just want to start a new operation.
     *
     */
    bool (* Depower)(void);
} bsp_onewire_t;


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL TYPEDEFS
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL VARIABLES
 *****************************************************************************************************************************************************
 */
extern const bsp_onewire_t onewire;


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */
extern uint8_t bsp_onewire_Crc8(const uint8_t* data, uint8_t len);
extern bool bsp_onewire_CheckCrc16(const uint8_t* input, size_t len, const uint8_t* inverted_crc, uint16_t crc_iv);
extern uint16_t bsp_onewire_Crc16(const uint8_t* input, size_t len, uint16_t crc_iv);
extern bool bsp_onewire_Search(bsp_onewire_search_t* search);
extern int bsp_onewire_ScanDevices(bsp_onewire_addr_t* list, int list_len);
#ifdef __cplusplus
}
#endif


#endif  /* BSP_ONEWIRE_INTERFACE_H_ */


/**
 *****************************************************************************************************************************************************
 *  END OF FILE
 *****************************************************************************************************************************************************
 */
