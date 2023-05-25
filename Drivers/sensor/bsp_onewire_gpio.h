/**
 * @file bsp_onewire.h
 * @defgroup onewire onewire
 * @{
 *
 * @brief Routines to access devices using the Dallas Semiconductor 1-Wire(tm)
 *        protocol.
 *
 * This is a port a bit-banging one wire driver modified from esp-idf-lib
 * and based on the implementation from NodeMCU.
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
#ifndef BSP_ONEWIRE_GPIO_H_
#define BSP_ONEWIRE_GPIO_H_


#ifdef __cplusplus
extern "C" {
#endif


/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include <stdbool.h>
///#include "bsp_gpio.h"
#include "bsp_onewire.h"


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL TYPEDEFS
 *****************************************************************************************************************************************************
 */
typedef uint64_t bsp_onewire_addr_t;


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */
extern result_t bsp_onewire_gpio_Initialise(hal_sensor_Handle_t* handle);
extern bsp_Result_t bsp_onewire_gpio_Init(void);
extern bsp_Result_t bsp_onewire_gpio_Deinit(void);
extern bool bsp_onewire_gpio_Reset(void);
extern bsp_Result_t bsp_onewire_gpio_Select(const bsp_onewire_addr_t addr);
extern bool bsp_onewire_gpio_SkipRom(void);
extern bsp_Result_t bsp_onewire_gpio_WriteBit(bool write_bit);
extern bsp_Result_t bsp_onewire_gpio_Write(uint8_t v);
extern bsp_Result_t bsp_onewire_gpio_WriteBytes(const uint8_t* buf, size_t count);
extern bsp_Result_t bsp_onewire_gpio_ReadBit(bool* read_bit);
extern bsp_Result_t bsp_onewire_gpio_Read(uint8_t* output_byte);
extern bsp_Result_t bsp_onewire_gpio_ReadBytes(uint8_t* buf, size_t count);
extern bool bsp_onewire_gpio_Power(void);
extern bool bsp_onewire_gpio_Depower(void);
extern void bsp_onewire_gpio_SearchStart(bsp_onewire_search_t* search);
extern void bsp_onewire_gpio_SearchPrefix(bsp_onewire_search_t* search, uint8_t family_code);
extern bsp_onewire_addr_t bsp_onewire_gpio_SearchNext(bsp_onewire_search_t* search);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  /* __BSP_ONEWIRE_GPIO_H__ */
