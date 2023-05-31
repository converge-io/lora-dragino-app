/** \file
 *****************************************************************************************************************************************************
 *  \brief      bsp_board header file
 *  \copyright  Copyright &copy; 2019, Octagon IO Ltd t/a Converge. All Rights Reserved.
 *
 *  \author     jon
 *  \date       6 Mar 2019
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
 *  \section HEADER_GUARD
 *****************************************************************************************************************************************************
 */
#ifndef BSP_BOARD_DRAGINO_H_
#define BSP_BOARD_DRAGINO_H_

#ifdef __cplusplus
extern "C"
{
#endif


/**
 *****************************************************************************************************************************************************
 *  \section INCLUDE FILES
 *****************************************************************************************************************************************************
 */
#include "bsp.h"
#include "bsp_onewire_gpio.h"
#include "tremo_gpio.h"
#include "tremo_rtc.h"


/**
 *****************************************************************************************************************************************************
 *  \section GLOBAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */
#define BSP_BOARD_TAIL_ONE_WIRE_PORT            GPIOC
#define BSP_BOARD_TAIL_ONE_WIRE_PIN             GPIO_PIN_13
#define BSP_BOARD_TAIL_CLK_ENABLE()             rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOC, true)

#define BSP_BOARD_TAIL_DETECT_PORT              GPIOA
#define BSP_BOARD_TAIL_DETECT_PIN               GPIO_PIN_8
#define BSP_BOARD_TAIL_DETECT_CLK_ENABLE()      rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOA, true)               

#define BSP_ONEWIRE_OBJECT                      BSP_ONEWIRE_DEFINE(_gpio_)


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


/**
 *****************************************************************************************************************************************************
 *  \section EXTERNAL FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */


#ifdef __cplusplus
}
#endif


#endif  /* BSP_BOARD_DRAGINO_H_ */


/**
 *****************************************************************************************************************************************************
 *  END OF FILE
 *****************************************************************************************************************************************************
 */
