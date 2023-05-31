
/** \file
 *****************************************************************************************************************************************************
 *  \brief      bsp_onewire source file
 *  \copyright  Copyright &copy; 2019, Octagon IO Ltd t/a Converge. All Rights Reserved.
 *
 *  \author     rahul
 *  \date       25 May 2023
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
#include "bsp_thermal_tail.h"
#include "bsp.h"
#include "bsp_onewire.h"
#include "log.h"
#include "bsp_sensor_ds18x20.h"
#include "bsp_sensor_ds28ea00.h"

#define BSP_SENSOR_THERMAL_TAIL_SUPPORTED_SENSORS             (20UL)
#define DS28_FAMILY_ID (0x42)


/**
 *****************************************************************************************************************************************************
 *  \section STATIC FUNCTION PROTOTYPES
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  \section STATIC VARIABLES
 *****************************************************************************************************************************************************
 */
static bsp_onewire_addr_t address_list[BSP_SENSOR_THERMAL_TAIL_SUPPORTED_SENSORS];
/**
 *****************************************************************************************************************************************************
 *  \section LOCAL DEFINITIONS/MACROS
 *****************************************************************************************************************************************************
 */

/**
 *****************************************************************************************************************************************************
 *  \section FUNCTION DEFINITIONS
 *****************************************************************************************************************************************************
 */


/**
 *****************************************************************************************************************************************************
 *  @brief 
 *  @params [in]
 *  @param [out] 
 *  @return 
 *****************************************************************************************************************************************************
 */
void bsp_thermal_tail_Init(void);


/**
 *****************************************************************************************************************************************************
 *  @brief 
 *  @params [in]
 *  @param [out] 
 *  @return *****************************************************************************************************************************************************
 */
int bsp_thermal_tail_Discover(void)
{
    int devices_found = 0;
    /* Sets the pin to its default state, useful because the bsp method that sets the pin as input does not actually init its state */
    onewire.Init();

    /* Delay very quickly */

    /* We take the assumption here that if the number of nodes on the thermal tails is bigger than one the tail is ds28ea00 based, this */
    /* is just temporary before RFC018 is implemented, after that the type of the sensors will be written in configuration */
    /*
    //if (expected > 1)
    {
    }
    else
    {
    */
    devices_found = bsp_onewire_ScanDevices(address_list, sizeof(address_list) / sizeof(address_list[0]));

    onewire.Deinit();
    uint8_t family_id = address_list[0] & 0xFF;
    LOG_PRINTF(LL_DEBUG,"\n\rFamily ID: 0x%x\r\n",family_id);

    return devices_found;
}

/**
 *****************************************************************************************************************************************************
 *  @brief 
 *  @params [in]
 *  @param [out] 
 *  @return 
 *****************************************************************************************************************************************************
 */
bsp_Result_t bsp_thermal_tail_Read(sample_t* sample)
{
    int probes = bsp_thermal_tail_Discover();
    float temperature;

    LOG_PRINTF(LL_DEBUG,"\n\rDetected %d probes \r\n",probes);
    for (int i = 0; i < probes; i++)
    {
        LOG_PRINTF(LL_DEBUG,"\n\rAddress: 0x%08X%08X \r\n",(uint32_t)(address_list[i] >> 32), (uint32_t)address_list[i]);
    }


    bsp_sensor_ds18x20_MeasureAndRead(address_list[0], &temperature);
    LOG_PRINTF(LL_DEBUG,"\n\rTemperature: %f\r\n",temperature);

}


/**
 *****************************************************************************************************************************************************
 *  END OF FILE
 *****************************************************************************************************************************************************
 */
