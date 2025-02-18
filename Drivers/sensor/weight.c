/******************************************************************************
 * @file    weight.c
 * @author  MCD Application Team
 * @version V1.1.0
 * @date    17-April-2019
 * @brief   manages the sensors on the application
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "lora_config.h"
#include "weight.h"
#include "tremo_gpio.h"
#include "timer.h"

uint32_t HX711_Buffer = 0;
uint32_t Weight_Maopi = 0;
float GapValue = 400.0;

void WEIGHT_SCK_Init(void)
{
    WEIGHT_SCK_CLK_ENABLE();
    gpio_set_iomux(WEIGHT_SCK_PORT, WEIGHT_SCK_PIN, 0);
    gpio_init(WEIGHT_SCK_PORT, WEIGHT_SCK_PIN,GPIO_MODE_OUTPUT_PP_LOW);
    HX711_SCK_0;
}


void WEIGHT_DOUT_Init(void)
{
    WEIGHT_DOUT_CLK_ENABLE();
    gpio_set_iomux(WEIGHT_DOUT_PORT, WEIGHT_DOUT_PIN, 0);
    gpio_init(WEIGHT_DOUT_PORT, WEIGHT_DOUT_PIN, GPIO_MODE_INPUT_PULL_DOWN);
}


void WEIGHT_SCK_DeInit(void)
{
    WEIGHT_SCK_CLK_ENABLE();
    gpio_set_iomux(WEIGHT_SCK_PORT, WEIGHT_SCK_PIN, 0);
    gpio_init(WEIGHT_SCK_PORT, WEIGHT_SCK_PIN,GPIO_MODE_ANALOG);
}


void WEIGHT_DOUT_DeInit(void)
{
    WEIGHT_DOUT_CLK_ENABLE();
    gpio_set_iomux(WEIGHT_DOUT_PORT, WEIGHT_DOUT_PIN, 0);
    gpio_init(WEIGHT_DOUT_PORT, WEIGHT_DOUT_PIN, GPIO_MODE_ANALOG);
}


uint32_t HX711_Read(void)
{
    uint32_t count;
    uint8_t i;
    HX711_SCK_0;
    count = 0;

    TimerTime_t currentTime = TimerGetCurrentTime();
    while (gpio_read(WEIGHT_DOUT_PORT,WEIGHT_DOUT_PIN) != 0)
    {
        if (TimerGetElapsedTime(currentTime) >= 2000)
        {
            break;
        }
    }
    for (i = 0; i < 24; i++)
    {
        HX711_SCK_1;
        count = count << 1;
        HX711_SCK_0;
        if (gpio_read(WEIGHT_DOUT_PORT,WEIGHT_DOUT_PIN) == 1)
        {
            count++;
        }
    }
    HX711_SCK_1;
    count = count ^ 0x800000;
    HX711_SCK_0;
    return(count);
}


void Get_Maopi(void)
{
    Weight_Maopi = HX711_Read();
}


int32_t Get_Weight(void)
{
    int32_t Weight_Shiwu = 0;

    HX711_Buffer = HX711_Read();
    if (HX711_Buffer != Weight_Maopi)
    {
        Weight_Shiwu = HX711_Buffer;
        Weight_Shiwu = Weight_Shiwu - Weight_Maopi;

        Weight_Shiwu = (int32_t)((float)Weight_Shiwu / GapValue);
    }
    else
    {
        Weight_Shiwu = 0;
    }

    return Weight_Shiwu;
}

