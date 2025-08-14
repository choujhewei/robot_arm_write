/*
 * pressure_sensor.cpp
 *
 *  Created on: Aug 14, 2025
 *      Author: jeffr
 */
#include "pressure_sensor.h"
#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx.h"
#include <cstdint>

uint16_t Read_Pressure_Right(void)
{
    LL_ADC_REG_StartConversionSWStart(ADC1);   // 每次都手動啟動一次轉換
    while (!LL_ADC_IsActiveFlag_EOCS(ADC1));   // 確保轉換完成
    return LL_ADC_REG_ReadConversionData12(ADC1);
}

uint16_t Read_Pressure_Left(void)
{
    LL_ADC_REG_StartConversionSWStart(ADC2);   // 每次都手動啟動一次轉換
    while (!LL_ADC_IsActiveFlag_EOCS(ADC2));   // 確保轉換完成
    return LL_ADC_REG_ReadConversionData12(ADC2);
}

