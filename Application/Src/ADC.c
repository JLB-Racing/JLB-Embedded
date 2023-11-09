/*
 * ADC.c
 *
 *  Created on: Nov 1, 2023
 *      Author: horgo
 */

#include "ADC.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
analog_signals_s adc_values = {0u};
uint8_t channel_idx = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
	adc_values.values_raw[channel_idx] = adc_value;
	channel_idx++;
	if(channel_idx == 8)
	{
		channel_idx = 0;
	}

	//TODO: enable run in  analog signal dependent tasks
}
