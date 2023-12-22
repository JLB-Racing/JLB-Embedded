/*
 * ADC.c
 *
 *  Created on: Nov 1, 2023
 *      Author: horgo
 */

#include "ADC.h"
#include "cmsis_os.h"

extern osThreadId_t mainTaskHandle;
uint32_t adc_values_raw[8];
analog_signals_s adc_values = {0u};
uint8_t channel_idx = 0;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adc_values.motor_batt_voltage_raw = adc_values_raw[0];
	adc_values.motor_curr_raw = adc_values_raw[1];
	adc_values.vbus_raw = adc_values_raw[2];
	adc_values.distance_short2_raw = adc_values_raw[5];
	adc_values.distance_short1_raw = adc_values_raw[3];
	adc_values.distance_long1_raw = adc_values_raw[4];
	adc_values.lv_batt_voltage_raw = adc_values_raw[6];
	adc_values.distance_long2_raw = adc_values_raw[7];

	//vTaskResume(static_cast<TaskHandle_t>(mainTaskHandle));
}
