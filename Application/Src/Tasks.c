/*
 * Tasks.c
 *
 *  Created on: Nov 9, 2023
 *      Author: horgo
 */

#include "Tasks.h"
#include "main.h"
#include "cmsis_os.h"

extern uint32_t adc_values_raw[8];
extern ADC_HandleTypeDef hadc1;
void ADCTask(void *argument)
{
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		HAL_ADC_Start_DMA(&hadc1, adc_values_raw, 8u);
		vTaskDelayUntil(&xLastWakeTime, 5u);
	}

}

