/*
 * Tasks.c
 *
 *  Created on: Nov 9, 2023
 *      Author: horgo
 */

#include "Tasks.h"
#include "main.h"
#include "cmsis_os.h"
#include "Servo.h"
#include "MotorControl.h"

extern uint16_t adc_values_raw[8];
extern ADC_HandleTypeDef hadc1;
float pwm_servo_test = 0.0f;

osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes =
{ .name = "ADCTask", .priority = (osPriority_t) osPriorityRealtime, .stack_size = 128 * 4 };

osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes =
{ .name = "MainTask", .priority = (osPriority_t) osPriorityRealtime1, .stack_size = 512 * 4 };

void RegistrateUserTasks()
{
	adcTaskHandle = osThreadNew(ADCTask, NULL, &adcTask_attributes);
	mainTaskHandle = osThreadNew(MainTask, NULL, &mainTask_attributes);

}

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

void MainTask(void * argument)
{
	static uint8_t direction = 1u;
	for (;;)
	{
		SetSteeringAngle(5.0f);
		/*if(pwm_servo_test > 19.0f)
		{
			direction = 0u;
		}
		else if(pwm_servo_test < -19.0f)
		{
			direction = 1u;
		}
		if(direction == 1u)
		{
			pwm_servo_test+= 0.1f;
		}
		else
		{
			pwm_servo_test-= 0.1f;
		}*/
		MotorControlTask();

		vTaskSuspend(mainTaskHandle);
	}
}
