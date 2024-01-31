/*
 * Encoder.c
 *
 *  Created on: Oct 30, 2023
 *      Author: horgo
 */

#include "Encoder.h"
#include "cmsis_os.h"

encoder_instance enc_instance_mot;
extern TIM_HandleTypeDef htim3;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim)
{
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	static uint8_t first_time = 0;
	if (!first_time)
	{
		encoder_value->velocity = 0;
		first_time = 1;
	}
	else
	{
		if (temp_counter == encoder_value->last_counter_value)
		{
			encoder_value->velocity = 0;
		}
		else if (temp_counter > encoder_value->last_counter_value)
		{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value->velocity = -encoder_value->last_counter_value - (__HAL_TIM_GET_AUTORELOAD(htim) - temp_counter);
			}
			else
			{
				encoder_value->velocity = temp_counter - encoder_value->last_counter_value;
			}
		}
		else
		{
			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value->velocity = temp_counter - encoder_value->last_counter_value;
			}
			else
			{
				encoder_value->velocity = temp_counter + (__HAL_TIM_GET_AUTORELOAD(htim) - encoder_value->last_counter_value);
			}
		}
	}
	//encoder_value->rpm = ((float) (encoder_value->velocity) / ENCODER_TASK_TIMESTEP) / 2 * 1000.0f * AB_ROT_PER_PULSE * GEAR_RATIO * 60;
	encoder_value->last_counter_value = temp_counter;
}

void Encoder_Task(void * argument)
{
	// measure velocity, position
	int16_t velocity_values[60] = {0};
	uint8_t index = 0;
	uint8_t i;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		update_encoder(&enc_instance_mot, &htim3);
		velocity_values[index++] = enc_instance_mot.velocity;

		if(index == 60)
		{
			index = 0;
		}

		enc_instance_mot.rpm = 0.0f;
		for(i = 0 ; i < 60; ++i)
		{
			enc_instance_mot.rpm += ((float)(velocity_values[i])) / 60.0f;
		}

		vTaskDelayUntil(&xLastWakeTime, 1u);
	}
}

float CalculateRPM()
{
	uint8_t i;
	static uint8_t index = 0u;
	static float rpm_averaging_array[10];
	float averaged_rpm = 0.0f;
	/*rpm_averaging_array[index++] = enc_instance_mot.rpm;

	if(index == 10)
	{
		index = 0;
	}

	averaged_rpm = 0.0f;
	for(i = 0 ; i < 10; ++i)
	{
		averaged_rpm += rpm_averaging_array[i] / 10.0f;
	}*/

	averaged_rpm = enc_instance_mot.rpm;
	averaged_rpm *= -1.36f;

	return averaged_rpm;
}
