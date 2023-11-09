/*
 * Encoder.c
 *
 *  Created on: Oct 30, 2023
 *      Author: horgo
 */

#include "Encoder.h"

uint32_t timer_counter = 0u;
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
	encoder_value->rpm = ((float) (encoder_value->velocity) / (float) (ENCODER_TASK_TIMESTEP)) / 2 * 1000.0f * AB_ROT_PER_PULSE * GEAR_RATIO * 60;
	encoder_value->last_counter_value = temp_counter;
}

void Encoder_Task()
{
	timer_counter = __HAL_TIM_GET_COUNTER(&htim3);
	// measure velocity, position
	update_encoder(&enc_instance_mot, &htim3);
}
