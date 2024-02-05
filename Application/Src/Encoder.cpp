/*
 * Encoder.c
 *
 *  Created on: Oct 30, 2023
 *      Author: horgo
 */

#include "Encoder.h"
#include "cmsis_os.h"
#include "math.h"
#include <deque>

#define ALPHA 0.1f

encoder_instance enc_instance_mot = {0, 0, 0, 0.0f};
extern TIM_HandleTypeDef htim3;
int64_t median_array[11];
uint8_t median_index = 0;

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
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	uint8_t circular_index = 0;
	for(;;)
	{
		//update_encoder(&enc_instance_mot, &htim3);
		median_array[circular_index++] = enc_instance_mot.velocity;

		if(circular_index == 5u)
		{
			circular_index = 0u;
		}
		vTaskDelayUntil(&xLastWakeTime, 2u);
	}
}

float CalculateRPM()
{
	update_encoder(&enc_instance_mot, &htim3);
	/*if(std::abs(enc_instance_mot.velocity * -1.0f / 10.0f* 1.29230f) < 950)
	{
		int64_t tmp = enc_instance_mot.velocity_prev;
		enc_instance_mot.velocity_prev = enc_instance_mot.velocity;
		return (enc_instance_mot.velocity * -1.0f / 10.0f* 1.29230f * 0.8f) + (tmp * -1.0f / 10.0f* 1.29230f *0.2f);
	}
	return enc_instance_mot.velocity_prev * -1.0f / 10.0f* 1.29230f;
	*/
	median_array[median_index++] = enc_instance_mot.velocity;

	if(median_index == 11)
	{
		median_index = 0;
	}

	float sorted_arr[11];
	uint8_t i;
	for(i = 0; i < 11; ++i)
	{
		sorted_arr[i] = median_array[i] * -1.0f / 10.0f* 1.29230f;
	}
	std::sort(std::begin(sorted_arr), std::end(sorted_arr));

	return sorted_arr[5];

}
