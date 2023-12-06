/*
 * MotorControl.c
 *
 *  Created on: Nov 2, 2023
 *      Author: horgo
 */
#include "Servo.h"
#include "main.h"

extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim1;
/* Measure Width */
uint32_t usWidth_throttle = 0;
uint32_t usWidth_steering = 0;


uint32_t IC_Val1 = 0;
uint32_t IC_Val1_prev = 0;
float period_length = 0.0f;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;


/* Angle in +90 to -90 degree */
void SetSteeringAngle(float angle)
{
	uint16_t compare = SERVO_NULL + (angle/ 21.5f * SERVO_RANGE);
	//uint16_t compare = (uint16_t)(angle);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, compare);
}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1_prev = IC_Val1;
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
		}

		else   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffffffff - IC_Val1) + IC_Val2;
			}

			float refClock = TIMCLOCK/(PRESCALAR);
			float mFactor = 1000000/refClock;

			usWidth_throttle = Difference*mFactor;
			period_length = refClock / Difference;

			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
			Is_First_Captured = 0; // set it back to false
		}

	}
}
