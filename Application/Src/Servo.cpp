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


uint32_t falling_value = 0;
float period_length = 0.0f;
uint8_t Is_First_Captured = 0;


/* Angle in +90 to -90 degree */
void SetSteeringAngle(float angle)
{
	if((usWidth_throttle > 1800) && (usWidth_throttle < 2800))
	{
		uint16_t compare = SERVO_NULL + (angle/ 22.5f * SERVO_RANGE);
	    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, compare);	}
	else
	{
	    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	}

}



void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// RISING
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
	{
		__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
	}
	//FALLING
	else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
	{
		falling_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);  // read second value


		float refClock = TIMCLOCK/(PRESCALAR);
		float mFactor = 1000000/refClock;

		usWidth_throttle = falling_value*mFactor;
	}
}
