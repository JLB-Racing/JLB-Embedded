/*
 * MotorControl.c
 *
 *  Created on: Nov 2, 2023
 *      Author: horgo
 */
#include "Servo.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;
/* Angle in +90 to -90 degree */
void SetSteeringAngle(float angle)
{
	uint16_t compare = SERVO_NULL + (angle / 90.0f * SERVO_RANGE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);
}
