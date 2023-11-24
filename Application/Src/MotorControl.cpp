/*
 * MotorControl.c
 *
 *  Created on: Nov 2, 2023
 *      Author: horgo
 */

#include "MotorControl.h"
#include "main.h"
#include "ADC.h"

extern analog_signals_s adc_values;
MotorControData_s motorcontrol = {0};
extern TIM_HandleTypeDef htim5;

float pi_integral_error = 0;

void MotorControlTask()
{
	HAL_GPIO_WritePin(DRIVE_ENABLE_GPIO_Port, DRIVE_ENABLE_Pin, GPIO_PIN_SET);
	motorcontrol.battery_voltage = ((float)(adc_values.motor_batt_voltage_raw)) / 4096.0f * 3.3f * ANALOG_TO_MOTOR_BATT;
	motorcontrol.motor_current = (((float)((adc_values.motor_curr_raw)) / 4096.0f) * 3.3f - MOTOR_CURR_NULL) / MOTOR_CURR_SENSITIVITY;

	float pi_error = motorcontrol.target_velocity - motorcontrol.actual_velocity;
	float pi_proportional_error = pi_error * SPEED_CONTROLER_KP;
	pi_integral_error = pi_error * SPEED_CONTROLLER_KI / MOTOR_CONTROL_TASK_FREQ;

	//float duty_cycle = pi_integral_error + pi_proportional_error;
	float duty_cycle = 0.6f;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, duty_cycle * PWM_COUNTER_PREIOD);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM_COUNTER_PREIOD - (duty_cycle * PWM_COUNTER_PREIOD));

}