/*
 * MotorControl.c
 *
 *  Created on: Nov 2, 2023
 *      Author: horgo
 */

#include "MotorControl.h"
#include "main.h"
#include "ADC.h"
#include "JLB/pid.hxx"

extern analog_signals_s adc_values;
extern MotorControData_s motorcontrol = {0};
extern TIM_HandleTypeDef htim5;
extern uint32_t usWidth_throttle;
float pi_integral_error = 0.0f;

PID motorcontrol_pid{SPEED_CONTROLER_KP,SPEED_CONTROLLER_KI, SPEED_CONTROLLER_KD, SPEED_CONTROLLER_TAU, SPEED_CONTROLLER_T, SPEED_CONTROLLER_MIN, SPEED_CONTROLLER_MAX, SPEED_CONTROLLER_DEADBAND,SPEED_CONTROLLER_DERIVATIVE_FILTER_ALPHA};

void MotorControlTask()
{
	if((usWidth_throttle > 1800) && (usWidth_throttle < 2800))
	{
		HAL_GPIO_WritePin(DRIVE_ENABLE_GPIO_Port, DRIVE_ENABLE_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(DRIVE_ENABLE_GPIO_Port, DRIVE_ENABLE_Pin, GPIO_PIN_RESET);
		motorcontrol.target_velocity = 0.0f;
		pi_integral_error = 0.0f;
	}
	motorcontrol.battery_voltage = ((float)(adc_values.motor_batt_voltage_raw)) / 4096.0f * 3.3f * ANALOG_TO_MOTOR_BATT;
	motorcontrol.motor_current = (((float)((adc_values.motor_curr_raw)) / 4096.0f) * 3.3f - MOTOR_CURR_NULL) / MOTOR_CURR_SENSITIVITY;

	/*float pi_error = motorcontrol.target_velocity - motorcontrol.actual_velocity;
	float pi_proportional_error = pi_error * SPEED_CONTROLER_KP;
	pi_integral_error += pi_error * SPEED_CONTROLLER_KI / MOTOR_CONTROL_TASK_FREQ;
	motorcontrol.duty_cycle = pi_integral_error + pi_proportional_error;
	*/

	motorcontrol.duty_cycle = motorcontrol_pid.update(motorcontrol.target_velocity, motorcontrol.actual_velocity, 0.005f);
	motorcontrol.duty_cycle += 0.5f;
	motorcontrol.duty_cycle = (motorcontrol.duty_cycle > 0.95f) ? 0.95f : motorcontrol.duty_cycle;
	motorcontrol.duty_cycle = (motorcontrol.duty_cycle < 0.5f) ? 0.05f : motorcontrol.duty_cycle;


	if((motorcontrol.target_velocity == 0.0f) && (motorcontrol.actual_velocity < 1.0f) && (motorcontrol.actual_velocity >= -1.0f))
	{
		motorcontrol.duty_cycle = 0.5f;
	}

	if(motorcontrol.target_velocity >= motorcontrol.actual_velocity && motorcontrol.duty_cycle < 0.5f)
	{
		motorcontrol.duty_cycle = 0.5f;

	}

	//float duty_cycle = 0.6f;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, motorcontrol.duty_cycle * PWM_COUNTER_PREIOD);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM_COUNTER_PREIOD - (motorcontrol.duty_cycle * PWM_COUNTER_PREIOD));

}
