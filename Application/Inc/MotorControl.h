/*
 * MotorControl.h
 *
 *  Created on: Nov 2, 2023
 *      Author: horgo
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#include "main.h"
#include "ADC.h"

#define ANALOG_TO_MOTOR_BATT		5.7f
#define MOTOR_CURR_NULL				1.66f
#define	MOTOR_CURR_SENSITIVITY		0.00886f

#define SPEED_CONTROLER_KP			1.0f
#define SPEED_CONTROLLER_KI			1.0f
#define MOTOR_CONTROL_TASK_FREQ		200 //[Hz]

#define PWM_COUNTER_PREIOD			1834

typedef struct
{
	float battery_voltage;
	float motor_current;
	float actual_velocity;
	float target_velocity;
	float duty_cycle;
} MotorControData_s;

void MotorControlTask();

#endif /* INC_MOTORCONTROL_H_ */