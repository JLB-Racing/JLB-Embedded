/*
 * MotorControl.h
 *
 *  Created on: Nov 2, 2023
 *      Author: horgo
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

#define SERVO_NULL			1500.0f
#define SERVO_RANGE			1500.0f
#define SERVO_ANGLE_OFFSET	4.0f
#define TIMCLOCK   110000000
#define PRESCALAR  110

void SetSteeringAngle(float angle);

#endif /* INC_SERVO_H_ */
