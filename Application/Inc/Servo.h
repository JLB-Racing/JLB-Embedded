/*
 * MotorControl.h
 *
 *  Created on: Nov 2, 2023
 *      Author: horgo
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#define PWM_PERIOD			20000 //[us]
#define SERVO_NULL			1500.0f
#define SERVO_RANGE			700.0f


void SetSteeringAngle(float angle);

#endif /* INC_SERVO_H_ */
