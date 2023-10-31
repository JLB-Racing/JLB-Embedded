/*
 * Encoder.h
 *
 *  Created on: Oct 30, 2023
 *      Author: horgo
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#define AB_PULSES_PER_ROT		1024.0f
#define AB_ROT_PER_PULSE		1.0f/AB_PULSES_PER_ROT
#define ENCODER_TASK_TIMESTEP	5u //[ms]
#define GEAR_RATIO				1u

typedef struct{
	int16_t velocity;
	uint32_t last_counter_value;
	float rpm;
}encoder_instance;

void Encoder_Task();

#endif /* INC_ENCODER_H_ */
