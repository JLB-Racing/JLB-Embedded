/*
 * Encoder.h
 *
 *  Created on: Oct 30, 2023
 *      Author: horgo
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

#define AB_PULSES_PER_ROT		1024.0f
#define AB_ROT_PER_PULSE		1.0f/AB_PULSES_PER_ROT
#define ENCODER_TASK_TIMESTEP	5.0f //[ms]
#define GEAR_RATIO				0.27085f

typedef struct{
	int16_t velocity;
	uint32_t last_counter_value;
	float rpm;
}encoder_instance;

void Encoder_Task(void * argument);

#endif /* INC_ENCODER_H_ */
