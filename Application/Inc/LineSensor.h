/*
 * LineSensor.h
 *
 *  Created on: Oct 18, 2023
 *      Author: horgo
 */

#ifndef INC_LINESENSOR_H_
#define INC_LINESENSOR_H_

#include "main.h"

#define INFRA_WAIT_TIME			350 //[us]

typedef struct
{
	uint16_t adc_values_f[32];
	uint16_t adc_values_r[32];

	float position_front;
	float position_rear;
}LineSensorData_s;

void LineSensorTask(void);

#endif /* INC_LINESENSOR_H_ */
