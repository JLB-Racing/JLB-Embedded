/*
 * LineSensor.h
 *
 *  Created on: Oct 18, 2023
 *      Author: horgo
 */

#ifndef INC_LINESENSOR_H_
#define INC_LINESENSOR_H_

#include "main.h"
#include <vector>

#define INFRA_WAIT_TIME			150 //[us]

typedef struct
{
	uint16_t adc_values_f[32];
	uint16_t adc_values_r[32];

	bool front_detection[32];
	bool rear_detection[32];

	float position_front;
	float position_rear;

	std::vector<float> front;
	std::vector<float> rear;
}LineSensorData_s;

void LineSensorTask(void);
float getPercentageFront(void);
#endif /* INC_LINESENSOR_H_ */
