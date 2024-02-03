/*
 * DistanceSensor.h
 *
 *  Created on: Oct 30, 2023
 *      Author: horgo
 */

#ifndef INC_DISTANCESENSOR_H_
#define INC_DISTANCESENSOR_H_

#define ADC_CHANNEL_LONG1	5
#define ADC_CHANNEL_LONG2	4
#define ADC_CHANNEL_SHORT1	6
#define ADC_CHANNEL_SHORT2	9

typedef struct
{
	float voltage_long[2];
	float voltage_short[2];
	float distance_long[2];
	float distance_short[2];

	float distance;
} DistanceSensorData_s;

void DistanceSensorTask(float steering_angle);


#endif /* INC_DISTANCESENSOR_H_ */
