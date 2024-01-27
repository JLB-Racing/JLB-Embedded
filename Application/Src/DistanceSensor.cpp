/*
 * DistanceSensor.c
 *
 *  Created on: Oct 30, 2023
 *      Author: horgo
 */

#include "DistanceSensor.h"
#include "ADC.h"
#include "main.h"
#include "math.h"


float short_poly_coeff[6] = { -1.72491320516644, 16.7372727496576, -64.1431581045353, 123.547133090677, -125.967551212446, 64.2367235368081 };
float long_poly_coeff[6] = { -1.38479217287386, 25.0295818844100, -148.629784172265, 397.848209142987, -517.458334094517, 306.722604866572 };

extern analog_signals_s adc_values;
DistanceSensorData_s distance_sensor = {0u};


void DistanceSensorTask()
{
	uint8_t i,j;
	distance_sensor.voltage_long[0] = ((float)(adc_values.distance_long1_raw)) / 4096.0f * 3.3f;
	distance_sensor.voltage_long[1] = ((float)(adc_values.distance_long2_raw)) / 4096.0f * 3.3f;
	distance_sensor.voltage_short[0] = ((float)(adc_values.distance_short1_raw)) / 4096.0f * 3.3f;
	distance_sensor.voltage_short[1] = ((float)(adc_values.distance_short2_raw)) / 4096.0f * 3.3f;

	for(i = 0; i < 1; ++i)
	{
		float x = distance_sensor.voltage_long[i];
		distance_sensor.distance_long[i] = 0.0f;
		for(j = 0; j < 6; ++j)
		{
			distance_sensor.distance_long[i] += pow(x,5-j) * long_poly_coeff[j];
		}

		distance_sensor.distance_short[i] = 0.0f;
		x = distance_sensor.voltage_short[i];
		for(j = 0; j < 6; ++j)
		{
			distance_sensor.distance_short[i] += pow(x,5-j) * short_poly_coeff[j];
		}
	}

	if((distance_sensor.distance_short[0] >= 20.0f) || (distance_sensor.distance_short[1] >= 20.0f))
	{
		//distance_sensor.distance = (distance_sensor.distance_long[0] + distance_sensor.distance_long[1]) / 2.0f;
		distance_sensor.distance = distance_sensor.distance_long[0];
		distance_sensor.distance -= 5.0f;
	}
	else
	{
		distance_sensor.distance = (distance_sensor.distance_short[0] + distance_sensor.distance_short[1]) / 2.0f;
		distance_sensor.distance -= 8.0f;

		//distance_sensor.distance = distance_sensor.distance_short[0];
	}

	distance_sensor.distance /= 100.0f;
}
