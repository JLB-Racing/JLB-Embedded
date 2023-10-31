/*
 * DistanceSensor.c
 *
 *  Created on: Oct 30, 2023
 *      Author: horgo
 */

float short_poly_coeff[6] = { -1.72491320516644, 16.7372727496576, -64.1431581045353, 123.547133090677, -125.967551212446, 64.2367235368081 };
float long_poly_coeff[6] = { -1.38479217287386, 25.0295818844100, -148.629784172265, 397.848209142987, -517.458334094517, 306.722604866572 };

extern uint16_t adc_values[10];

void DistanceSensorTask()
{
	adc_values[ADC_CHANNEL_LONG1]
}
