/*
 * ADC.h
 *
 *  Created on: Nov 1, 2023
 *      Author: horgo
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "main.h"

typedef struct
{
	uint16_t motor_batt_voltage_raw;
	uint16_t motor_curr_raw;
	uint16_t vbus_raw;
	uint16_t distance_short2_raw;
	uint16_t distance_short1_raw;
	uint16_t distance_long1_raw;
	uint16_t lv_batt_voltage_raw;
	uint16_t distance_long2_raw;

} analog_signals_s;

#endif /* INC_ADC_H_ */
