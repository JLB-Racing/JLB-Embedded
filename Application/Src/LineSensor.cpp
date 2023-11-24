/*
 * LineSensor.c
 *
 *  Created on: Oct 18, 2023
 *      Author: horgo
 */

#include "LineSensor.h"
#include "main.h"

//TODO: remove this after LED control has been tested
//#define LINE_SENSOR_LED_TEST

LineSensorData_s ls_data = {0u};

GPIO_TypeDef* front_adc_cs_ports[4] = {ADCF1_CS_GPIO_Port, ADCF2_CS_GPIO_Port, ADCF3_CS_GPIO_Port, ADCF4_CS_GPIO_Port};
uint16_t front_adc_cs_pins[4] = {ADCF1_CS_Pin, ADCF2_CS_Pin, ADCF3_CS_Pin, ADCF4_CS_Pin};

GPIO_TypeDef* rear_adc_cs_ports[4] = {ADCR1_CS_GPIO_Port, ADCR2_CS_GPIO_Port, ADCR3_CS_GPIO_Port, ADCR4_CS_GPIO_Port};
uint16_t rear_adc_cs_pins[4] = {ADCR1_CS_Pin, ADCR2_CS_Pin, ADCR3_CS_Pin, ADCR4_CS_Pin};

GPIO_TypeDef* infra_le_ports[2] = {INFRA_LE_F_GPIO_Port, INFRA_LE_R_GPIO_Port};
uint16_t infra_le_pins[2] = {INFRA_LE_F_Pin, INFRA_LE_R_Pin};

GPIO_TypeDef* infra_oe_ports[2] = {INFRA_OE_F_GPIO_Port, INFRA_OE_R_GPIO_Port};
uint16_t infra_oe_pins[2] = {INFRA_OE_F_Pin, INFRA_OE_R_Pin};

GPIO_TypeDef* led_le_ports[2] = {LED_LE_F_GPIO_Port, LED_LE_R_GPIO_Port};
uint16_t led_le_pins[2] = {LED_LE_F_Pin, LED_LE_R_Pin};

GPIO_TypeDef* led_oe_ports[2] = {LED_OE_F_GPIO_Port, LED_OE_R_GPIO_Port};
uint16_t led_oe_pins[2] = {LED_OE_F_Pin, LED_OE_R_Pin};


// SPI for reading out ADC values from the line sensors
extern SPI_HandleTypeDef hspi1;
// SPI for controlling the LEDs on the line sensors line sensors
extern SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim6;

uint16_t infra_adc_values_test[32];
uint8_t infra_adc_data[32*2];

/* Turns on every #num and #num + 4 Infraled on every led driving IC.*/
void TurnOnInfraLEDs(GPIO_TypeDef* LE_port[2], uint16_t LE_pin[2],GPIO_TypeDef* OE_port[2], uint16_t OE_pin[2], uint8_t num)
{
	uint8_t i;
	uint8_t data = 0x11 << num;

	for(i = 0; i < 4; ++i)
	{
		HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
	}
	//TODO: maybe add a delay to let the latch in
	HAL_GPIO_WritePin(LE_port[0], LE_pin[0], GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_port[1], LE_pin[1], GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_port[0], LE_pin[0], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LE_port[1], LE_pin[1], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OE_port[0], OE_pin[0], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OE_port[1], OE_pin[1], GPIO_PIN_RESET);

}

void TurnOnLEDs(GPIO_TypeDef *LE_port[2], uint16_t LE_pin[2], GPIO_TypeDef *OE_port[2], uint16_t OE_pin[2], uint32_t front, uint32_t rear)
{
	HAL_GPIO_WritePin(OE_port[0], OE_pin[0], GPIO_PIN_SET);
	HAL_GPIO_WritePin(OE_port[1], OE_pin[1], GPIO_PIN_SET);
	uint8_t i;
	for (i = 0; i < 4; ++i)
	{
		uint8_t data_front = (front >> (8u*i)) & 0xFF;
		HAL_SPI_Transmit(&hspi2, &data_front, 1, HAL_MAX_DELAY);
	}
	//TODO: maybe add a delay to let the latch in
	HAL_GPIO_WritePin(LE_port[0], LE_pin[0], GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_port[0], LE_pin[0], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OE_port[0], OE_pin[0], GPIO_PIN_RESET);
	for (i = 0; i < 4; ++i)
	{
		uint8_t data_rear = (rear >> (8u*i)) & 0xFF;
		HAL_SPI_Transmit(&hspi2, &data_rear, 1, HAL_MAX_DELAY);
	}

	//TODO: maybe add a delay to let the latch in
	HAL_GPIO_WritePin(LE_port[1], LE_pin[1], GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_port[1], LE_pin[1], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OE_port[1], OE_pin[1], GPIO_PIN_RESET);
}


void TurnOffInfraLEDs(GPIO_TypeDef* OE_port[2], uint16_t OE_pin[2])
{
	HAL_GPIO_WritePin(OE_port[0], OE_pin[0], GPIO_PIN_SET);
	HAL_GPIO_WritePin(OE_port[1], OE_pin[1], GPIO_PIN_SET);
}

/* Reads out from all of the 4 adc ICs on one line sensor card 2 adc values each defined by num and writes it to res */
void ReadADCValues(GPIO_TypeDef* ports[4], uint16_t pins[4], uint8_t num, uint8_t *res)
{
	uint8_t i;
	uint8_t tmp[2] = {0, 0};

	for(i = 0; i < 4; ++i)
	{
		HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
		uint32_t delay_start = __HAL_TIM_GetCounter(&htim6);
		while((__HAL_TIM_GetCounter(&htim6) - delay_start) < INFRA_WAIT_TIME);
		tmp[0] = num << 3u;
		HAL_SPI_Transmit(&hspi1, tmp, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, &res[i*4], 2, HAL_MAX_DELAY);

		tmp[0] = (4 + num) << 3u;
		HAL_SPI_Transmit(&hspi1, tmp, 2, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi1, &res[i*4 + 2], 2, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
	}

}
void LineSensorTask(void)
{
#ifdef LINE_SENSOR_LED_TEST
	static uint32_t leds_front = 1u;
	static uint32_t leds_rear = 0x80000000;

	leds_front = leds_front << 1u;
	leds_rear = leds_rear >> 1u;
	if(leds_front == 0u)
	{
		leds_front = 1u;
		leds_rear = 0x80000000;
	}

	//infra led test
	TurnOnLEDs(infra_le_ports, infra_le_pins, infra_oe_ports, infra_oe_pins, 0x00000000, 0xFFFFFFFF);
	TurnOnLEDs(led_le_ports, led_le_pins, led_oe_ports, led_oe_pins, 0x00000000, 0xFFFFFFFF);
	uint32_t delay_start = __HAL_TIM_GetCounter(&htim6);
	while((__HAL_TIM_GetCounter(&htim6) - delay_start) < INFRA_WAIT_TIME);
	uint8_t i, j;
	uint8_t tmp[2] = {0, 0};
	for(i = 0; i < 4; ++i)
	{
		HAL_GPIO_WritePin(rear_adc_cs_ports[i], rear_adc_cs_pins[i], GPIO_PIN_RESET);
		for(j = 0; j < 8; ++j)
		{
			tmp[0] = j << 3u;
			HAL_SPI_Transmit(&hspi1, tmp, 2, HAL_MAX_DELAY);
			HAL_SPI_Receive(&hspi1, &infra_adc_data[i*8 + j*2], 2, HAL_MAX_DELAY);
			infra_adc_values_test[i*8 + j] = (uint16_t)(infra_adc_data[i*8 + j*2] << 8u) | (infra_adc_data[i*8 + j*2 + 1]);

		}
		HAL_GPIO_WritePin(rear_adc_cs_ports[i], rear_adc_cs_pins[i], GPIO_PIN_SET);
	}


#else
	uint8_t j,k = 0;
	int8_t i;
	for(i = 0; i < 4; ++i)
	{
		TurnOnInfraLEDs(infra_le_ports, infra_le_pins, infra_oe_ports, infra_le_pins, i);
		uint32_t delay_start = __HAL_TIM_GetCounter(&htim6);
		while((__HAL_TIM_GetCounter(&htim6) - delay_start) < INFRA_WAIT_TIME);

		for(j = 0; j < 4; ++j)
		{
			uint8_t tmp[2] = {0, 0};
			uint8_t adc_ic_values[16];
			HAL_GPIO_WritePin(rear_adc_cs_ports[j], rear_adc_cs_pins[j], GPIO_PIN_RESET);
			for(k = 0; k < 8; ++k)
			{
				tmp[0] = k << 3u;
				HAL_SPI_Transmit(&hspi1, tmp, 2, HAL_MAX_DELAY);
				HAL_SPI_Receive(&hspi1, &adc_ic_values[k*2], 2, HAL_MAX_DELAY);
			}
			HAL_GPIO_WritePin(rear_adc_cs_ports[j], rear_adc_cs_pins[j], GPIO_PIN_SET);

			// i = id of LED, j = id of ic, adc_ic_values conotaint the 8 adc values from one ic
			ls_data.adc_values_r[j*8 + i] = (uint16_t)(adc_ic_values[i*2] << 8u) | (adc_ic_values[i*2+1]);
			ls_data.adc_values_r[j*8 + i + 4] = (uint16_t)(adc_ic_values[(i*2) + 8] << 8u) | (adc_ic_values[(i*2) + 8 + 1]);
		}

		TurnOffInfraLEDs(infra_oe_ports, infra_le_pins);
	}
	TurnOffInfraLEDs(led_oe_ports, led_oe_pins);

	float denominator_f = 0.0f;
	float denominator_r = 0.0f;
	uint32_t led_front = 0u;
	uint32_t led_rear = 0u;
	for(i = 2; i <= 31; ++i)
	{
		ls_data.position_front += (float)((i - 16 + 0.5f) * ls_data.adc_values_f[i-1]);
		ls_data.position_rear += (float)((i - 16 + 0.5f) * ls_data.adc_values_r[i-1]);
		denominator_f += (float)(ls_data.adc_values_f[i-1]);
		denominator_r += (float)(ls_data.adc_values_r[i-1]);
		if(ls_data.adc_values_f[i-1] > 1400)
		{
			led_front |= 1 << (i-2);
		}

		if(ls_data.adc_values_r[i-1] > 1400)
		{
			led_rear |= 0x70000000 >> (i-2);
		}
	}

	TurnOnLEDs(led_le_ports, led_le_pins, led_oe_ports, led_oe_pins, led_front, led_rear);

	ls_data.position_front /= denominator_f;
	ls_data.position_rear /= denominator_r;


#endif
}