/*
 * LineSensor.c
 *
 *  Created on: Oct 18, 2023
 *      Author: horgo
 */

#include "LineSensor.h"
#include "main.h"

LineSensorData_s ls_data = {0u};

GPIO_TypeDef* front_adc_cs_ports[4] = {ADCF1_CS_GPIO_Port, ADCF2_CS_GPIO_Port, ADCF3_CS_GPIO_Port, ADCF4_CS_GPIO_Port};
uint16_t front_adc_cs_pins[4] = {ADCF1_CS_Pin, ADCF2_CS_Pin, ADCF3_CS_Pin, ADCF4_CS_Pin};

GPIO_TypeDef* rear_adc_cs_ports[4] = {ADCR1_CS_GPIO_Port, ADCR2_CS_GPIO_Port, ADCR3_CS_GPIO_Port, ADCR4_CS_GPIO_Port};
uint16_t rear_adc_cs_pins[4] = {ADCR1_CS_Pin, ADCR2_CS_Pin, ADCR3_CS_Pin, ADCR4_CS_Pin};

GPIO_TypeDef* infra_le_ports[2] = {INFRA_LE_F_GPIO_Port, INFRA_LE_R_GPIO_Port};
uint16_t infra_le_pins[2] = {INFRA_LE_F_Pin, INFRA_LE_R_Pin};

GPIO_TypeDef* infra_oe_ports[2] = {INFRA_OE_F_GPIO_Port, INFRA_OE_R_GPIO_Port};
uint16_t infra_oe_pins[2] = {INFRA_OE_F_Pin, INFRA_OE_R_Pin};


// SPI for reading out ADC values from the line sensors
extern SPI_HandleTypeDef hspi1;
// SPI for controlling the LEDs on the line sensors line sensors
extern SPI_HandleTypeDef hspi2;

/* Turns on every #num and #num + 4 Infraled on every led driving IC.*/
void TurnOnInfraLEDs(GPIO_TypeDef* LE_port[2], uint16_t LE_pin[2],GPIO_TypeDef* OE_port[2], uint16_t OE_pin[2], uint8_t num)
{
	uint8_t i;
	uint8_t data = 0x11;
	data = data << num;

	for(i = 0; i < 4; ++i)
	{
		HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);
	}
	//TODO: maybe add a delay to let the latch in
	HAL_GPIO_WritePin(LE_port[0], LE_pin[0], GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_port[1], LE_pin[1], GPIO_PIN_SET);
	HAL_GPIO_WritePin(LE_port[1], LE_pin[0], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LE_port[1], LE_pin[1], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OE_port[0], OE_pin[0], GPIO_PIN_RESET);
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
	uint8_t i, data;
	for(i = 0; i < 4; ++i)
	{
		HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
		data = num;
		HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi2, &res[i*4], 2, HAL_MAX_DELAY);
		data = 4 + num;
		HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);
		HAL_SPI_Receive(&hspi2, &res[i*4 + 2], 2, HAL_MAX_DELAY);
	}

}
void LineSensorTask(void)
{
	uint8_t i,j = 0;
	uint8_t temp_res_front[16] = {0u};
	uint8_t temp_res_rear[16] = {0u};
	for(i = 0; i < 4; ++i)
	{
		TurnOnInfraLEDs(infra_le_ports, infra_le_pins, infra_oe_ports, infra_le_pins, 0);
		//TODO us delay
		ReadADCValues(front_adc_cs_ports, front_adc_cs_pins, 0, temp_res_front);
		ReadADCValues(rear_adc_cs_ports, rear_adc_cs_pins, 0, temp_res_rear);
		TurnOffInfraLEDs(infra_oe_ports, infra_le_pins);
		for(j = 0; j < 4 ; ++j)
		{
			ls_data.adc_values_f[i*8 + j] = (temp_res_front[4*j] << 8u) | (temp_res_front[4*j + 1]);
			ls_data.adc_values_f[i*8 + j + 4] = (temp_res_front[4*j+2] << 8u) | (temp_res_front[4*j + 3]);

			ls_data.adc_values_r[i*8 + j] = (temp_res_rear[4*j] << 8u) | (temp_res_rear[4*j + 1]);
			ls_data.adc_values_r[i*8 + j + 4] = (temp_res_rear[4*j+2] << 8u) | (temp_res_rear[4*j + 3]);
		}
	}

}
