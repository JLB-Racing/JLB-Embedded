/*
 * Radio.c
 *
 *  Created on: Oct 16, 2023
 *      Author: horgo
 */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "main.h"
#include "Radio.h"

extern UART_HandleTypeDef huart4;
uint8_t radio_rxBuffer[12];
uint8_t character_pointer = 0u;
uint8_t countdown_value = 6u;
bool flood_arived = false;
char pirate_from, pirate_to, pirate_next;
int pirate_percentage = 0;

void Radio_Init()
{
	HAL_UART_Receive_IT(&huart4, &radio_rxBuffer[character_pointer], 1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t length = cahracter_pointer;
	if (radio_rxBuffer[character_pointer] == '\r')
	{
		// Labirinth countdown message received
		if (length == 1)
		{
			countdown_value = radio_rxBuffer[0];
		}
		if (length == 7)
		{
			//FLOOD message received
			if (!strcmp("FLOOD!\r", reinterpret_cast<const char*>(radio_rxBuffer)))
			{
				flood_arived = true;
			}
			else
			{
				sscanf(reinterpret_cast<const char*>(radio_rxBuffer), "%c%c%c%03d", &pirate_from, &pirate_to, &pirate_next, &pirate_percentage);
			}
		}

		character_pointer = 0u;
	}
	else
	{
		character_pointer++;
	}
	HAL_UART_Receive_IT(&huart4, &radio_rxBuffer[character_pointer], 1);

}
