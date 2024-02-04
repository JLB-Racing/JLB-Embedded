/*
 * Radio.c
 *
 *  Created on: Oct 16, 2023
 *      Author: horgo
 */
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "Radio.h"
#include "Tasks.h"


extern UART_HandleTypeDef huart4;
uint8_t radio_rxBuffer[12];
uint8_t character_pointer = 0u;
uint8_t countdown_value = 6u;
char pirate_from, pirate_to, pirate_next;
int pirate_percentage = 0;
bool flood_arrived = false;


void Radio_Init()
{
	HAL_UART_Receive_IT(&huart4, &radio_rxBuffer[character_pointer], 1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t length = character_pointer - 1;
	uint8_t i;
	if (radio_rxBuffer[character_pointer - 1] == '\r')
	{
		// Labirinth countdown message received
		if (length == 1)
		{
			countdown_value = radio_rxBuffer[0] - '0';
		}
		if (length == 6)
		{
			//FLOOD message received
			if (!strncmp("FLOOD!\r", reinterpret_cast<const char*>(radio_rxBuffer),6))
			{
				flood_arrived = true;
			}
			else
			{
				sscanf(reinterpret_cast<const char*>(radio_rxBuffer), "%c%c%c%03d", &pirate_from, &pirate_to, &pirate_next, &pirate_percentage);
				/*pirate_from = radio_rxBuffer[0];
				pirate_to = radio_rxBuffer[1];
				pirate_next = radio_rxBuffer[2];
				pirate_percentage = (uint8_t)(radio_rxBuffer[3] - '0' + 1) * 100;
				pirate_percentage += (uint8_t)(radio_rxBuffer[4] - '0' + 1) * 10;
				pirate_percentage += (uint8_t)(radio_rxBuffer[5] - '0' + 1);*/

			}
		}

		character_pointer = 0u;
	}
	HAL_UART_Receive_IT(&huart4, &radio_rxBuffer[character_pointer++], 1);

}
