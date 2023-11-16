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
uint8_t countdown_value = 6;
bool flood_active = false;
uint8_t flood_counter = 0;
char pirate_from, pirate_to, pirate_next;
int pirate_percentage = 0;
void Radio_Init()
{
    HAL_UART_Receive_IT (&huart4, radio_rxBuffer, 12);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t i, length;
    HAL_UART_Receive_IT(&huart4, radio_rxBuffer, 12);
    // Check to see where is the termination character to determine what type of message is received
    for(i = 0; i < 12; ++i)
    {
    	if(radio_rxBuffer[i] == '\r')
    	{
    		length = i;
    	}
    }
    // Labirinth countdown message received
    if(length == 1)
    {
    	countdown_value = radio_rxBuffer[0];
    }
    if(length == 6)
    {
    	//FLOOD message received
    	if(!strcmp("FLOOD!\r", reinterpret_cast<const char*>(radio_rxBuffer)))
    	{
    		flood_active = true;
    	}
    	else
    	{
    		sscanf(reinterpret_cast<const char*>(radio_rxBuffer), "%c%c%c%03d", &pirate_from, &pirate_to, &pirate_next, &pirate_percentage);
    	}
    }

    else
    {
    	flood_counter++;
    }

    if(flood_counter > 10)
    {
    	flood_active = false;
    }

}
