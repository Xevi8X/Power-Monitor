/*
 * logic.c
 *
 *  Created on: 19 paź 2022
 *      Author: Wojtek
 */
#include "power_params.h"
#include "i2c.h"

float compensators[8]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

//0 - auto, 1 - manual
uint8_t state = 0;

uint8_t getState()
{
	return state;
}

void searchCompensators()
{
	printf("\nStarting searching compensators\n");
	for(int i = 0; i < 8; i++) PCF8574_turnOff(i);

	for(int i = 0; i < 8; i++)
	{
		printf("%d: ",i);
		PCF8574_turnOn(i);
		HAL_Delay(500);
		if(getS(0) > 20.0f)
		{
			compensators[i] = getQ(0);
			if(compensators[i] > 0) printf("inductive    ");
			else printf("capacitive   ");
			printf("Q:%.2f  P:%.2f\n", compensators[i], getP(0));
		}
		else
		{
			compensators[i] = 0.0f;
			printf("unconnected\n");
		}
		turnOffInZero(i);
		HAL_Delay(300);
	}
	sendCompensatorsData();
}

void sendCompensatorsData()
{
	printf("@6#");
	for(int i = 0; i < 8; i++)
	{
		printf("%.2f;",compensators[i]);
	}
	printf("\n");
}
