/*
 * logic.c
 *
 *  Created on: 19 paź 2022
 *      Author: Wojtek
 */
#include "power_params.h"
#include "i2c.h"
#include <math.h>
#include <stdlib.h>
#include "logic.h"
#include <float.h>

float compensators[8]={0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

//0 - auto, 1 - manual
uint8_t state = 1;


int cap_length;
int ind_length;
Setting* Cap;
Setting* Ind;
float precision;

uint8_t getState()
{
	return state;
}

void setState(uint8_t val)
{
	if(val == 0 || val == 1)
		state = val;
}

void searchCompensators()
{
	precision = FLT_MAX;
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
			if(fabs(compensators[i]) < precision) precision = fabs(compensators[i]);
		}
		else
		{
			compensators[i] = 0.0f;
			printf("unconnected\n");
		}
		PCF8574_turnOff(i);
		HAL_Delay(300);
	}
	sendCompensatorsData();
	sortConfigs();
	printConfigs();
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

void sortConfigs()
{
	freeSettings();
	splitElements();
}

void splitElements()
{
	cap_length = 0;
	ind_length = 0;
	int capacitives[8];
	int inductives[8];
	for (int i = 0; i < 8; i++)
	{
		capacitives[i] = -1;
		inductives[i] = -1;
		if (compensators[i] > 0.0f)
		{
			inductives[ind_length++] = i;
		}
		if (compensators[i] < 0.0f)
		{
			capacitives[cap_length++] = i;
		}
	}

	ind_length = 1 << ind_length;
	cap_length = 1 << cap_length;
	allocSettings(capacitives, inductives);
	sortSettings();
}

void allocSettings(int capacitives[], int inductives[])
{
	Cap = (Setting*)calloc(cap_length, sizeof(Setting));
	Ind = (Setting*)calloc(ind_length, sizeof(Setting));

	for (int i = 0; i < cap_length; i++)
	{
		Cap[i].aggregatedPower = calcAggregated(i, capacitives);
		Cap[i].switches = calcDirectIndex(i, capacitives);

	}

	for (int i = 0; i < ind_length; i++)
	{
		Ind[i].aggregatedPower = calcAggregated(i, inductives);
		Ind[i].switches = calcDirectIndex(i, inductives);
	}

}

void sortSettings()
{
	qsort(Cap, cap_length, sizeof(Setting), compareSetting);
	qsort(Ind, ind_length, sizeof(Setting), compareSetting);
}

int compareSetting(const void* a, const void* b) {
	Setting* x = (Setting*)a;
	Setting* y = (Setting*)b;
	return x->aggregatedPower - y->aggregatedPower;
}

float calcAggregated(uint8_t sw, int mixTable[])
{
	float sum = 0.0f;
	for (int i = 0; i < 8; i++)
	{
		if ((sw & 1) == 1)
		{
			sum += compensators[mixTable[i]];
		}
		sw >>= 1;
	}
	return sum;
}

uint8_t calcDirectIndex(uint8_t sw, int mixTable[])
{
	uint8_t direct = 0;
	for (int i = 0; i < 8; i++)
	{
		if ((sw & 1) == 1)
		{
			direct |= (1 << mixTable[i]);
		}
		sw >>= 1;
	}
	return direct;
}

void freeSettings()
{
	free(Cap);
	free(Ind);
}

void printConfigs()
{
	printf("\nAvailable configs:\nCapacitors:\n");
	for(uint8_t i = 0; i < cap_length; i++) printf(BYTE_TO_BINARY_PATTERN":  %.2f\n", BYTE_TO_BINARY(Cap[i].switches), Cap[i].aggregatedPower);
	printf("Inductors:\n");
	for(uint8_t i = 0; i < ind_length; i++) printf(BYTE_TO_BINARY_PATTERN":  %.2f\n", BYTE_TO_BINARY(Ind[i].switches), Ind[i].aggregatedPower);
}

float actualCompensatedPower()
{
	uint8_t sw = PCF8574_getState();
	float sum = 0.0f;
	for (int i = 0; i < 8; i++)
	{
		if ((sw & 1) == 1)
		{
			sum += compensators[i];
		}
		sw >>= 1;
	}
	return sum;
}

void compensate()
{
	if(state == 0 && fabs(getQ(0)) > 2*precision)
	{
		float Q_summed = getQ(0);
		float Q_compensated = actualCompensatedPower();
		float Q = Q_summed - Q_compensated;

		uint8_t res;
		if(Q > 0)
		{
			res = chooseSetting(Q, Cap, cap_length);
		}
		else
		{
			res = chooseSetting(Q,Ind, ind_length);
		}
		softSwitch(res);
		execCommand(101,0);
	}
}

uint8_t chooseSetting(float Q, Setting* tab, int length)
{
	uint8_t res = 0;
	float diff = FLT_MAX;

	for(int i = 0; i < length; i++)
	{
		if(fabs(Q + tab[i].aggregatedPower) < diff)
		{
			res = tab[i].switches;
			diff = fabs(Q + tab[i].aggregatedPower);
		}
	}
	return res;
}
