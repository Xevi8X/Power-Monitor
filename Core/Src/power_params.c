/*
 * power_params.c
 *
 *  Created on: Aug 14, 2022
 *      Author: Wojtek
 */

#include "power_params.h"
#include <math.h>
#include "i2c.h"
#include "logic.h"

uint32_t ADC_Buffer[2*CHANNELS];
uint32_t* halfOfADC_Buffer = ADC_Buffer + CHANNELS;
int16_t data[BUFFERSIZE][CHANNELS*2];
uint32_t time[BUFFERSIZE];
int32_t  Tinterval;
uint16_t indexCircBuffer;
uint8_t oversamplingIndex;
uint16_t calibZeros[CHANNELS*2] = {0};
uint64_t RMS[2*CHANNELS] = {0};
//0,1,2 - V
//3,4,5 - A
uint8_t correctionRMS;
uint16_t calibCounter;
int64_t P[CHANNELS] = {0};
uint8_t sign[2*CHANNELS][BUFFERSIZE/8] = {0};
uint8_t disableSetting;
uint8_t positive[3] = {0,0,0};
uint8_t crossingIndex[3] = {0,0,0};
uint16_t crossingPoint[3][8];
uint8_t pinToTurnOff = 0;
uint8_t pinToTurnOn = 0;

void powerParamInit()
{
	indexCircBuffer = 0;
	oversamplingIndex = 0;
	correctionRMS = 1;
	calibCounter = 0;
	disableSetting = 0;
}

void CalcRMScorection()
{
		while(indexCircBuffer!= 0);
		__disable_irq();
		uint32_t timeOfBufforing = time[BUFFERSIZE-1]- time[0];
		uint32_t halfPhase = 1000000/EXPECTEDFREQ/2;
		uint16_t halfPeriods = timeOfBufforing/halfPhase;
		Tinterval = halfPhase*halfPeriods;
		while(time[BUFFERSIZE-1-correctionRMS] > time[0] + Tinterval) correctionRMS++;
		Tinterval = time[BUFFERSIZE-1-correctionRMS] - time[0];
		__enable_irq();
}

void CalibrateZero()
{

	printf("Starting calibration...\n");
	while(indexCircBuffer!= 0);
	__disable_irq();

	//Vpp calibration
	int32_t min = 1 << 16, max = 0;
	for(uint8_t j = 0; j < CHANNELS*2;j++)
	{
		for(uint16_t i = correctionRMS; i < BUFFERSIZE;i++)
		{
			if(max < data[i][j]) max = data[i][j];
			if(min > data[i][j]) min = data[i][j];
			data[i][j] = 0;
		}
		calibZeros[j] += (min+max)/2;
		data[0][j] = -calibZeros[j];
		RMS[j] = 0;
	}
	for(uint8_t j = 0; j < CHANNELS;j++)
	{
		P[j] = 0;
	}
	printf("Calibration completed\n");
	__enable_irq();
}

void takeData(uint32_t* buffer)
{
	if(oversamplingIndex == OVERSAMPLING)
	{

		oversamplingIndex = 0;
		time[indexCircBuffer] = getCurrentMicros();
		for(uint8_t i = 0; i < CHANNELS*2;i++)
		{
			RMS[i] += data[indexCircBuffer][i]*data[indexCircBuffer][i];
		}
		for(uint8_t i = 0; i < CHANNELS;i++)
		{
			P[i] += data[indexCircBuffer][2*i]*data[indexCircBuffer][2*i+1];
		}

		for(uint8_t i = 0; i < CHANNELS*2;i++)
		{
			if(data[indexCircBuffer][i]> 0) setSign(i,indexCircBuffer, 1);
			else setSign(i,indexCircBuffer, 0);
		}


		for(uint8_t v_channel = 0; v_channel < CHANNELS; v_channel++)
		{
			if(data[indexCircBuffer][2*v_channel+1]> 0)
			{
				if(positive[v_channel] == 0)
				{
					crossingPoint[v_channel][crossingIndex[v_channel]] = data[indexCircBuffer][2*v_channel] > 0 ? 1 : 0;
					crossingIndex[v_channel] = (crossingIndex[v_channel]+1) % 8;
					if(v_channel == 0)
					{
						uint8_t pins = PCF8574_getState();
						pins &= (~pinToTurnOff);
						pins |= pinToTurnOn;
						PCF8574_setState(pins);
						pinToTurnOn = 0;
						pinToTurnOff = 0;
					}
				}
				positive[v_channel] = 1;
			}
			else
			{
				positive[v_channel] = 0;
			}
		}
		indexCircBuffer++;
		if(indexCircBuffer == BUFFERSIZE) indexCircBuffer = 0;
		for(uint8_t i = 0; i < CHANNELS;i++)
		{
			P[i] -= data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][2*i]*data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][2*i+1];
		}
		for(uint8_t i = 0; i < CHANNELS*2;i++)
		{
			RMS[i] -= data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][i]*data[(indexCircBuffer+ correctionRMS)% BUFFERSIZE][i];
			data[indexCircBuffer][i] = -calibZeros[i];
		}
	}

	for(uint8_t i = 0; i < CHANNELS;i++)
	{
		data[indexCircBuffer][2*i] += (uint16_t) buffer[i];
		data[indexCircBuffer][2*i+1] += (uint16_t) (buffer[i] >> 16);
	}
	oversamplingIndex++;
}

void setSign(uint8_t channel, uint16_t index,uint8_t value)
{
	if(disableSetting != 0) return;
	if(value == 1)
	{
		sign[channel][index/8] |= (1 << (index % 8));
	}
	else
	{
		sign[channel][index/8] &= ~(1 << (index % 8));
	}
}

float calcXOR(uint8_t channel)
{
	disableSetting = 1;
	uint16_t count = 0;
	uint16_t counter = 0;
	for(uint8_t i = 0; i < (BUFFERSIZE)/8; i++)
	{
		uint8_t xor = (sign[channel*2][i]) ^ (sign[channel*2 + 1][i]);
		while (xor > 0)
		{
			if(counter < BUFFERSIZE - correctionRMS)
			{
				count += xor & 1;
			}
			xor >>= 1;
			counter++;
		}
	}
	float angle = (float)count;
	angle *= 180.0f;
	angle /= (BUFFERSIZE-correctionRMS);


	if(isCapacitive(channel) > 0) angle *=(-1.0f);

	disableSetting = 0;
	return angle;
}

uint8_t isCapacitive(uint8_t channel)
{
	int res = 0;
		for(int i = 0; i < 8; i++)
		{
			if(crossingPoint[channel][i] > 0) res++;
			else res--;
		}
	return res > 0 ? 1 : 0;
}


void printBufforData()
{
	printf("Starting printing out data...\n");
	while(indexCircBuffer!= 0);
	__disable_irq();
	printf("t,V,I\n");
	printf("@3#\n");
	for(int i = correctionRMS; i < BUFFERSIZE; i++) printf("@4#%lu;%f;%f\n",time[i],((float)data[i][1])/(VOLTAGESCALE * OVERSAMPLING),((float)data[i][0])/(CURRENTSCALE * OVERSAMPLING));
	printf("@5#\n");
	printf("Printing completed\n");
	__enable_irq();
}

void turnOffInZero(uint8_t pin)
{
	PCF8574_check(pin);
	pinToTurnOff |= (1 << pin);
}

void turnOnInZero(uint8_t pin)
{
	PCF8574_check(pin);
	pinToTurnOn |= (1 << pin);
}

void softSwitch(uint8_t state)
{
	pinToTurnOff = ~state;
	pinToTurnOn = state;
}

void execCommand(uint8_t commandNo, uint8_t arg)
{
	switch(commandNo)
	{
		case 100:
		{
			CalibrateZero();
			break;
		}
		case 101:
		{
			printf("@2#%u;%u\n",PCF8574_getState(), getState());
			break;
		}
		case 102:
		{
			setState(arg);
			printf("@2#%u;%u\n",PCF8574_getState(), getState());
			break;
		}
		case 103:
		{
			sendCompensatorsData();
			break;
		}
		case 104:
		{
			printBufforData();
			break;
		}
		case 105:
		{
			if(getState() == 1)
			{
				PCF8574_setState(arg);
				printf("@2#%u;%u\n",PCF8574_getState(), getState());
			}
			break;
		}
		case 106:
		{
			searchCompensators();
			break;
		}

	}
}

uint32_t* getADC_Buffer()
{
	return ADC_Buffer;
}

uint32_t* gethalfOfADC_Buffer()
{
	return halfOfADC_Buffer;
}

float getV(uint8_t channel)
{
	return sqrt(((float)RMS[2*channel+1])/(BUFFERSIZE - correctionRMS)) / (VOLTAGESCALE * OVERSAMPLING);
}

float getI(uint8_t channel)
{
	return sqrt(((float)RMS[2*channel])/(BUFFERSIZE - correctionRMS)) / (CURRENTSCALE * OVERSAMPLING);
}

float getP(uint8_t channel)
{
	return P[channel] /((BUFFERSIZE - correctionRMS) * VOLTAGESCALE * CURRENTSCALE * OVERSAMPLING * OVERSAMPLING);
}

float getS(uint8_t channel)
{
	return getV(channel) * getI(channel);
}

float getQ(uint8_t channel)
{
	float P = getP(channel);
	float S = getS(channel);
	float Q = sqrt(S*S-P*P);
	if(isCapacitive(channel) > 0) Q *=(-1.0f);
	return Q;
}

void getParams(Params* p, uint8_t channel)
{

	p->V = getV(channel);
	p->I = getI(channel);
	p->P = getP(channel);
	p->S = getS(channel);
	p->Q = getQ(channel);
	p->fi = calcXOR(channel);
	return;
}



