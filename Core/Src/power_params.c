/*
 * power_params.c
 *
 *  Created on: Aug 14, 2022
 *      Author: Wojtek
 */

#include "power_params.h"
#include <math.h>

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
		indexCircBuffer++;
		if(indexCircBuffer == BUFFERSIZE)
		{
			indexCircBuffer = 0;
			//calibCounter++;
			//if(calibCounter == CALIBRATIONPERIOD)
			//{
			//	calibCounter = 0;
			//	CalibrateZero();
			//}
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
		for(uint8_t i = 0; i < CHANNELS;i++)
		{
			P[i] -= data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][2*i]*data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][2*i+1];
		}
		for(uint8_t i = 0; i < CHANNELS*2;i++)
		{
			RMS[i] -= data[(indexCircBuffer+correctionRMS)% BUFFERSIZE][i]*data[(indexCircBuffer+ correctionRMS)% BUFFERSIZE][i];
			setSign(i,indexCircBuffer, (data[indexCircBuffer][i]> 0) ? 1: 0);
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


	float angle = count;
	angle /= (BUFFERSIZE-correctionRMS);
	//angle = 1 - angle;
	angle *= 180.0f;
	disableSetting = 0;
	return angle;
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
	return P[channel] /(BUFFERSIZE - correctionRMS)/ (VOLTAGESCALE *CURRENTSCALE * OVERSAMPLING * OVERSAMPLING);
}

float getS(uint8_t channel)
{
	return getV(channel) * getI(channel);
}

float getQ(uint8_t channel)
{
	float P = getP(channel);
	float S = getS(channel);
	return sqrt(S*S-P*P);
}

void getParams(Params* p, uint8_t channel)
{

	p->V = getV(channel);
	p->I = getI(channel);
	p->P = getP(channel);
	p->S = p->V*p->I;
	p->Q = sqrt(p->S*p->S-p->P*p->P);
	p->fi = calcXOR(channel);
	return;
}



