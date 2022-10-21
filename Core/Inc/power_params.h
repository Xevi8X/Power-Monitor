/*
 * power_params.h
 *
 *  Created on: Aug 14, 2022
 *      Author: Wojtek
 */

#ifndef INC_POWER_PARAMS_H_
#define INC_POWER_PARAMS_H_

#include "stm32f1xx_hal.h"
#include "smart_common.h"
#include <stdio.h>


#define CHANNELS 3
#define OVERSAMPLING 8
#define BUFFERSIZE 512
#define EXPECTEDFREQ 50
#define CALIBRATIONPERIOD 4096
#define CURRENTSCALE 51.47
#define VOLTAGESCALE 5.19
#define SHOWDATAPERIOD 1000

typedef struct
{
	float V,I,P,S,Q,fi;
} Params;


void powerParamInit();
void CalcRMScorection();
void CalibrateZero();
void takeData(uint32_t* buffer);
void setSign(uint8_t channel, uint16_t index,uint8_t value);
float calcXOR(uint8_t channel);
uint8_t isCapacitive(uint8_t channel);
void printBufforData();
void turnOffInZero(uint8_t pin);
void execCommand(uint8_t commandNo, uint8_t arg);

uint32_t* getADC_Buffer();
uint32_t* gethalfOfADC_Buffer();

float getV(uint8_t channel);
float getI(uint8_t channel);
float getP(uint8_t channel);
float getS(uint8_t channel);
float getQ(uint8_t channel);
void getParams(Params* p, uint8_t channel);

#endif /* INC_POWER_PARAMS_H_ */
