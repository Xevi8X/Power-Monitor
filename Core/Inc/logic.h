/*
 * logic.h
 *
 *  Created on: 19 paź 2022
 *      Author: Wojtek
 */

#ifndef INC_LOGIC_H_
#define INC_LOGIC_H_

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

typedef struct setting
{
    uint8_t switches;
    float aggregatedPower;
}
Setting;


uint8_t getState();
void setState(uint8_t val);
void searchCompensators();
void sendCompensatorsData();

void sortConfigs();
void splitElements();
void allocSettings(int capacitives[], int inductives[]);
void sortSettings();
int compareSetting(const void* a, const void* b);
float calcAggregated(uint8_t sw, int mixTable[]);
uint8_t calcDirectIndex(uint8_t sw, int mixTable[]);
void freeSettings();
void printConfigs();
float actualCompensatedPower();
void compensate();
uint8_t chooseSetting(float Q, Setting* tab, int length);


#endif /* INC_LOGIC_H_ */
