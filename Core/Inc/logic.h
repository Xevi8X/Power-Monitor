/*
 * logic.h
 *
 *  Created on: 19 paź 2022
 *      Author: Wojtek
 */

#ifndef INC_LOGIC_H_
#define INC_LOGIC_H_

uint8_t getState();
void setState(uint8_t val);
void searchCompensators();
void sendCompensatorsData();

#endif /* INC_LOGIC_H_ */
