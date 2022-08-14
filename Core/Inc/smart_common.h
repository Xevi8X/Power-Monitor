/*
 * smart_common.h
 *
 *  Created on: Aug 14, 2022
 *      Author: Wojciech Gajda
 */

#ifndef INC_SMART_COMMON_H_
#define INC_SMART_COMMON_H_

#include "stm32f1xx_hal.h"

int __io_putchar(int ch);

static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}

uint32_t getCurrentMicros(void);

#endif /* INC_SMART_COMMON_H_ */
