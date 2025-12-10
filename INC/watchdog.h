/*
 * watchdog.h
 *
 *  Created on: Dec 9, 2025
 *      Author: jaylanwong
 */

#ifndef INC_WATCHDOG_H_
#define INC_WATCHDOG_H_

#include "stm32l4xx_hal.h"

void WATCHDOG_init(void);
static inline void WATCHDOG_refresh(void);


#endif /* INC_WATCHDOG_H_ */
