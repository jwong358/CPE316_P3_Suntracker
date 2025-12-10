/*
 * pin.h
 *
 *  Created on: Dec 9, 2025
 *      Author: jaylanwong
 */

#ifndef INC_PIN_H_
#define INC_PIN_H_

#include "stm32l4xx_hal.h"

void PIN_init(void);
void EXTI4_IRQHandler(void);

#endif /* INC_PIN_H_ */
