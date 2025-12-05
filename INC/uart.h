/*
 * uart.h
 *
 *  Created on: Dec 4, 2025
 *      Author: SALUT
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32l476xx.h"
#include <stdint.h>

void UART2_Init(uint32_t sysclk_hz);
void UART2_WriteChar(char c);
void UART2_WriteString(const char *s);
void UART2_WriteESC(const char *seq);



#endif /* INC_UART_H_ */
