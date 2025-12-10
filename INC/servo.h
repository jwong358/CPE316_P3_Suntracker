/*
 * servo.h
 *
 *  Created on: Dec 8, 2025
 *      Author: jaylanwong
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#include "stm32l4xx_hal.h"
#include <stdint.h>

#define SERVO_MIN_US 500 									// 1.0ms (0°)
#define SERVO_MAX_US 2500									// 2.0ms (180°)
#define SERVO_MID_US   ((SERVO_MIN_US + SERVO_MAX_US) / 2)		// 1.5 ms (90°)
#define SERVO_SLOW   800000
#define SERVO_FAST   100000

void SERVO_init(uint32_t timer_clk_hz);
void SERVO_set_pulse_us(uint16_t pulse_us);
void SERVO_set_angle(float deg);


#endif /* SRC_SERVO_H_ */
