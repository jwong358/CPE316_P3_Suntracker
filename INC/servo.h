#ifndef SERVO_H
#define SERVO_H

#include "stm32l476xx.h"
#include <stdint.h>

#define SERVO_MIN_US   500   // 1.0 ms pulse
#define SERVO_MAX_US   2500   // 2.0 ms pulse
#define SERVO_MID_US   ((SERVO_MIN_US + SERVO_MAX_US) / 2)   // 1.5 ms (90°)
#define SERVO_PERIOD_US 20000 // 20 ms period (50 Hz)

/**
 * Initialize TIM1 CH1 on PA8 for servo control (50 Hz PWM).
 *        Assumes SystemCoreClock / APB2 timer clock = 80 MHz.
 */
void Servo_Init(void);

/**
 * Set servo pulse width in microseconds (typically 1000–2000 us).
 */
void Servo_SetPulseUs(uint16_t us);

/**
 * Set servo angle in degrees (0–180) mapped to 1000–2000 us.
 */
void Servo_SetAngleDeg(float deg);

#endif // SERVO_H
