/*
 * servo.c
 *
 *  Created on: Dec 8, 2025
 *      Author: jaylanwong
 */

#include "servo.h"

void SERVO_init(uint32_t timer_clk_hz){
	/* enable clocks */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 	// GPIOA
	RCC->APB1ENR |= RCC_APB1ENR_TIM1EN;		// TIM1

	/* GPIO PA8 */
	GPIOA->MODER &= ~(0x3 << (8 * 2));      // clear
	GPIOA->MODER |=  (0x2 << (8 * 2));      // AF mode

	GPIOA->OTYPER &= ~(1 << 8); 			// push-pull
	GPIOA->OSPEEDR |= (0x3 << (8 * 2));		// high speed
	GPIOA->PUPDR   &= ~(0x3 << (8 * 2));    // no pull

	GPIOA->AFR[1] &= ~(0xF << (4 * 0));
	GPIOA->AFR[1] |=  (0x1 << (4 * 0));     // AF1 = TIM1

	/* TIM1 init */
	TIM1->CR1 = 0;
	TIM1->PSC = (timer_clk_hz/1000000) - 1;							// 1MHz clock
	TIM1->ARR = 20000 - 1;											// 20ms period

	// PWM setup
	TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE);
	TIM1->CCMR1 |=  (6 << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE; 	//OC1M = 110 (PWM mode 1), OC1PE = 1 (preload)

	// Enable capture/compare 1 output
	TIM1->CCER &= ~TIM_CCER_CC1P;  									// active high
	TIM1->CCER |=  TIM_CCER_CC1E;									// enable output

	// Start with servo in middle
	TIM1->CCR1 = SERVO_MID_US;   											// 90°

	// Auto-reload preload enable
	TIM1->CR1 |= TIM_CR1_ARPE;

	// Enable counter
	TIM1->CR1 |= TIM_CR1_CEN;

}

void SERVO_set_pulse_us(uint16_t pulse_us) {
    // safety clamp between 0-180°
    if (pulse_us < SERVO_MIN_US) pulse_us = SERVO_MIN_US;
    if (pulse_us > SERVO_MAX_US) pulse_us = SERVO_MAX_US;
    TIM1->CCR1 = pulse_us;   // because 1 tick = 1 µs in our setup
}

void SERVO_set_angle(float deg) {
    if (deg < 0.0) deg = 0.0;
    if (deg > 180.0) deg = 180.0;

    // Map 0–180°
    float pulse = SERVO_MIN_US + (deg / 180.0) * (float)(SERVO_MAX_US - SERVO_MIN_US);
    SERVO_set_pulse_us((uint16_t)pulse + 0.5);
}
