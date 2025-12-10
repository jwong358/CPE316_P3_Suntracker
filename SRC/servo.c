#include "servo.h"

/*
 * We configure TIM1 to tick at 1 MHz (1 us per tick):
 *   TIMCLK = 80 MHz  -> PSC = 80 - 1 = 79  -> 80 MHz / 80 = 1 MHz
 *
 * For 20 ms period:
 *   ARR = 20000 - 1 = 19999  (counts 0..19999 = 20000 us)
 *
 * Pulse width in microseconds maps directly to CCR1 value.
 */

void Servo_Init(void)
{
    /* 1. Enable clocks: GPIOA + TIM1 (APB2) */
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB2ENR  |= RCC_APB2ENR_TIM1EN;

    /* 2. Configure PA8 as AF1 (TIM1_CH1) */
    // PA8: alternate function mode
    GPIOA->MODER &= ~(0x3 << (2 * 8));   // clear MODER8
    GPIOA->MODER |=  (0x2 << (2 * 8));   // AF mode

    // Select AF1 for PA8 (AFR[1], index 0)
    GPIOA->AFR[1] &= ~(0xF << (4 * 0)); // clear AFRH0
    GPIOA->AFR[1] |=  (0x1 << (4 * 0)); // AF1: TIM1_CH1

    // Optional: high speed, no pull
    GPIOA->OSPEEDR |= (0x3 << (2 * 8));   // very high speed
    GPIOA->PUPDR   &= ~(0x3 << (2 * 8));  // no pull-up/pull-down

    /* 3. Configure TIM1 for 50 Hz PWM on CH1 */

    // Disable timer during configuration
    TIM1->CR1 &= ~TIM_CR1_CEN;

    // Prescaler: 80 MHz / 80 = 1 MHz (1 us per tick)
    TIM1->PSC = 79;

    // Auto-reload: 20000 us period (20 ms) => ARR = 20000 - 1
    TIM1->ARR = SERVO_PERIOD_US - 1;

    // Set default pulse width to middle (1.5 ms)
    TIM1->CCR1 = SERVO_MID_US;

    // PWM mode 1 on CH1, enable preload
    // OC1M = 110 (PWM1), OC1PE = 1
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_CC1S);
    TIM1->CCMR1 |=  (0x6 << TIM_CCMR1_OC1M_Pos);  // PWM mode 1
    TIM1->CCMR1 |=  TIM_CCMR1_OC1PE;             // preload enable

    // Enable capture/compare output on CH1, active high
    TIM1->CCER &= ~TIM_CCER_CC1P;  // active high
    TIM1->CCER |=  TIM_CCER_CC1E;  // enable output

    // Enable auto-reload preload
    TIM1->CR1 |= TIM_CR1_ARPE;

    // Because TIM1 is an advanced timer, need to enable MOE in BDTR
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Generate an update event to load registers
    TIM1->EGR |= TIM_EGR_UG;

    // Finally, enable TIM1 counter
    TIM1->CR1 |= TIM_CR1_CEN;
}

/**
 * @brief Set servo pulse in microseconds.
 *        Values outside [SERVO_MIN_US, SERVO_MAX_US] are clamped.
 */
void Servo_SetPulseUs(uint16_t us)
{
    if (us < SERVO_MIN_US)
        us = SERVO_MIN_US;
    if (us > SERVO_MAX_US)
        us = SERVO_MAX_US;

    TIM1->CCR1 = us;
}

/**
 * @brief Set servo angle in degrees (0-180).
 *        0°  -> 1000 us
 *        90° -> 1500 us
 *        180°-> 2000 us
 */
void Servo_SetAngleDeg(float deg)
{
    if (deg < 0.0)
        deg = 0.0;
    if (deg > 180.0)
        deg = 180.0;

    float us = SERVO_MIN_US +
               (deg / 180.0) * (float)(SERVO_MAX_US - SERVO_MIN_US);

    Servo_SetPulseUs((uint16_t)(us + 0.5));
}
