
/*
 * pin.c
 *
 *  Created on: Dec 9, 2025
 *      Author: jaylanwong
 */

#include "pin.h"
#include "servo.h"

void PIN_init(void){

    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* GPIO PA4 */
    GPIOA->MODER &= ~(0x3 << (4 * 2));  // input

    GPIOA->PUPDR &= ~(0x3 << (4 * 2));
    GPIOA->PUPDR |=  (0x1 << (4 * 2));  // pull-up

    /* EXTI */
    SYSCFG->EXTICR[1] &= ~(0xF << 0);  // clear EXTI4

    // Configure EXTI4 to trigger on falling edge (button press to GND)
    EXTI->IMR1  |=  (1 << 4);   // Unmask line 0
    EXTI->FTSR1 |=  (1 << 4);   // Falling edge
    EXTI->RTSR1 &= ~(1 << 4);   // No rising edge (optional)

    // 4) Enable EXTI0 IRQ in NVIC
    NVIC_EnableIRQ(EXTI4_IRQn);
}

void EXTI4_IRQHandler(void) {
    // Check and clear pending bit
    if (EXTI->PR1 & (1 << 4)) {
        EXTI->PR1 = (1 << 4);   // write 1 to clear

        // dance
        Servo_SetAngleDeg(45.0);
        for (volatile int i = 0; i < 8000000; i++) { __NOP(); }
        Servo_SetAngleDeg(135.0);
        for (volatile int i = 0; i < 8000000; i++) { __NOP(); }
        Servo_SetAngleDeg(45.0);
        for (volatile int i = 0; i < 8000000; i++) { __NOP(); }
        Servo_SetAngleDeg(135.0);
        for (volatile int i = 0; i < 8000000; i++) { __NOP(); }
        Servo_SetAngleDeg(90.0);
    }
}
