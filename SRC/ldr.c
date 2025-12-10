/*
 * uart.c
 *
 *  Created on: Dec 8, 2025
 *      Author: SALUT
 */

#include "idr.h"

static uint32_t ldr_sample_time_bits = LDR_SMPR_47C5;

/* Internal helper: do a single conversion on given ADC channel number */
static uint16_t LDR_ADC_ReadChannel(uint8_t channel)
{
    // 1) Select channel in SQ1
    ADC1->SQR1 &= ~ADC_SQR1_SQ1_Msk;
    ADC1->SQR1 |= ((uint32_t)channel << ADC_SQR1_SQ1_Pos);

    // 2) Clear EOC/EOS flags
    ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_EOS;

    // 3) Start conversion
    ADC1->CR |= ADC_CR_ADSTART;

    // 4) Wait until EOC set, with timeout to avoid hard lock
    uint32_t timeout = 1000000U;
    while (!(ADC1->ISR & ADC_ISR_EOC)) {
        if (--timeout == 0U) {
            return 0xFFFF;    // special error value
        }
    }

    // 5) Read data (reading DR clears EOC)
    return (uint16_t)ADC1->DR;
}

void LDR_ADC_Init(void)
{
    /* --- GPIO setup: PA0, PA1 analog --- */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;

    // PA0, PA1 -> analog mode (11)
    GPIOA->MODER |= (3U << (0 * 2)) | (3U << (1 * 2));
    GPIOA->PUPDR &= ~((3U << (0 * 2)) | (3U << (1 * 2)));

    // Connect pins to ADC via analog switches (L4-specific!)
    GPIOA->ASCR |= GPIO_ASCR_EN_0 | GPIO_ASCR_EN_1;

    /* --- ADC clock enable --- */
    RCC->AHB2ENR |= RCC_AHB2ENR_ADCEN;

    // Select ADC kernel clock source: SYSCLK (async mode)
    RCC->CCIPR &= ~RCC_CCIPR_ADCSEL;                 // clear ADCSEL[1:0]
    RCC->CCIPR |=  (3U << RCC_CCIPR_ADCSEL_Pos);     // 11: SYSCLK as ADC clock

    /* Make sure ADC is disabled (required for calibration) */
    if (ADC1->CR & ADC_CR_ADEN)
    {
        ADC1->CR |= ADC_CR_ADDIS;
        while (ADC1->CR & ADC_CR_ADEN) { }
    }

    /* Exit deep power-down, enable regulator */
    ADC1->CR &= ~ADC_CR_DEEPPWD;
    ADC1->CR |=  ADC_CR_ADVREGEN;

    // Wait for regulator to stabilize (> 20us, be generous)
    for (volatile int i = 0; i < 8000; i++) __NOP();

    /* --- Calibration (single-ended) --- */
    ADC1->CR &= ~ADC_CR_ADCALDIF;    // ensure single-ended mode
    ADC1->CR |=  ADC_CR_ADCAL;

    uint32_t timeout = 1000000U;
    while ((ADC1->CR & ADC_CR_ADCAL) && --timeout) {
    }
    if (timeout == 0) {
        // calibration failed; you could handle this if you want
        return;
    }

    /* --- ADC common clock config: leave CKMODE = 0 (async) --- */
    ADC123_COMMON->CCR &= ~(ADC_CCR_PRESC | ADC_CCR_CKMODE);
    // optional: set prescaler in PRESC if you want to slow the ADC clock

    /* --- Enable ADC1 --- */
    ADC1->ISR |= ADC_ISR_ADRDY;  // clear ADRDY
    ADC1->CR  |= ADC_CR_ADEN;    // enable ADC

    timeout = 1000000U;
    while (!(ADC1->ISR & ADC_ISR_ADRDY) && --timeout) {
    }
    if (timeout == 0) {
        // ADC never became ready; handle if desired
        return;
    }

    /* --- Configure ADC1: 12-bit, right align, single conv, SW trigger --- */
    ADC1->CFGR &= ~ADC_CFGR_RES;
    ADC1->CFGR &= ~ADC_CFGR_ALIGN;
    ADC1->CFGR &= ~ADC_CFGR_CONT;
    ADC1->CFGR &= ~ADC_CFGR_EXTEN;
    ADC1->SQR1 &= ~ADC_SQR1_L;  // one conversion in sequence

    // Set sample times for channels 5 and 6
    LDR_ADC_SetSampleTime(LDR_SMPR_47C5);
}

void LDR_ADC_SetSampleTime(uint32_t sampleTimeBits)
{
    ldr_sample_time_bits = sampleTimeBits & 0x7U;

    // Channel 5 (PA0)
    ADC1->SMPR1 &= ~(7U << ADC_SMPR1_SMP5_Pos);
    ADC1->SMPR1 |=  (ldr_sample_time_bits << ADC_SMPR1_SMP5_Pos);

    // Channel 6 (PA1)
    ADC1->SMPR1 &= ~(7U << ADC_SMPR1_SMP6_Pos);
    ADC1->SMPR1 |=  (ldr_sample_time_bits << ADC_SMPR1_SMP6_Pos);
}

uint16_t LDR_ReadLeft(void)
{
    return LDR_ADC_ReadChannel(5U);  // ADC1_IN5 = PA0
}

uint16_t LDR_ReadRight(void)
{
    return LDR_ADC_ReadChannel(6U);  // ADC1_IN6 = PA1
}
