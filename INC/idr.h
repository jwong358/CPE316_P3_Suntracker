#ifndef LDR_H
#define LDR_H

#include "stm32l476xx.h"
#include <stdint.h>

/*
 * LDR module (register-level, no HAL)
 *
 * - Left  LDR -> PA0 -> ADC1_IN5
 * - Right LDR -> PA1 -> ADC1_IN6
 *
 * You must call LDR_ADC_Init() once before using LDR_ReadLeft/Right().
 */

/* Sample time options (values written into SMPx[2:0] bits) */
#define LDR_SMPR_2C5     (0U)  /* 2.5 cycles  */
#define LDR_SMPR_6C5     (1U)  /* 6.5 cycles  */
#define LDR_SMPR_12C5    (2U)  /* 12.5 cycles */
#define LDR_SMPR_24C5    (3U)  /* 24.5 cycles */
#define LDR_SMPR_47C5    (4U)  /* 47.5 cycles */
#define LDR_SMPR_92C5    (5U)  /* 92.5 cycles */
#define LDR_SMPR_247C5   (6U)  /* 247.5 cycles */
#define LDR_SMPR_640C5   (7U)  /* 640.5 cycles */

/* Initialize GPIO + ADC1 (12-bit, single conversion, SW trigger) */
void LDR_ADC_Init(void);

/* Optionally change ADC sample time for both LDR channels */
void LDR_ADC_SetSampleTime(uint32_t sampleTimeBits);

/* Blocking single conversion reads (polling EOC) */
uint16_t LDR_ReadLeft(void);   /* PA0 / ADC1_IN5 */
uint16_t LDR_ReadRight(void);  /* PA1 / ADC1_IN6 */

#endif /* LDR_H */
