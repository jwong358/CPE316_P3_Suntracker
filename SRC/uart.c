/*
 * uart.c
 *
 *  Created on: Dec 4, 2025
 *      Author: SALUT
 */

#include "uart.h"
#include "stm32l476xx.h"
#include "servo.h"



extern int servo_us;
extern int speed;
extern int deadband;
extern volatile uint8_t g_print_status;


#define SERVO_US_MIN           500
#define SERVO_US_MAX          2500
#define SERVO_US_MID  ((SERVO_US_MIN + SERVO_US_MAX) / 2)
#define SERVO_SLOW       800000
#define SERVO_FAST       100000
#define TRACK_DEADBAND_SLOW   100
#define TRACK_DEADBAND_FAST    40


void Servo_SetPulseUs(uint16_t us);


void UART2_Init(uint32_t sysclk_hz)
{
    RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;

    // PA2, PA3: alternate function mode
    GPIOA->MODER &= ~((0x3 << (2*2)) | (0x3 << (3*2)));
    GPIOA->MODER |=  ((0x2 << (2*2)) | (0x2 << (3*2)));

    // Set AF7 on PA2, PA3 (AFRL bits)
    GPIOA->AFR[0] &= ~((0xF << (2*4)) | (0xF << (3*4)));
    GPIOA->AFR[0] |=  ((0x7  << (2*4)) | (0x7  << (3*4)));

    // high speed
    GPIOA->OSPEEDR |= ((0x3 << (2*2)) | (0x3 << (3*2)));

    // No pull-up/down
    GPIOA->PUPDR &= ~((0x3 << (2*2)) | (0x3 << (3*2)));

    /* Disable USART2 before config */
    USART2->CR1 &= ~USART_CR1_UE;

    /* Set baud rate 115200*/
    // BRR = fck / baud; for 80 MHz & 115200
    uint32_t brr = (sysclk_hz + (115200/2)) / 115200;
    USART2->BRR = brr;

    /* 8N1, oversampling by 16 is default: M[1:0]=00, PCE=0, STOP=00 */
    USART2->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE);
    USART2->CR2 &= ~(USART_CR2_STOP);

    /* Enable TX, RX, and RX interrupt */
    USART2->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;

    /* Enable USART2 */
    USART2->CR1 |= USART_CR1_UE;

    /* Enable NVIC interrupt */
    NVIC_EnableIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 2);
}

void UART2_WriteChar(char c)
{
    while (!(USART2->ISR & USART_ISR_TXE)) {
    }
    USART2->TDR = (uint8_t)c;
}

void UART2_WriteString(const char *s)
{
    while (*s) {
        UART2_WriteChar(*s++);
    }
}

/* For VT100 send ESC + sequence */
void UART2_WriteESC(const char *seq)
{
    UART2_WriteChar(0x1B);
    UART2_WriteString(seq);
}


void UART2_OnRxChar(char c)
{
    switch (c)
    {
    case 'h':
    case 'H':
        UART2_WriteString("\r\nCommands:\r\n");
        UART2_WriteString("  h - help\r\n");
        UART2_WriteString("  f - fast tracking\r\n");
        UART2_WriteString("  s - slow tracking\r\n");
        UART2_WriteString("  c - center servo\r\n");
        UART2_WriteString("  p - print one status line\r\n");
        break;

    case 'f':   // FAST mode
    case 'F':
        speed    = SERVO_FAST;
        deadband = TRACK_DEADBAND_FAST;
        UART2_WriteString("\r\nMode: FAST (small deadband, quick moves)\r\n");
        break;

    case 's':   // SLOW mode
    case 'S':
        speed    = SERVO_SLOW;
        deadband = TRACK_DEADBAND_SLOW;
        UART2_WriteString("\r\nMode: SLOW (large deadband, slower moves)\r\n");
        break;

    case 'c':   // center servo
    case 'C':
        servo_us = SERVO_US_MID;
        Servo_SetPulseUs((uint16_t)servo_us);
        UART2_WriteString("\r\nServo centered.\r\n");
        break;

    case 'p':   // print one status line
    case 'P':
        g_print_status = 1;
        break;

    default:
        // optional: echo or ignore unknown commands
        // UART2_WriteChar(c);
        break;
    }
}


/* ISR called on RX */
void USART2_IRQHandler(void)
{
    if (USART2->ISR & USART_ISR_RXNE) {
        char c = (char)USART2->RDR;  // reading RDR clears RXNE
        UART2_OnRxChar(c);
    }
}

void UART2_WriteNumber(uint32_t n)
{
    char buf[10];      // enough for 32-bit unsigned
    int i = 0;

    // Special case for zero:
    if (n == 0) {
        UART2_WriteChar('0');
        return;
    }

    // Extract digits (in reverse order)
    while (n > 0) {
        uint32_t digit = n % 10;
        buf[i++] = '0' + digit;
        n /= 10;
    }

    // Output digits in correct order
    while (i > 0) {
        UART2_WriteChar(buf[--i]);
    }
}


