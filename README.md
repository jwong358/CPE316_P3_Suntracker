🌞 Sun Tracking Solar Panel (Tracking Mechanism Only)

CPE 316 – Microcontrollers
Cal Poly SLO

📌 Overview

This project implements a sun-tracking control system using an STM32L476 microcontroller.
Rather than generating power, the project focuses on the tracking mechanism itself — sensing light intensity and mechanically orienting a platform toward the brightest light source.

Two light-dependent resistors (LDRs) are read using the ADC at the register level, and a servo motor is driven using PWM to rotate the platform. A UART-based command-line interface allows real-time control, monitoring, and testing. Safety and robustness are handled using a watchdog timer and optional limit-switch interrupts.

🧠 Features

Dual LDR light sensing (left/right comparison)

Register-level ADC configuration (no HAL ADC)

PWM servo control using TIM1

Fast and slow tracking modes

On-demand telemetry via UART (no constant spamming)

Servo centering and sweep test (0–180° in 15° steps)

Watchdog reset if servo becomes stuck at limits

Modular software design (UART, ADC, Servo, Watchdog, Pins)

🧩 Hardware Requirements

STM32 NUCLEO-L476RG

2× LDRs (photoresistors)

2× 10 kΩ resistors

1× Servo motor (e.g., SG90 or equivalent)

External 5 V supply for servo

Breadboard and jumper wires

USB cable for programming and UART communication

🔌 Wiring Summary
LDR Voltage Dividers
3.3V ── LDR ──┬── PA0 (Left) / PA1 (Right)
              └─ 10kΩ ── GND

Servo
Servo Wire	Connection
Orange (Signal)	PA8 (TIM1_CH1)
Red (V+)	External 5 V
Brown (GND)	Common GND

⚠️ Important: The servo ground must be connected to the Nucleo ground.

🖥️ UART Commands

Connect to the board using a serial terminal:

Baud: 115200

Data: 8

Parity: None

Stop bits: 1

Flow control: None

Supported Commands
Command	Description
h	Show help menu
f	Fast tracking mode
s	Slow tracking mode
c	Center servo
p	Print one status line
t	Servo sweep test (0–180° in 15° steps)
🧠 Software Structure
/Core
 ├── Src
 │   ├── main.c        # Main tracking loop
 │   ├── uart.c        # Register-level UART + CLI
 │   ├── servo.c       # TIM1 PWM servo control
 │   ├── ldr.c         # Register-level ADC for LDRs
 │   ├── watchdog.c   # Independent watchdog logic
 │   └── pin.c         # GPIO / interrupt handling
 └── Inc
     ├── uart.h
     ├── servo.h
     ├── ldr.h
     ├── watchdog.h
     └── pin.h


All peripherals (UART, ADC, PWM, Watchdog) are configured at the register level to demonstrate low-level embedded control.

🔁 Tracking Algorithm

Read left and right LDR values using ADC.

Compute error: error = left − right.

Apply deadband to prevent jitter.

Adjust servo position incrementally using PWM.

Refresh watchdog unless servo is at min/max limit.

Repeat continuously.
