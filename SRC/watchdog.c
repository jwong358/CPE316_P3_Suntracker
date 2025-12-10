/*
 * watchdog.c
 *
 *  Created on: Dec 9, 2025
 *      Author: jaylanwong
 */
#include "watchdog.h"

void WATCHDOG_init(void){

    IWDG->KR = 0x5555;		// Enable write access to IWDG_PR and IWDG_RLR
    IWDG->PR = 0x4;  		// /64
    IWDG->RLR = 1999;		// ~4s timeout = (RLR + 1) * PR / 32KHz
    IWDG->KR = 0xAAAA;		// Reload the counter
    IWDG->KR = 0xCCCC;		// start watchdog timer
}

void WATCHDOG_refresh(void){
	IWDG->KR = 0xAAAA; 						// Reload
}
