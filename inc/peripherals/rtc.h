/*
 * rtc.h
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#ifndef RTC_H
#define RTC_H

#include "mode.h"

/*** RTC macros ***/

// RTC wake-up timer period.
// Warning: this value must be lower than the watchdog period = 25s.
 #define RTC_WAKEUP_PERIOD_SECONDS	5

/*** RTC functions ***/

void RTC_Reset(void);
void RTC_Init(void);
void RTC_StartWakeUpTimer(unsigned int delay_seconds);
void RTC_StopWakeUpTimer(void);
volatile unsigned char RTC_GetWakeUpTimerFlag(void);
void RTC_ClearWakeUpTimerFlag(void);

#endif /* RTC_H */
