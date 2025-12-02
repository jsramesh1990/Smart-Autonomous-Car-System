/**
 * smart_autonomous_car/src/utils/timer.h
 * 
 * Timer utilities for precise timing
 */

#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

// Timer Functions
void Timer_Init(void);
uint32_t millis(void);
uint32_t micros(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
void Timer_SetTimeout(uint32_t duration_ms);
bool Timer_IsTimeout(void);
void Timer_Reset(void);

#endif // TIMER_H
