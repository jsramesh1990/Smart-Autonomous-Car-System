/**
 * smart_autonomous_car/src/utils/timer.c
 * 
 * Timer implementation
 */

#include "timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Timer2 for millis() function
static volatile uint32_t timer_millis = 0;
static uint32_t timeout_target = 0;

void Timer_Init(void) {
    // Configure Timer2 for 1ms interrupts
    // CTC mode, prescaler 64
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22); // Prescaler 64
    
    // Compare value for 1ms interrupt
    // 16MHz / 64 = 250kHz
    // 250kHz / 250 = 1kHz (1ms)
    OCR2A = 249;
    
    // Enable compare interrupt
    TIMSK2 = (1 << OCIE2A);
    
    // Enable global interrupts
    sei();
}

uint32_t millis(void) {
    uint32_t m;
    
    // Disable interrupts to read safely
    uint8_t oldSREG = SREG;
    cli();
    m = timer_millis;
    SREG = oldSREG;
    
    return m;
}

uint32_t micros(void) {
    // Simple microsecond counter (not as accurate as millis)
    return timer_millis * 1000;
}

void delay_ms(uint32_t ms) {
    uint32_t start = millis();
    while(millis() - start < ms) {
        // Wait
    }
}

void delay_us(uint32_t us) {
    // Simple delay loop
    // Approximately 4 cycles per iteration
    // 16MHz -> 4MHz effective -> 0.25us per iteration
    uint32_t iterations = us * 4;
    for(uint32_t i = 0; i < iterations; i++) {
        __asm__ __volatile__ ("nop");
    }
}

void Timer_SetTimeout(uint32_t duration_ms) {
    timeout_target = millis() + duration_ms;
}

bool Timer_IsTimeout(void) {
    return millis() >= timeout_target;
}

void Timer_Reset(void) {
    timeout_target = 0;
}

// Timer2 compare interrupt handler
ISR(TIMER2_COMPA_vect) {
    timer_millis++;
}
