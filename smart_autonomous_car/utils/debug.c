/**
 * smart_autonomous_car/src/utils/debug.c
 * 
 * Debug implementation
 */

#include "debug.h"
#include <avr/io.h>
#include <util/delay.h>

void Debug_Init(void) {
    // Configure UART for debug output
    // 9600 baud, 8 data bits, no parity, 1 stop bit
    
    // Set baud rate
    UBRR0H = (uint8_t)(UART_BAUD >> 8);
    UBRR0L = (uint8_t)UART_BAUD;
    
    // Enable transmitter
    UCSR0B = (1 << TXEN0);
    
    // Set frame format: 8 data bits, 1 stop bit
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    
    _delay_ms(100);
    Debug_Print("Debug Initialized\r\n");
}

void Debug_Print(const char *message) {
    while(*message) {
        // Wait for transmit buffer empty
        while(!(UCSR0A & (1 << UDRE0)));
        
        // Send character
        UDR0 = *message++;
    }
}

void Debug_PrintNumber(int16_t number) {
    char buffer[8];
    sprintf(buffer, "%d", number);
    Debug_Print(buffer);
}

void Debug_PrintFloat(float number, uint8_t decimals) {
    char buffer[16];
    dtostrf(number, 0, decimals, buffer);
    Debug_Print(buffer);
}

void Debug_PrintHex(uint16_t number) {
    char buffer[8];
    sprintf(buffer, "0x%04X", number);
    Debug_Print(buffer);
}

void Debug_NewLine(void) {
    Debug_Print("\r\n");
}
