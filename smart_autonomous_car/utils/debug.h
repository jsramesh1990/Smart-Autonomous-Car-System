/**
 * smart_autonomous_car/src/utils/debug.h
 * 
 * Debug utilities for serial output
 */

#ifndef DEBUG_H
#define DEBUG_H

#include <stdio.h>

// Debug Functions
void Debug_Init(void);
void Debug_Print(const char *message);
void Debug_PrintNumber(int16_t number);
void Debug_PrintFloat(float number, uint8_t decimals);
void Debug_PrintHex(uint16_t number);
void Debug_NewLine(void);

// Debug Macros
#ifdef DEBUG_ENABLE
    #define DEBUG_PRINT(msg) Debug_Print(msg)
    #define DEBUG_PRINT_NUM(num) Debug_PrintNumber(num)
    #define DEBUG_PRINT_FLOAT(num, dec) Debug_PrintFloat(num, dec)
    #define DEBUG_PRINT_HEX(num) Debug_PrintHex(num)
    #define DEBUG_NEWLINE() Debug_NewLine()
#else
    #define DEBUG_PRINT(msg)
    #define DEBUG_PRINT_NUM(num)
    #define DEBUG_PRINT_FLOAT(num, dec)
    #define DEBUG_PRINT_HEX(num)
    #define DEBUG_NEWLINE()
#endif

#endif // DEBUG_H
