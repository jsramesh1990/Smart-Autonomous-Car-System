/**
 * smart_autonomous_car/src/peripherals/display_manager.h
 * 
 * Display management for LCD and 7-segment
 */

#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include "../config.h"

// LCD Functions
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_WriteChar(char ch);
void LCD_WriteString(const char *str);
void LCD_WriteNumber(int16_t num);
void LCD_WriteFloat(float num, uint8_t decimals);
void LCD_CreateCustomChar(uint8_t location, const uint8_t *charmap);

// 7-Segment Functions
void SevenSegment_Init(void);
void SevenSegment_DisplayNumber(uint16_t num);
void SevenSegment_DisplayFloat(float num, uint8_t decimals);
void SevenSegment_DisplayHex(uint16_t num);
void SevenSegment_Clear(void);

// Display Manager Functions
void Display_Init(void);
void Display_Update(void);
void Display_ShowMessage(const char *line1, const char *line2);
void Display_ShowSensorData(void);
void Display_ShowSystemStatus(void);
void Display_ShowError(uint16_t error_code);

#endif // DISPLAY_MANAGER_H
