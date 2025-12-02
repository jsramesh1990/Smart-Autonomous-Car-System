/**
 * smart_autonomous_car/src/peripherals/led_controller.h
 * 
 * LED control module for status indication
 */

#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include "../config.h"

// LED Functions
void LED_Init(void);
void LED_Set(uint8_t led_num, bool state);
void LED_Toggle(uint8_t led_num);
void LED_SetColor(LEDColor color);
void LED_Blink(uint8_t led_num, uint16_t on_time, uint16_t off_time);
void LED_Sequence(uint8_t pattern);
void LED_UpdateStatus(void);
void LED_ErrorFlash(uint16_t error_code);

// LED Patterns
#define PATTERN_SYSTEM_READY 0x01
#define PATTERN_CALIBRATING  0x02
#define PATTERN_RUNNING      0x03
#define PATTERN_ERROR        0x04
#define PATTERN_LOW_BATTERY  0x05
#define PATTERN_LINE_DETECTED 0x06
#define PATTERN_OBSTACLE_DETECTED 0x07

#endif // LED_CONTROLLER_H
