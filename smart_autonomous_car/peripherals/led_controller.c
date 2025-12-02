/**
 * smart_autonomous_car/src/peripherals/led_controller.c
 * 
 * LED control implementation
 */

#include "led_controller.h"
#include <avr/io.h>
#include <util/delay.h>

// Global variables for LED blinking
static uint32_t blink_timers[8] = {0};
static uint16_t blink_on_times[8] = {0};
static uint16_t blink_off_times[8] = {0};
static bool blink_states[8] = {false};

void LED_Init(void) {
    // Set LED pins as output
    LED_DDR |= (1 << LED1_PIN) | (1 << LED2_PIN) | 
               (1 << LED3_PIN) | (1 << LED4_PIN) |
               (1 << RGB_RED_PIN) | (1 << RGB_GREEN_PIN) | 
               (1 << RGB_BLUE_PIN);
    
    // Turn off all LEDs initially
    LED_PORT &= ~((1 << LED1_PIN) | (1 << LED2_PIN) | 
                  (1 << LED3_PIN) | (1 << LED4_PIN) |
                  (1 << RGB_RED_PIN) | (1 << RGB_GREEN_PIN) | 
                  (1 << RGB_BLUE_PIN));
}

void LED_Set(uint8_t led_num, bool state) {
    switch(led_num) {
        case 1:
            if(state) LED_PORT |= (1 << LED1_PIN);
            else LED_PORT &= ~(1 << LED1_PIN);
            break;
        case 2:
            if(state) LED_PORT |= (1 << LED2_PIN);
            else LED_PORT &= ~(1 << LED2_PIN);
            break;
        case 3:
            if(state) LED_PORT |= (1 << LED3_PIN);
            else LED_PORT &= ~(1 << LED3_PIN);
            break;
        case 4:
            if(state) LED_PORT |= (1 << LED4_PIN);
            else LED_PORT &= ~(1 << LED4_PIN);
            break;
        case 5: // Red component of RGB
            if(state) LED_PORT |= (1 << RGB_RED_PIN);
            else LED_PORT &= ~(1 << RGB_RED_PIN);
            break;
        case 6: // Green component of RGB
            if(state) LED_PORT |= (1 << RGB_GREEN_PIN);
            else LED_PORT &= ~(1 << RGB_GREEN_PIN);
            break;
        case 7: // Blue component of RGB
            if(state) LED_PORT |= (1 << RGB_BLUE_PIN);
            else LED_PORT &= ~(1 << RGB_BLUE_PIN);
            break;
    }
}

void LED_Toggle(uint8_t led_num) {
    switch(led_num) {
        case 1:
            LED_PORT ^= (1 << LED1_PIN);
            break;
        case 2:
            LED_PORT ^= (1 << LED2_PIN);
            break;
        case 3:
            LED_PORT ^= (1 << LED3_PIN);
            break;
        case 4:
            LED_PORT ^= (1 << LED4_PIN);
            break;
        case 5:
            LED_PORT ^= (1 << RGB_RED_PIN);
            break;
        case 6:
            LED_PORT ^= (1 << RGB_GREEN_PIN);
            break;
        case 7:
            LED_PORT ^= (1 << RGB_BLUE_PIN);
            break;
    }
}

void LED_SetColor(LEDColor color) {
    // Turn off all RGB LEDs first
    LED_PORT &= ~((1 << RGB_RED_PIN) | (1 << RGB_GREEN_PIN) | (1 << RGB_BLUE_PIN));
    
    switch(color) {
        case LED_RED:
            LED_PORT |= (1 << RGB_RED_PIN);
            break;
        case LED_GREEN:
            LED_PORT |= (1 << RGB_GREEN_PIN);
            break;
        case LED_BLUE:
            LED_PORT |= (1 << RGB_BLUE_PIN);
            break;
        case LED_YELLOW:
            LED_PORT |= (1 << RGB_RED_PIN) | (1 << RGB_GREEN_PIN);
            break;
        case LED_CYAN:
            LED_PORT |= (1 << RGB_GREEN_PIN) | (1 << RGB_BLUE_PIN);
            break;
        case LED_MAGENTA:
            LED_PORT |= (1 << RGB_RED_PIN) | (1 << RGB_BLUE_PIN);
            break;
        case LED_WHITE:
            LED_PORT |= (1 << RGB_RED_PIN) | (1 << RGB_GREEN_PIN) | (1 << RGB_BLUE_PIN);
            break;
        case LED_OFF:
        default:
            // Already turned off
            break;
    }
}

void LED_Blink(uint8_t led_num, uint16_t on_time, uint16_t off_time) {
    if(led_num >= 1 && led_num <= 7) {
        uint8_t idx = led_num - 1;
        blink_on_times[idx] = on_time;
        blink_off_times[idx] = off_time;
        blink_timers[idx] = millis();
        blink_states[idx] = true;
        LED_Set(led_num, true); // Start with LED on
    }
}

void LED_UpdateStatus(void) {
    static uint32_t last_update = 0;
    uint32_t current_time = millis();
    
    if(current_time - last_update < 10) {
        return; // Update every 10ms
    }
    last_update = current_time;
    
    // Update blinking LEDs
    for(uint8_t i = 0; i < 7; i++) {
        if(blink_on_times[i] > 0 && blink_off_times[i] > 0) {
            uint32_t elapsed = current_time - blink_timers[i];
            
            if(blink_states[i] && elapsed >= blink_on_times[i]) {
                LED_Set(i + 1, false);
                blink_states[i] = false;
                blink_timers[i] = current_time;
            } else if(!blink_states[i] && elapsed >= blink_off_times[i]) {
                LED_Set(i + 1, true);
                blink_states[i] = true;
                blink_timers[i] = current_time;
            }
        }
    }
    
    // Update status based on system state
    switch(sys_data.state) {
        case STATE_INIT:
            LED_SetColor(LED_CYAN);
            LED_Sequence(PATTERN_CALIBRATING);
            break;
            
        case STATE_IDLE:
            LED_SetColor(LED_BLUE);
            LED_Set(1, true);
            LED_Set(2, false);
            LED_Set(3, false);
            LED_Set(4, true);
            break;
            
        case STATE_ACTIVE:
            // Mode-specific LED patterns
            switch(sys_data.mode) {
                case MODE_MANUAL:
                    LED_SetColor(LED_GREEN);
                    break;
                case MODE_LINE_FOLLOW:
                    LED_SetColor(LED_YELLOW);
                    // Show line detection on individual LEDs
                    LED_Set(1, sys_data.sensors.ir_left > LINE_DETECTED_THRESHOLD);
                    LED_Set(2, sys_data.sensors.ir_center_left > LINE_DETECTED_THRESHOLD);
                    LED_Set(3, sys_data.sensors.ir_center_right > LINE_DETECTED_THRESHOLD);
                    LED_Set(4, sys_data.sensors.ir_right > LINE_DETECTED_THRESHOLD);
                    break;
                case MODE_WALL_FOLLOW_LEFT:
                case MODE_WALL_FOLLOW_RIGHT:
                    LED_SetColor(LED_CYAN);
                    // Blink based on wall distance
                    if(sys_data.sensors.wall_distance < 10.0f) {
                        LED_Blink(1, 100, 100);
                    }
                    break;
                case MODE_OBSTACLE_AVOID:
                    LED_SetColor(LED_MAGENTA);
                    break;
                case MODE_AUTO_PILOT:
                    LED_SetColor(LED_WHITE);
                    break;
                default:
                    break;
            }
            break;
            
        case STATE_EMERGENCY_STOP:
            LED_SetColor(LED_RED);
            LED_Blink(1, 100, 100);
            LED_Blink(2, 100, 100);
            LED_Blink(3, 100, 100);
            LED_Blink(4, 100, 100);
            break;
            
        case STATE_ERROR:
            LED_ErrorFlash(sys_data.error_code);
            break;
            
        default:
            break;
    }
}

void LED_Sequence(uint8_t pattern) {
    static uint8_t seq_step = 0;
    static uint32_t seq_timer = 0;
    
    if(millis() - seq_timer < 200) {
        return; // Wait 200ms between steps
    }
    seq_timer = millis();
    
    switch(pattern) {
        case PATTERN_SYSTEM_READY:
            // Knight rider style
            LED_Set(1 + seq_step, true);
            if(seq_step > 0) LED_Set(seq_step, false);
            seq_step = (seq_step + 1) % 4;
            break;
            
        case PATTERN_CALIBRATING:
            // Progress bar style
            for(uint8_t i = 1; i <= 4; i++) {
                LED_Set(i, i <= seq_step);
            }
            seq_step = (seq_step + 1) % 5;
            break;
            
        case PATTERN_RUNNING:
            // Rotating pattern
            uint8_t led_on = (seq_step % 4) + 1;
            for(uint8_t i = 1; i <= 4; i++) {
                LED_Set(i, i == led_on);
            }
            seq_step = (seq_step + 1) % 4;
            break;
            
        case PATTERN_ERROR:
            // All LEDs flash
            bool flash_state = (seq_step % 2) == 0;
            for(uint8_t i = 1; i <= 4; i++) {
                LED_Set(i, flash_state);
            }
            seq_step++;
            break;
    }
}

void LED_ErrorFlash(uint16_t error_code) {
    static uint8_t error_bit = 0;
    static uint32_t error_timer = 0;
    
    // Find the highest error bit
    if(error_code == 0) {
        LED_SetColor(LED_OFF);
        return;
    }
    
    if(millis() - error_timer < 500) {
        return;
    }
    error_timer = millis();
    
    // Flash the error bit position
    while(error_bit < 16) {
        if(error_code & (1 << error_bit)) {
            // Flash this error position
            bool flash_state = ((millis() / 250) % 2) == 0;
            if(flash_state) {
                // Binary display of error bit on LEDs 1-4
                for(uint8_t i = 0; i < 4; i++) {
                    LED_Set(i + 1, (error_bit >> i) & 0x01);
                }
                LED_SetColor(LED_RED);
            } else {
                for(uint8_t i = 1; i <= 4; i++) {
                    LED_Set(i, false);
                }
                LED_SetColor(LED_OFF);
            }
            break;
        }
        error_bit = (error_bit + 1) % 16;
    }
}
