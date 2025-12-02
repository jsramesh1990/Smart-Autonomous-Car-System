/**
 * smart_autonomous_car/src/peripherals/display_manager.c
 * 
 * Display management implementation
 */

#include "display_manager.h"
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

// LCD Command definitions
#define LCD_CLEAR_DISPLAY 0x01
#define LCD_RETURN_HOME 0x02
#define LCD_ENTRY_MODE_SET 0x04
#define LCD_DISPLAY_CONTROL 0x08
#define LCD_CURSOR_SHIFT 0x10
#define LCD_FUNCTION_SET 0x20
#define LCD_SET_CGRAM_ADDR 0x40
#define LCD_SET_DDRAM_ADDR 0x80

// LCD flags for display entry mode
#define LCD_ENTRY_RIGHT 0x00
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INCREMENT 0x01
#define LCD_ENTRY_SHIFT_DECREMENT 0x00

// LCD flags for display on/off control
#define LCD_DISPLAY_ON 0x04
#define LCD_DISPLAY_OFF 0x00
#define LCD_CURSOR_ON 0x02
#define LCD_CURSOR_OFF 0x00
#define LCD_BLINK_ON 0x01
#define LCD_BLINK_OFF 0x00

// LCD flags for display/cursor shift
#define LCD_DISPLAY_MOVE 0x08
#define LCD_CURSOR_MOVE 0x00
#define LCD_MOVE_RIGHT 0x04
#define LCD_MOVE_LEFT 0x00

// LCD flags for function set
#define LCD_8BIT_MODE 0x10
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// Custom character definitions
const uint8_t battery_full[8] = {
    0b01110,
    0b11111,
    0b10001,
    0b10101,
    0b10101,
    0b10001,
    0b11111,
    0b01110
};

const uint8_t battery_half[8] = {
    0b01110,
    0b11111,
    0b10001,
    0b10001,
    0b10101,
    0b10101,
    0b11111,
    0b01110
};

const uint8_t battery_low[8] = {
    0b01110,
    0b11111,
    0b10001,
    0b10001,
    0b10001,
    0b10101,
    0b11111,
    0b01110
};

const uint8_t arrow_up[8] = {
    0b00100,
    0b01110,
    0b11111,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100
};

const uint8_t arrow_down[8] = {
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b00100,
    0b11111,
    0b01110,
    0b00100
};

const uint8_t degree_symbol[8] = {
    0b01100,
    0b10010,
    0b10010,
    0b01100,
    0b00000,
    0b00000,
    0b00000,
    0b00000
};

// 7-segment digit patterns (common cathode)
const uint8_t digit_patterns[] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71  // F
};

// Private function prototypes
static void LCD_SendCommand(uint8_t cmd);
static void LCD_SendData(uint8_t data);
static void LCD_PulseEnable(void);
static void LCD_SendNibble(uint8_t nibble);
static void SevenSegment_DisplayDigit(uint8_t digit, uint8_t position);

void LCD_Init(void) {
    // Configure LCD pins as output
    DDRD |= (1 << LCD_RS_PIN) | (1 << LCD_RW_PIN) | (1 << LCD_EN_PIN);
    LCD_DATA_DDR = 0xFF; // All data pins as output
    
    // Wait for LCD to power up
    _delay_ms(50);
    
    // Initialization sequence for 4-bit mode
    LCD_SendNibble(0x03);
    _delay_ms(5);
    LCD_SendNibble(0x03);
    _delay_us(150);
    LCD_SendNibble(0x03);
    _delay_us(150);
    LCD_SendNibble(0x02); // Set to 4-bit mode
    
    // Function set: 4-bit mode, 2 lines, 5x8 dots
    LCD_SendCommand(LCD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8DOTS);
    
    // Display off
    LCD_SendCommand(LCD_DISPLAY_CONTROL | LCD_DISPLAY_OFF);
    
    // Clear display
    LCD_Clear();
    
    // Entry mode set: increment, no shift
    LCD_SendCommand(LCD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
    
    // Display on, cursor off, blink off
    LCD_SendCommand(LCD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    
    // Create custom characters
    LCD_CreateCustomChar(0, battery_full);
    LCD_CreateCustomChar(1, battery_half);
    LCD_CreateCustomChar(2, battery_low);
    LCD_CreateCustomChar(3, arrow_up);
    LCD_CreateCustomChar(4, arrow_down);
    LCD_CreateCustomChar(5, degree_symbol);
}

static void LCD_SendCommand(uint8_t cmd) {
    PORTD &= ~(1 << LCD_RS_PIN); // RS = 0 for command
    PORTD &= ~(1 << LCD_RW_PIN); // RW = 0 for write
    LCD_SendNibble(cmd >> 4);    // Send high nibble
    LCD_SendNibble(cmd & 0x0F);  // Send low nibble
    _delay_us(50);               // Wait for command to execute
}

static void LCD_SendData(uint8_t data) {
    PORTD |= (1 << LCD_RS_PIN);  // RS = 1 for data
    PORTD &= ~(1 << LCD_RW_PIN); // RW = 0 for write
    LCD_SendNibble(data >> 4);   // Send high nibble
    LCD_SendNibble(data & 0x0F); // Send low nibble
    _delay_us(50);               // Wait for data to write
}

static void LCD_PulseEnable(void) {
    PORTD |= (1 << LCD_EN_PIN);
    _delay_us(1);
    PORTD &= ~(1 << LCD_EN_PIN);
    _delay_us(50);
}

static void LCD_SendNibble(uint8_t nibble) {
    // Clear data pins
    LCD_DATA_PORT &= 0xF0;
    // Set data pins according to nibble
    LCD_DATA_PORT |= (nibble & 0x0F);
    LCD_PulseEnable();
}

void LCD_Clear(void) {
    LCD_SendCommand(LCD_CLEAR_DISPLAY);
    _delay_ms(2); // Clear command needs more time
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? 0x00 : 0x40;
    address += col;
    LCD_SendCommand(LCD_SET_DDRAM_ADDR | address);
}

void LCD_WriteChar(char ch) {
    LCD_SendData(ch);
}

void LCD_WriteString(const char *str) {
    while(*str) {
        LCD_WriteChar(*str++);
    }
}

void LCD_WriteNumber(int16_t num) {
    char buffer[8];
    itoa(num, buffer, 10);
    LCD_WriteString(buffer);
}

void LCD_WriteFloat(float num, uint8_t decimals) {
    char buffer[16];
    dtostrf(num, 4, decimals, buffer);
    LCD_WriteString(buffer);
}

void LCD_CreateCustomChar(uint8_t location, const uint8_t *charmap) {
    if(location > 7) return;
    
    LCD_SendCommand(LCD_SET_CGRAM_ADDR | (location << 3));
    for(uint8_t i = 0; i < 8; i++) {
        LCD_SendData(charmap[i]);
    }
}

void SevenSegment_Init(void) {
    // Configure 7-segment pins as output
    SEVEN_SEG_DDR |= (1 << SEVEN_SEG_EN1) | (1 << SEVEN_SEG_EN2);
    SEVEN_SEG_PORT = 0xFF; // Turn off all segments initially
}

void SevenSegment_DisplayNumber(uint16_t num) {
    static uint8_t current_digit = 0;
    static uint32_t last_update = 0;
    
    if(millis() - last_update < 5) {
        return; // Multiplex at ~200Hz
    }
    last_update = millis();
    
    // Extract digits
    uint8_t digits[4];
    digits[0] = (num / 1000) % 10;
    digits[1] = (num / 100) % 10;
    digits[2] = (num / 10) % 10;
    digits[3] = num % 10;
    
    // Turn off both displays
    SEVEN_SEG_PORT |= (1 << SEVEN_SEG_EN1) | (1 << SEVEN_SEG_EN2);
    
    // Display current digit
    if(current_digit < 4) {
        SevenSegment_DisplayDigit(digits[current_digit], current_digit);
    }
    
    // Move to next digit
    current_digit = (current_digit + 1) % 4;
}

static void SevenSegment_DisplayDigit(uint8_t digit, uint8_t position) {
    // Set digit pattern
    if(digit < 16) {
        SEVEN_SEG_PORT = digit_patterns[digit];
    }
    
    // Enable the correct digit
    if(position == 0) {
        SEVEN_SEG_PORT &= ~(1 << SEVEN_SEG_EN1);
    } else if(position == 1) {
        SEVEN_SEG_PORT &= ~(1 << SEVEN_SEG_EN2);
    }
}

void SevenSegment_DisplayFloat(float num, uint8_t decimals) {
    uint16_t integer_part = (uint16_t)num;
    uint16_t fractional_part = (uint16_t)((num - integer_part) * 100);
    
    // Simple display: show integer part
    SevenSegment_DisplayNumber(integer_part);
}

void SevenSegment_DisplayHex(uint16_t num) {
    static uint8_t current_digit = 0;
    static uint32_t last_update = 0;
    
    if(millis() - last_update < 5) {
        return;
    }
    last_update = millis();
    
    uint8_t digits[4];
    digits[0] = (num >> 12) & 0x0F;
    digits[1] = (num >> 8) & 0x0F;
    digits[2] = (num >> 4) & 0x0F;
    digits[3] = num & 0x0F;
    
    // Turn off both displays
    SEVEN_SEG_PORT |= (1 << SEVEN_SEG_EN1) | (1 << SEVEN_SEG_EN2);
    
    if(current_digit < 4) {
        SevenSegment_DisplayDigit(digits[current_digit], current_digit);
    }
    
    current_digit = (current_digit + 1) % 4;
}

void SevenSegment_Clear(void) {
    SEVEN_SEG_PORT = 0xFF;
    SEVEN_SEG_PORT |= (1 << SEVEN_SEG_EN1) | (1 << SEVEN_SEG_EN2);
}

void Display_Init(void) {
    LCD_Init();
    SevenSegment_Init();
    
    // Display welcome message
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("Smart Car System");
    LCD_SetCursor(1, 0);
    LCD_WriteString("Initializing...");
    
    _delay_ms(1000);
}

void Display_Update(void) {
    static uint32_t last_update = 0;
    static uint8_t display_mode = 0;
    
    if(millis() - last_update < DISPLAY_UPDATE_INTERVAL) {
        return;
    }
    last_update = millis();
    
    // Cycle through display modes
    display_mode = (display_mode + 1) % 4;
    
    switch(display_mode) {
        case 0:
            Display_ShowSystemStatus();
            break;
        case 1:
            Display_ShowSensorData();
            break;
        case 2:
            // Show motor status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_WriteString("Motors:");
            LCD_SetCursor(0, 8);
            LCD_WriteString("L:");
            LCD_WriteNumber(sys_data.motors.left_speed);
            LCD_SetCursor(1, 0);
            LCD_WriteString("R:");
            LCD_WriteNumber(sys_data.motors.right_speed);
            LCD_WriteString(" T:");
            LCD_WriteNumber(sys_data.motors.target_speed);
            break;
        case 3:
            // Show runtime and battery
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_WriteString("Runtime:");
            LCD_WriteNumber(sys_data.runtime / 60);
            LCD_WriteString("m");
            LCD_SetCursor(1, 0);
            LCD_WriteString("Battery:");
            LCD_WriteNumber(sys_data.battery_percent);
            LCD_WriteString("% ");
            LCD_WriteChar(sys_data.battery_percent > 70 ? 0 : 
                         sys_data.battery_percent > 30 ? 1 : 2);
            break;
    }
    
    // Update 7-segment with obstacle distance
    SevenSegment_DisplayNumber((uint16_t)sys_data.sensors.obstacle_distance);
}

void Display_ShowMessage(const char *line1, const char *line2) {
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString(line1);
    LCD_SetCursor(1, 0);
    LCD_WriteString(line2);
}

void Display_ShowSensorData(void) {
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("IR:");
    LCD_WriteNumber(sys_data.sensors.ir_left);
    LCD_WriteString("-");
    LCD_WriteNumber(sys_data.sensors.ir_right);
    LCD_SetCursor(1, 0);
    LCD_WriteString("US:");
    LCD_WriteFloat(sys_data.sensors.obstacle_distance, 1);
    LCD_WriteString("cm");
}

void Display_ShowSystemStatus(void) {
    LCD_Clear();
    LCD_SetCursor(0, 0);
    
    // Show mode
    switch(sys_data.mode) {
        case MODE_MANUAL: LCD_WriteString("MANUAL"); break;
        case MODE_LINE_FOLLOW: LCD_WriteString("LINE FOLLOW"); break;
        case MODE_WALL_FOLLOW_LEFT: LCD_WriteString("WALL L"); break;
        case MODE_WALL_FOLLOW_RIGHT: LCD_WriteString("WALL R"); break;
        case MODE_OBSTACLE_AVOID: LCD_WriteString("OBST AVOID"); break;
        case MODE_AUTO_PILOT: LCD_WriteString("AUTO PILOT"); break;
        case MODE_CALIBRATION: LCD_WriteString("CALIBRATE"); break;
    }
    
    LCD_SetCursor(0, 12);
    if(sys_data.state == STATE_ACTIVE) {
        LCD_WriteChar('*');
    } else if(sys_data.state == STATE_EMERGENCY_STOP) {
        LCD_WriteChar('!');
    } else {
        LCD_WriteChar('-');
    }
    
    LCD_SetCursor(1, 0);
    LCD_WriteString("State:");
    switch(sys_data.state) {
        case STATE_INIT: LCD_WriteString("INIT"); break;
        case STATE_IDLE: LCD_WriteString("IDLE"); break;
        case STATE_ACTIVE: LCD_WriteString("ACTIVE"); break;
        case STATE_EMERGENCY_STOP: LCD_WriteString("EMERGENCY"); break;
        case STATE_ERROR: LCD_WriteString("ERROR"); break;
        case STATE_CALIBRATING: LCD_WriteString("CALIB"); break;
        case STATE_SLEEP: LCD_WriteString("SLEEP"); break;
    }
}

void Display_ShowError(uint16_t error_code) {
    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_WriteString("ERROR:");
    LCD_SetCursor(1, 0);
    
    if(error_code & ERROR_ULTRASONIC_TIMEOUT) {
        LCD_WriteString("ULTRASONIC");
    } else if(error_code & ERROR_IR_SENSOR_FAIL) {
        LCD_WriteString("IR SENSOR");
    } else if(error_code & ERROR_MOTOR_OVERLOAD_LEFT) {
        LCD_WriteString("MOTOR L OVLD");
    } else if(error_code & ERROR_MOTOR_OVERLOAD_RIGHT) {
        LCD_WriteString("MOTOR R OVLD");
    } else if(error_code & ERROR_LOW_BATTERY) {
        LCD_WriteString("LOW BATTERY");
    } else if(error_code & ERROR_HIGH_BATTERY) {
        LCD_WriteString("HIGH BATTERY");
    } else if(error_code & ERROR_LINE_LOST) {
        LCD_WriteString("LINE LOST");
    } else if(error_code & ERROR_WALL_LOST) {
        LCD_WriteString("WALL LOST");
    } else if(error_code & ERROR_SYSTEM_OVERHEAT) {
        LCD_WriteString("OVERHEAT");
    } else if(error_code & ERROR_COMMUNICATION) {
        LCD_WriteString("COMM ERROR");
    } else {
        LCD_WriteString("UNKNOWN");
    }
    
    // Show error code on 7-segment in hex
    SevenSegment_DisplayHex(error_code);
}
