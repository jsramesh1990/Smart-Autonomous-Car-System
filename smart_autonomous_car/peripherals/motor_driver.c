/**
 * smart_autonomous_car/src/peripherals/motor_driver.c
 * 
 * Motor control implementation for L298N
 */

#include "motor_driver.h"
#include <avr/io.h>
#include <avr/interrupt.h>

// PWM configuration for Timer0 (8-bit)
#define PWM_FREQUENCY 5000  // 5kHz
#define PWM_PRESCALER 8

// Current sensing variables
static uint16_t motor_current_left = 0;
static uint16_t motor_current_right = 0;

void Motor_Init(void) {
    // Configure motor pins as output
    MOTOR_DDR |= (1 << MOTOR_LEFT_EN) | (1 << MOTOR_LEFT_IN1) | 
                 (1 << MOTOR_LEFT_IN2) | (1 << MOTOR_RIGHT_EN) | 
                 (1 << MOTOR_RIGHT_IN1) | (1 << MOTOR_RIGHT_IN2);
    
    // Initialize PWM for motor speed control
    // Timer0 for left motor (OC0A = PD6)
    // Timer0 for right motor (OC0B = PD5)
    
    // Fast PWM mode, non-inverting
    TCCR0A = (1 << WGM00) | (1 << WGM01) |  // Fast PWM
             (1 << COM0A1) |                // Clear OC0A on compare match
             (1 << COM0B1);                 // Clear OC0B on compare match
    
    // Prescaler 8
    TCCR0B = (1 << CS01);
    
    // Set initial duty cycle to 0%
    OCR0A = 0;
    OCR0B = 0;
    
    // Set initial direction to forward
    MOTOR_PORT &= ~((1 << MOTOR_LEFT_IN1) | (1 << MOTOR_LEFT_IN2) |
                    (1 << MOTOR_RIGHT_IN1) | (1 << MOTOR_RIGHT_IN2));
    
    // Enable motor drivers
    MOTOR_PORT |= (1 << MOTOR_LEFT_EN) | (1 << MOTOR_RIGHT_EN);
}

void Motor_SetSpeed(uint8_t motor, uint8_t speed) {
    // Limit speed to valid range
    if(speed > MAX_MOTOR_SPEED) {
        speed = MAX_MOTOR_SPEED;
    }
    
    if(motor == MOTOR_LEFT) {
        OCR0A = speed;
        sys_data.motors.left_speed = speed;
    } else if(motor == MOTOR_RIGHT) {
        OCR0B = speed;
        sys_data.motors.right_speed = speed;
    }
}

void Motor_SetDirection(uint8_t motor, MotorDirection dir) {
    uint8_t in1_pin, in2_pin;
    
    if(motor == MOTOR_LEFT) {
        in1_pin = MOTOR_LEFT_IN1;
        in2_pin = MOTOR_LEFT_IN2;
        sys_data.motors.left_dir = dir;
    } else if(motor == MOTOR_RIGHT) {
        in1_pin = MOTOR_RIGHT_IN1;
        in2_pin = MOTOR_RIGHT_IN2;
        sys_data.motors.right_dir = dir;
    } else {
        return;
    }
    
    switch(dir) {
        case DIR_FORWARD:
            MOTOR_PORT &= ~(1 << in1_pin);
            MOTOR_PORT |= (1 << in2_pin);
            break;
        case DIR_BACKWARD:
            MOTOR_PORT |= (1 << in1_pin);
            MOTOR_PORT &= ~(1 << in2_pin);
            break;
        case DIR_STOP:
            MOTOR_PORT &= ~(1 << in1_pin);
            MOTOR_PORT &= ~(1 << in2_pin);
            break;
        case DIR_ROTATE_LEFT:
            if(motor == MOTOR_LEFT) {
                MOTOR_PORT &= ~(1 << in1_pin);
                MOTOR_PORT |= (1 << in2_pin);
            } else {
                MOTOR_PORT |= (1 << in1_pin);
                MOTOR_PORT &= ~(1 << in2_pin);
            }
            break;
        case DIR_ROTATE_RIGHT:
            if(motor == MOTOR_LEFT) {
                MOTOR_PORT |= (1 << in1_pin);
                MOTOR_PORT &= ~(1 << in2_pin);
            } else {
                MOTOR_PORT &= ~(1 << in1_pin);
                MOTOR_PORT |= (1 << in2_pin);
            }
            break;
    }
}

void Motor_Stop(uint8_t motor) {
    Motor_SetSpeed(motor, 0);
    Motor_SetDirection(motor, DIR_STOP);
}

void Motor_Brake(uint8_t motor) {
    // Short motor terminals for braking
    uint8_t in1_pin, in2_pin;
    
    if(motor == MOTOR_LEFT) {
        in1_pin = MOTOR_LEFT_IN1;
        in2_pin = MOTOR_LEFT_IN2;
    } else if(motor == MOTOR_RIGHT) {
        in1_pin = MOTOR_RIGHT_IN1;
        in2_pin = MOTOR_RIGHT_IN2;
    } else {
        return;
    }
    
    MOTOR_PORT |= (1 << in1_pin);
    MOTOR_PORT |= (1 << in2_pin);
    Motor_SetSpeed(motor, MAX_MOTOR_SPEED);
    
    // Hold brake for 100ms
    _delay_ms(100);
    Motor_Stop(motor);
}

void Motor_Coast(uint8_t motor) {
    Motor_SetDirection(motor, DIR_STOP);
    Motor_SetSpeed(motor, 0);
}

void Motor_Forward(uint8_t speed) {
    Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
    Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
    Motor_SetSpeed(MOTOR_LEFT, speed);
    Motor_SetSpeed(MOTOR_RIGHT, speed);
    sys_data.motors.target_speed = speed;
}

void Motor_Backward(uint8_t speed) {
    Motor_SetDirection(MOTOR_LEFT, DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RIGHT, DIR_BACKWARD);
    Motor_SetSpeed(MOTOR_LEFT, speed);
    Motor_SetSpeed(MOTOR_RIGHT, speed);
    sys_data.motors.target_speed = speed;
}

void Motor_TurnLeft(uint8_t speed) {
    // Left motor slower, right motor faster
    uint8_t left_speed = speed * 0.7;
    uint8_t right_speed = speed;
    Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
    Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
    Motor_SetSpeed(MOTOR_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_RIGHT, right_speed);
}

void Motor_TurnRight(uint8_t speed) {
    // Left motor faster, right motor slower
    uint8_t left_speed = speed;
    uint8_t right_speed = speed * 0.7;
    Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
    Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
    Motor_SetSpeed(MOTOR_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_RIGHT, right_speed);
}

void Motor_RotateLeft(uint8_t speed) {
    Motor_SetDirection(MOTOR_LEFT, DIR_BACKWARD);
    Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
    Motor_SetSpeed(MOTOR_LEFT, speed);
    Motor_SetSpeed(MOTOR_RIGHT, speed);
}

void Motor_RotateRight(uint8_t speed) {
    Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
    Motor_SetDirection(MOTOR_RIGHT, DIR_BACKWARD);
    Motor_SetSpeed(MOTOR_LEFT, speed);
    Motor_SetSpeed(MOTOR_RIGHT, speed);
}

void Motor_DifferentialDrive(int16_t left_speed, int16_t right_speed) {
    // Handle negative speeds (reverse)
    if(left_speed >= 0) {
        Motor_SetDirection(MOTOR_LEFT, DIR_FORWARD);
        Motor_SetSpeed(MOTOR_LEFT, left_speed);
    } else {
        Motor_SetDirection(MOTOR_LEFT, DIR_BACKWARD);
        Motor_SetSpeed(MOTOR_LEFT, -left_speed);
    }
    
    if(right_speed >= 0) {
        Motor_SetDirection(MOTOR_RIGHT, DIR_FORWARD);
        Motor_SetSpeed(MOTOR_RIGHT, right_speed);
    } else {
        Motor_SetDirection(MOTOR_RIGHT, DIR_BACKWARD);
        Motor_SetSpeed(MOTOR_RIGHT, -right_speed);
    }
}

void Motor_EmergencyStop(void) {
    // Immediate stop with braking
    Motor_Brake(MOTOR_LEFT);
    Motor_Brake(MOTOR_RIGHT);
    
    // Disable PWM outputs
    OCR0A = 0;
    OCR0B = 0;
    
    // Set all control pins low
    MOTOR_PORT &= ~((1 << MOTOR_LEFT_IN1) | (1 << MOTOR_LEFT_IN2) |
                    (1 << MOTOR_RIGHT_IN1) | (1 << MOTOR_RIGHT_IN2));
    
    sys_data.emergency_stop = true;
}

uint16_t Motor_ReadCurrent(uint8_t motor) {
    // In a real system, this would read from a current sensor
    // For simulation, return calculated value based on speed
    if(motor == MOTOR_LEFT) {
        // Simulate current reading (typically 100-500mA per motor)
        return 100 + (sys_data.motors.left_speed * 400 / MAX_MOTOR_SPEED);
    } else if(motor == MOTOR_RIGHT) {
        return 100 + (sys_data.motors.right_speed * 400 / MAX_MOTOR_SPEED);
    }
    return 0;
}
