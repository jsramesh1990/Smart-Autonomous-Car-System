/**
 * smart_autonomous_car/src/peripherals/motor_driver.h
 * 
 * Motor control driver for L298N
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "../config.h"

// Motor Functions
void Motor_Init(void);
void Motor_SetSpeed(uint8_t motor, uint8_t speed);
void Motor_SetDirection(uint8_t motor, MotorDirection dir);
void Motor_Stop(uint8_t motor);
void Motor_Brake(uint8_t motor);
void Motor_Coast(uint8_t motor);
void Motor_Forward(uint8_t speed);
void Motor_Backward(uint8_t speed);
void Motor_TurnLeft(uint8_t speed);
void Motor_TurnRight(uint8_t speed);
void Motor_RotateLeft(uint8_t speed);
void Motor_RotateRight(uint8_t speed);
void Motor_DifferentialDrive(int16_t left_speed, int16_t right_speed);
void Motor_EmergencyStop(void);
uint16_t Motor_ReadCurrent(uint8_t motor);

#endif // MOTOR_DRIVER_H
