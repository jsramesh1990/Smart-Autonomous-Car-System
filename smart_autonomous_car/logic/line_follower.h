/**
 * smart_autonomous_car/src/logic/line_follower.h
 * 
 * Line following algorithm with PID control
 */

#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include "../config.h"

// Line Follower Functions
void LineFollower_Init(void);
void LineFollower_Execute(SystemData *data);
void LineFollower_Calibrate(void);
bool LineFollower_IsLineDetected(void);
float LineFollower_CalculateError(void);
void LineFollower_UpdatePID(PIDController *pid, float kp, float ki, float kd);
void LineFollower_SetSpeed(uint8_t base_speed);

#endif // LINE_FOLLOWER_H
