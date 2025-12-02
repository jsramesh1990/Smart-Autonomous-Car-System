/**
 * smart_autonomous_car/src/logic/wall_follower.h
 * 
 * Wall following algorithm with PID control
 */

#ifndef WALL_FOLLOWER_H
#define WALL_FOLLOWER_H

#include "../config.h"

// Wall Follower Functions
void WallFollower_Init(void);
void WallFollower_Execute(SystemData *data);
void WallFollower_SetSide(bool follow_left);
void WallFollower_SetDistance(float target_distance);
bool WallFollower_IsWallDetected(void);
float WallFollower_CalculateError(float current_distance);
void WallFollower_UpdatePID(PIDController *pid, float kp, float ki, float kd);

#endif // WALL_FOLLOWER_H
