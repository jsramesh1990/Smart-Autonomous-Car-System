/**
 * smart_autonomous_car/src/logic/obstacle_avoider.h
 * 
 * Obstacle avoidance algorithm
 */

#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H

#include "../config.h"

// Obstacle Avoider Functions
void ObstacleAvoider_Init(void);
void ObstacleAvoider_Execute(SystemData *data);
void ObstacleAvoider_SetThreshold(float threshold);
bool ObstacleAvoider_DetectObstacle(void);
void ObstacleAvoider_AvoidObstacle(float distance);
void ObstacleAvoider_EmergencyStop(void);

#endif // OBSTACLE_AVOIDER_H
