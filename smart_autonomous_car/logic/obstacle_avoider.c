/**
 * smart_autonomous_car/src/logic/obstacle_avoider.c
 * 
 * Obstacle avoidance implementation
 */

#include "obstacle_avoider.h"
#include "../peripherals/motor_driver.h"

// Obstacle avoider state
static float obstacle_threshold = OBSTACLE_THRESHOLD;
static uint8_t avoid_speed = DEFAULT_SPEED;
static uint32_t last_avoid_time = 0;

void ObstacleAvoider_Init(void) {
    obstacle_threshold = OBSTACLE_THRESHOLD;
    avoid_speed = DEFAULT_SPEED;
    last_avoid_time = 0;
}

void ObstacleAvoider_Execute(SystemData *data) {
    float distance = data->sensors.obstacle_distance;
    
    if(distance < 5.0) {
        // Emergency stop - too close!
        ObstacleAvoider_EmergencyStop();
        return;
    }
    
    if(distance < obstacle_threshold) {
        // Obstacle detected - avoid it
        ObstacleAvoider_AvoidObstacle(distance);
        last_avoid_time = millis();
    } else {
        // No obstacle - move forward
        if(millis() - last_avoid_time < 1000) {
            // Continue avoidance maneuver for 1 second
            return;
        }
        Motor_Forward(avoid_speed);
        data->motors.target_speed = avoid_speed;
    }
}

void ObstacleAvoider_SetThreshold(float threshold) {
    if(threshold >= 5.0 && threshold <= 50.0) {
        obstacle_threshold = threshold;
    }
}

bool ObstacleAvoider_DetectObstacle(void) {
    return (sys_data.sensors.obstacle_distance < obstacle_threshold && 
            sys_data.sensors.obstacle_distance > 2.0);
}

void ObstacleAvoider_AvoidObstacle(float distance) {
    // Determine avoidance strategy based on distance
    if(distance < 10.0) {
        // Very close - back up and turn
        ObstacleAvoider_BackAndTurn();
    } else if(distance < obstacle_threshold) {
        // Close but not critical - turn away
        ObstacleAvoider_TurnAway();
    }
}

void ObstacleAvoider_BackAndTurn(void) {
    // Back up for 500ms
    Motor_Backward(150);
    _delay_ms(500);
    
    // Turn 90 degrees
    if(rand() % 2 == 0) {
        Motor_RotateLeft(150);
    } else {
        Motor_RotateRight(150);
    }
    _delay_ms(800); // Time for ~90 degree turn
    
    // Stop and wait
    Motor_Stop(MOTOR_LEFT);
    Motor_Stop(MOTOR_RIGHT);
    _delay_ms(200);
}

void ObstacleAvoider_TurnAway(void) {
    // Gentle turn based on "which side is clearer"
    // For simplicity, alternate directions
    static bool turn_left = true;
    
    if(turn_left) {
        Motor_TurnLeft(avoid_speed);
    } else {
        Motor_TurnRight(avoid_speed);
    }
    
    // Turn for a bit
    _delay_ms(300);
    
    // Alternate direction for next time
    turn_left = !turn_left;
}

void ObstacleAvoider_EmergencyStop(void) {
    Motor_EmergencyStop();
    sys_data.state = STATE_EMERGENCY_STOP;
    
    // Sound alarm (if available)
    // Flash lights
    LED_SetColor(LED_RED);
    for(uint8_t i = 0; i < 10; i++) {
        LED_Toggle(1);
        LED_Toggle(2);
        LED_Toggle(3);
        LED_Toggle(4);
        _delay_ms(100);
    }
}
