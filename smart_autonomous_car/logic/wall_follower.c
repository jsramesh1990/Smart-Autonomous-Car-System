/**
 * smart_autonomous_car/src/logic/wall_follower.c
 * 
 * Wall following implementation
 */

#include "wall_follower.h"
#include "../peripherals/motor_driver.h"

// Wall follower state
static bool follow_left_side = true;
static float target_wall_distance = WALL_FOLLOW_DISTANCE;
static uint8_t wall_follower_speed = DEFAULT_SPEED;

void WallFollower_Init(void) {
    // Initialize PID controller
    sys_data.wall_pid.kp = WALL_PID_KP;
    sys_data.wall_pid.ki = WALL_PID_KI;
    sys_data.wall_pid.kd = WALL_PID_KD;
    sys_data.wall_pid.integral = 0;
    sys_data.wall_pid.derivative = 0;
    sys_data.wall_pid.prev_error = 0;
    sys_data.wall_pid.max_output = PID_MAX_OUTPUT;
    sys_data.wall_pid.min_output = PID_MIN_OUTPUT;
    
    follow_left_side = true;
    target_wall_distance = WALL_FOLLOW_DISTANCE;
    wall_follower_speed = DEFAULT_SPEED;
}

void WallFollower_Execute(SystemData *data) {
    float current_distance;
    bool wall_detected = Sensor_DetectWall(&current_distance);
    
    if(!wall_detected) {
        // Wall lost - enter search mode
        data->error_code |= ERROR_WALL_LOST;
        WallFollower_SearchWall();
        return;
    }
    
    // Wall detected - follow it
    data->error_code &= ~ERROR_WALL_LOST;
    
    // Calculate error from desired distance
    float error = WallFollower_CalculateError(current_distance);
    
    // Adjust error based on which side we're following
    if(!follow_left_side) {
        error = -error; // Invert for right side following
    }
    
    // Calculate PID correction
    float correction = PID_Calculate(&data->wall_pid, error);
    
    // Apply correction to motor speeds
    int16_t left_speed = wall_follower_speed;
    int16_t right_speed = wall_follower_speed;
    
    if(follow_left_side) {
        // Too close to left wall -> turn right
        // Too far from left wall -> turn left
        left_speed += correction;
        right_speed -= correction;
    } else {
        // Too close to right wall -> turn left
        // Too far from right wall -> turn right
        left_speed -= correction;
        right_speed += correction;
    }
    
    // Limit speeds
    if(left_speed > MAX_MOTOR_SPEED) left_speed = MAX_MOTOR_SPEED;
    if(left_speed < 0) left_speed = 0;
    if(right_speed > MAX_MOTOR_SPEED) right_speed = MAX_MOTOR_SPEED;
    if(right_speed < 0) right_speed = 0;
    
    // Apply motor control
    Motor_DifferentialDrive(left_speed, right_speed);
    
    // Update system data
    data->motors.left_speed = left_speed;
    data->motors.right_speed = right_speed;
    data->motors.target_speed = wall_follower_speed;
    data->sensors.wall_distance = current_distance;
}

void WallFollower_SetSide(bool follow_left) {
    follow_left_side = follow_left;
    
    if(follow_left_side) {
        sys_data.mode = MODE_WALL_FOLLOW_LEFT;
    } else {
        sys_data.mode = MODE_WALL_FOLLOW_RIGHT;
    }
}

void WallFollower_SetDistance(float target_distance) {
    if(target_distance >= 5.0 && target_distance <= 50.0) {
        target_wall_distance = target_distance;
    }
}

bool WallFollower_IsWallDetected(void) {
    float distance;
    return Sensor_DetectWall(&distance);
}

float WallFollower_CalculateError(float current_distance) {
    // Error is positive if too close, negative if too far
    return target_wall_distance - current_distance;
}

void WallFollower_SearchWall(void) {
    static bool searching = false;
    static uint32_t search_start_time = 0;
    
    if(!searching) {
        searching = true;
        search_start_time = millis();
        
        // Start searching by turning toward the wall side
        if(follow_left_side) {
            Motor_RotateLeft(150);
        } else {
            Motor_RotateRight(150);
        }
    }
    
    // Check if wall found
    float distance;
    if(Sensor_DetectWall(&distance)) {
        searching = false;
        return;
    }
    
    // Give up after 3 seconds and switch to other side
    if(millis() - search_start_time > 3000) {
        searching = false;
        follow_left_side = !follow_left_side;
        
        // Update mode
        if(follow_left_side) {
            sys_data.mode = MODE_WALL_FOLLOW_LEFT;
        } else {
            sys_data.mode = MODE_WALL_FOLLOW_RIGHT;
        }
    }
}

void WallFollower_UpdatePID(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0; // Reset integral on update
    pid->prev_error = 0;
}
