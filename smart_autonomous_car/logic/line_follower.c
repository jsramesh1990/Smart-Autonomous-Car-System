/**
 * smart_autonomous_car/src/logic/line_follower.c
 * 
 * Line following implementation
 */

#include "line_follower.h"
#include "../peripherals/motor_driver.h"
#include <math.h>

// Line follower state
static bool line_lost = false;
static uint32_t line_lost_time = 0;
static uint8_t line_follower_speed = DEFAULT_SPEED;

void LineFollower_Init(void) {
    // Initialize PID controller
    sys_data.line_pid.kp = LINE_PID_KP;
    sys_data.line_pid.ki = LINE_PID_KI;
    sys_data.line_pid.kd = LINE_PID_KD;
    sys_data.line_pid.integral = 0;
    sys_data.line_pid.derivative = 0;
    sys_data.line_pid.prev_error = 0;
    sys_data.line_pid.max_output = PID_MAX_OUTPUT;
    sys_data.line_pid.min_output = PID_MIN_OUTPUT;
    
    line_follower_speed = DEFAULT_SPEED;
    line_lost = false;
}

void LineFollower_Execute(SystemData *data) {
    if(!LineFollower_IsLineDetected()) {
        // Line lost - search for line
        if(!line_lost) {
            line_lost = true;
            line_lost_time = millis();
            data->error_code |= ERROR_LINE_LOST;
        }
        
        LineFollower_SearchLine();
        return;
    }
    
    // Line detected - follow it
    line_lost = false;
    data->error_code &= ~ERROR_LINE_LOST;
    
    // Calculate error from line position
    float error = LineFollower_CalculateError();
    
    // Calculate PID correction
    float correction = PID_Calculate(&data->line_pid, error);
    
    // Apply correction to motor speeds
    int16_t left_speed = line_follower_speed - correction;
    int16_t right_speed = line_follower_speed + correction;
    
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
    data->motors.target_speed = line_follower_speed;
}

void LineFollower_Calibrate(void) {
    Display_ShowMessage("Line Follower", "Calibration");
    
    // Perform calibration routine
    // Move forward slowly while calibrating sensors
    Motor_Forward(100);
    
    for(uint8_t i = 0; i < 50; i++) {
        Sensor_ReadIR(&sys_data.sensors);
        // Update calibration values here if needed
        _delay_ms(100);
    }
    
    Motor_Stop(MOTOR_LEFT);
    Motor_Stop(MOTOR_RIGHT);
    
    Display_ShowMessage("Calibration", "Complete!");
    _delay_ms(1000);
}

bool LineFollower_IsLineDetected(void) {
    // Line is detected if at least one sensor sees it
    uint8_t sensors_on_line = 0;
    
    if(sys_data.sensors.ir_left > LINE_DETECTED_THRESHOLD) sensors_on_line++;
    if(sys_data.sensors.ir_center_left > LINE_DETECTED_THRESHOLD) sensors_on_line++;
    if(sys_data.sensors.ir_center_right > LINE_DETECTED_THRESHOLD) sensors_on_line++;
    if(sys_data.sensors.ir_right > LINE_DETECTED_THRESHOLD) sensors_on_line++;
    
    return sensors_on_line >= 1;
}

float LineFollower_CalculateError(void) {
    // Calculate weighted error based on sensor readings
    // Returns error in range [-100, 100] where negative is left, positive is right
    
    float error = 0.0;
    float total_weight = 0.0;
    
    // Sensor positions: -100 (left), -33, 33, 100 (right)
    float sensor_positions[] = {-100.0, -33.0, 33.0, 100.0};
    uint16_t sensor_values[] = {
        sys_data.sensors.ir_left,
        sys_data.sensors.ir_center_left,
        sys_data.sensors.ir_center_right,
        sys_data.sensors.ir_right
    };
    
    for(uint8_t i = 0; i < 4; i++) {
        if(sensor_values[i] > LINE_DETECTED_THRESHOLD) {
            // Normalize sensor value
            float normalized = (sensor_values[i] - LINE_DETECTED_THRESHOLD) / 
                              (1023.0 - LINE_DETECTED_THRESHOLD);
            
            // Add weighted contribution
            error += sensor_positions[i] * normalized;
            total_weight += normalized;
        }
    }
    
    if(total_weight > 0) {
        error /= total_weight; // Average weighted error
    } else {
        error = 0; // No line detected
    }
    
    return error;
}

void LineFollower_SearchLine(void) {
    static bool search_right = true;
    static uint32_t search_start_time = 0;
    
    if(millis() - line_lost_time > 5000) {
        // Give up after 5 seconds
        Motor_Stop(MOTOR_LEFT);
        Motor_Stop(MOTOR_RIGHT);
        return;
    }
    
    // Alternate search direction
    if(millis() - search_start_time > 1000) {
        search_right = !search_right;
        search_start_time = millis();
    }
    
    if(search_right) {
        Motor_RotateRight(150);
    } else {
        Motor_RotateLeft(150);
    }
}

void LineFollower_UpdatePID(PIDController *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0; // Reset integral on update
    pid->prev_error = 0;
}

void LineFollower_SetSpeed(uint8_t base_speed) {
    if(base_speed > MAX_MOTOR_SPEED) {
        line_follower_speed = MAX_MOTOR_SPEED;
    } else if(base_speed < 50) {
        line_follower_speed = 50; // Minimum speed
    } else {
        line_follower_speed = base_speed;
    }
}

// PID calculation function
float PID_Calculate(PIDController *pid, float error) {
    pid->integral += error;
    
    // Anti-windup
    if(pid->integral > pid->max_output) {
        pid->integral = pid->max_output;
    } else if(pid->integral < pid->min_output) {
        pid->integral = pid->min_output;
    }
    
    pid->derivative = error - pid->prev_error;
    
    float output = (pid->kp * error) + 
                   (pid->ki * pid->integral) + 
                   (pid->kd * pid->derivative);
    
    // Limit output
    if(output > pid->max_output) {
        output = pid->max_output;
    } else if(output < pid->min_output) {
        output = pid->min_output;
    }
    
    pid->prev_error = error;
    return output;
}
