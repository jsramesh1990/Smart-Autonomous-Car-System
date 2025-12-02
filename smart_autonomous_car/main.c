/**
 * smart_autonomous_car/src/main.c
 * 
 * Main system controller for Smart Autonomous Car
 * Combines: LED control, 7-segment, LCD, motor control,
 * IR sensors, ultrasonic, line follower, and wall follower
 */

#include "config.h"
#include "peripherals/led_controller.h"
#include "peripherals/display_manager.h"
#include "peripherals/motor_driver.h"
#include "peripherals/sensor_suite.h"
#include "logic/line_follower.h"
#include "logic/wall_follower.h"
#include "logic/obstacle_avoider.h"
#include "logic/state_machine.h"
#include "utils/timer.h"
#include "utils/debug.h"

// Global system variables
SystemState system_state = STATE_IDLE;
CarMode current_mode = MODE_MANUAL;
SystemData sys_data;

// Function prototypes
void System_Init(void);
void Process_User_Input(void);
void Update_Display(void);
void Update_LED_Status(void);
void Execute_Current_Mode(void);
void Emergency_Stop(void);
void System_Diagnostics(void);

int main(void) {
    System_Init();
    
    DEBUG_PRINT("Smart Autonomous Car System Started\r\n");
    Display_Message("SYSTEM READY", "Mode: MANUAL");
    
    // Main system loop
    while(1) {
        // 1. Read all sensors
        Sensor_ReadAll(&sys_data.sensors);
        
        // 2. Process user inputs (buttons, switches)
        Process_User_Input();
        
        // 3. Check for emergency conditions
        if(sys_data.sensors.obstacle_distance < 5.0) {
            Emergency_Stop();
            continue;
        }
        
        // 4. Execute current operation mode
        Execute_Current_Mode();
        
        // 5. Update displays and indicators
        Update_Display();
        Update_LED_Status();
        
        // 6. System diagnostics (every 100 cycles)
        static uint32_t diag_counter = 0;
        if(++diag_counter >= 100) {
            System_Diagnostics();
            diag_counter = 0;
        }
        
        // 7. Small delay for stability
        delay_ms(10);
    }
    
    return 0;
}

/**
 * Initialize all system components
 */
void System_Init(void) {
    // Initialize all peripherals
    LED_Init();
    SevenSegment_Init();
    LCD_Init();
    Motor_Init();
    IR_Sensors_Init();
    Ultrasonic_Init();
    
    // Initialize logic modules
    LineFollower_Init();
    WallFollower_Init();
    ObstacleAvoider_Init();
    
    // Initialize system state
    system_state = STATE_IDLE;
    current_mode = MODE_MANUAL;
    
    // Set initial motor speed
    sys_data.motor_speed_left = 0;
    sys_data.motor_speed_right = 0;
    
    // Initialize PID controllers
    sys_data.line_pid.kp = 1.5;
    sys_data.line_pid.ki = 0.1;
    sys_data.line_pid.kd = 0.5;
    
    sys_data.wall_pid.kp = 2.0;
    sys_data.wall_pid.ki = 0.05;
    sys_data.wall_pid.kd = 0.3;
    
    DEBUG_PRINT("System Initialization Complete\r\n");
}

/**
 * Process user inputs from buttons/switches
 */
void Process_User_Input(void) {
    static uint8_t last_button_state = 0;
    uint8_t current_button_state = Read_Buttons();
    
    // Mode selection button
    if((current_button_state & MODE_BUTTON) && !(last_button_state & MODE_BUTTON)) {
        current_mode = (current_mode + 1) % NUM_MODES;
        system_state = STATE_ACTIVE;
        
        switch(current_mode) {
            case MODE_MANUAL:
                Display_Message("MODE: MANUAL", "Use Joystick");
                break;
            case MODE_LINE_FOLLOW:
                Display_Message("MODE: LINE", "Following");
                break;
            case MODE_WALL_FOLLOW:
                Display_Message("MODE: WALL", "Following");
                break;
            case MODE_OBSTACLE_AVOID:
                Display_Message("MODE: AVOID", "Obstacles");
                break;
            case MODE_AUTO:
                Display_Message("MODE: AUTO", "Intelligent");
                break;
        }
    }
    
    // Emergency stop button
    if(current_button_state & STOP_BUTTON) {
        Emergency_Stop();
    }
    
    // Speed adjustment
    if(current_button_state & SPEED_UP_BUTTON) {
        sys_data.max_speed = MIN(sys_data.max_speed + 10, 255);
    }
    if(current_button_state & SPEED_DOWN_BUTTON) {
        sys_data.max_speed = MAX(sys_data.max_speed - 10, 100);
    }
    
    last_button_state = current_button_state;
}

/**
 * Execute behavior based on current mode
 */
void Execute_Current_Mode(void) {
    switch(current_mode) {
        case MODE_MANUAL:
            // Manual control via joystick
            sys_data.motor_speed_left = Read_Joystick_Y();
            sys_data.motor_speed_right = Read_Joystick_X();
            Motor_SetSpeed(MOTOR_LEFT, sys_data.motor_speed_left);
            Motor_SetSpeed(MOTOR_RIGHT, sys_data.motor_speed_right);
            break;
            
        case MODE_LINE_FOLLOW:
            LineFollower_Execute(&sys_data);
            break;
            
        case MODE_WALL_FOLLOW:
            WallFollower_Execute(&sys_data);
            break;
            
        case MODE_OBSTACLE_AVOID:
            ObstacleAvoider_Execute(&sys_data);
            break;
            
        case MODE_AUTO:
            // Intelligent mode combining all behaviors
            Execute_Auto_Mode();
            break;
    }
}

/**
 * Intelligent autonomous mode combining all behaviors
 */
void Execute_Auto_Mode(void) {
    // Priority 1: Obstacle avoidance
    if(sys_data.sensors.obstacle_distance < 15.0) {
        ObstacleAvoider_Execute(&sys_data);
        Display_Message("AVOIDING", "Obstacle");
        return;
    }
    
    // Priority 2: Line following (if line detected)
    if(LineFollower_IsLineDetected(&sys_data.sensors)) {
        LineFollower_Execute(&sys_data);
        Display_Message("FOLLOWING", "Line");
        return;
    }
    
    // Priority 3: Wall following (if wall detected)
    if(sys_data.sensors.wall_distance < 30.0 && sys_data.sensors.wall_distance > 5.0) {
        WallFollower_Execute(&sys_data);
        Display_Message("FOLLOWING", "Wall");
        return;
    }
    
    // Default: Explore mode
    sys_data.motor_speed_left = sys_data.max_speed;
    sys_data.motor_speed_right = sys_data.max_speed * 0.9; // Slight turn
    Motor_SetSpeed(MOTOR_LEFT, sys_data.motor_speed_left);
    Motor_SetSpeed(MOTOR_RIGHT, sys_data.motor_speed_right);
    Display_Message("EXPLORING", "No guidance");
}

/**
 * Update all displays with current system information
 */
void Update_Display(void) {
    char buffer[17];
    
    // LCD Display (16x2)
    switch(display_page) {
        case 0:
            // Page 1: Mode and speed
            sprintf(buffer, "M:%d Spd:%03d", current_mode, sys_data.max_speed);
            LCD_SetCursor(0, 0);
            LCD_WriteString(buffer);
            
            sprintf(buffer, "L:%03d R:%03d", 
                    sys_data.motor_speed_left, 
                    sys_data.motor_speed_right);
            LCD_SetCursor(0, 1);
            LCD_WriteString(buffer);
            break;
            
        case 1:
            // Page 2: Sensor readings
            sprintf(buffer, "IR:%d%d%d%d", 
                    sys_data.sensors.ir_left,
                    sys_data.sensors.ir_center_left,
                    sys_data.sensors.ir_center_right,
                    sys_data.sensors.ir_right);
            LCD_SetCursor(0, 0);
            LCD_WriteString(buffer);
            
            sprintf(buffer, "US:%03dcm WL:%03d", 
                    (int)sys_data.sensors.obstacle_distance,
                    (int)sys_data.sensors.wall_distance);
            LCD_SetCursor(0, 1);
            LCD_WriteString(buffer);
            break;
            
        case 2:
            // Page 3: System status
            sprintf(buffer, "State:%d Bat:%03d", 
                    system_state,
                    Read_Battery_Level());
            LCD_SetCursor(0, 0);
            LCD_WriteString(buffer);
            
            sprintf(buffer, "Err:%04x", sys_data.error_code);
            LCD_SetCursor(0, 1);
            LCD_WriteString(buffer);
            break;
    }
    
    // 7-Segment Display: Show distance to nearest obstacle
    int distance = (int)sys_data.sensors.obstacle_distance;
    SevenSegment_DisplayNumber(distance);
    
    // Update display page every 2 seconds
    static uint32_t display_timer = 0;
    if(Get_Timer() - display_timer > 2000) {
        display_page = (display_page + 1) % 3;
        display_timer = Get_Timer();
    }
}

/**
 * Update LED indicators based on system status
 */
void Update_LED_Status(void) {
    // Pattern 1: Mode indicator (RGB LED)
    switch(current_mode) {
        case MODE_MANUAL:
            LED_SetColor(LED_BLUE);
            break;
        case MODE_LINE_FOLLOW:
            LED_SetColor(LED_GREEN);
            break;
        case MODE_WALL_FOLLOW:
            LED_SetColor(LED_YELLOW);
            break;
        case MODE_OBSTACLE_AVOID:
            LED_SetColor(LED_ORANGE);
            break;
        case MODE_AUTO:
            LED_SetColor(LED_PURPLE);
            break;
    }
    
    // Pattern 2: Obstacle warning (flashing red)
    if(sys_data.sensors.obstacle_distance < 10.0) {
        static uint8_t flash_state = 0;
        if(flash_state) {
            LED_SetColor(LED_RED);
        }
        flash_state = !flash_state;
    }
    
    // Pattern 3: Line detection indicators (individual LEDs)
    LED_Set(LED1, sys_data.sensors.ir_left);
    LED_Set(LED2, sys_data.sensors.ir_center_left);
    LED_Set(LED3, sys_data.sensors.ir_center_right);
    LED_Set(LED4, sys_data.sensors.ir_right);
    
    // Pattern 4: Motor status LEDs
    LED_Set(LED5, sys_data.motor_speed_left > 0);
    LED_Set(LED6, sys_data.motor_speed_right > 0);
    
    // Pattern 5: Error indicator
    if(sys_data.error_code != 0) {
        LED_Blink(LED_ERROR, 100, 100);
    }
}

/**
 * Emergency stop procedure
 */
void Emergency_Stop(void) {
    system_state = STATE_EMERGENCY;
    
    // Stop motors immediately
    Motor_SetSpeed(MOTOR_LEFT, 0);
    Motor_SetSpeed(MOTOR_RIGHT, 0);
    Motor_Brake();
    
    // Visual and audible alarm
    LED_SetColor(LED_RED);
    LED_Blink(LED_ERROR, 50, 50);
    Buzzer_On();
    
    Display_Message("EMERGENCY STOP", "Check Obstacle");
    
    // Wait for reset
    while(!(Read_Buttons() & RESET_BUTTON)) {
        delay_ms(10);
    }
    
    Buzzer_Off();
    system_state = STATE_IDLE;
    current_mode = MODE_MANUAL;
}

/**
 * System diagnostics and error checking
 */
void System_Diagnostics(void) {
    sys_data.error_code = 0;
    
    // Check sensor readings
    if(sys_data.sensors.obstacle_distance > 400.0) {
        sys_data.error_code |= ERROR_ULTRASONIC;
    }
    
    if(sys_data.sensors.ir_left == sys_data.sensors.ir_right && 
       sys_data.sensors.ir_center_left == sys_data.sensors.ir_center_right) {
        sys_data.error_code |= ERROR_IR_SENSORS;
    }
    
    // Check motor currents
    if(Read_Motor_Current(MOTOR_LEFT) > MAX_CURRENT || 
       Read_Motor_Current(MOTOR_RIGHT) > MAX_CURRENT) {
        sys_data.error_code |= ERROR_MOTOR_OVERLOAD;
    }
    
    // Check battery level
    if(Read_Battery_Level() < BATTERY_LOW) {
        sys_data.error_code |= ERROR_LOW_BATTERY;
        Display_Message("LOW BATTERY", "Please Charge");
    }
    
    // Log errors
    if(sys_data.error_code != 0) {
        DEBUG_PRINT("Error detected: 0x%04X\r\n", sys_data.error_code);
    }
}

/**
 * Interrupt Service Routine for sensors
 */
void Sensor_ISR(void) {
    // Ultrasonic echo detection
    if(ULTRASONIC_ECHO_PIN) {
        uint32_t pulse_width = Measure_Pulse_Width();
        sys_data.sensors.obstacle_distance = pulse_width * 0.034 / 2;
    }
    
    // IR sensor interrupts
    if(IR_LEFT_PIN) {
        sys_data.sensors.ir_left = 1;
    }
    if(IR_RIGHT_PIN) {
        sys_data.sensors.ir_right = 1;
    }
}

/**
 * PID Controller for precise movement
 */
float PID_Calculate(PID_Controller* pid, float error) {
    pid->integral += error;
    pid->derivative = error - pid->prev_error;
    pid->prev_error = error;
    
    float output = (pid->kp * error) + 
                   (pid->ki * pid->integral) + 
                   (pid->kd * pid->derivative);
    
    // Anti-windup
    if(output > pid->max_output) {
        output = pid->max_output;
        pid->integral -= error; // Prevent integral windup
    } else if(output < pid->min_output) {
        output = pid->min_output;
        pid->integral -= error;
    }
    
    return output;
}
