/**
 * smart_autonomous_car/src/logic/state_machine.c
 * 
 * State machine implementation
 */

#include "state_machine.h"
#include "../peripherals/display_manager.h"
#include "../peripherals/led_controller.h"
#include <string.h>

// State machine variables
static SystemState current_state = STATE_INIT;
static SystemState previous_state = STATE_INIT;
static uint32_t state_entry_time = 0;

void StateMachine_Init(void) {
    current_state = STATE_INIT;
    previous_state = STATE_INIT;
    state_entry_time = millis();
    sys_data.state = STATE_INIT;
}

void StateMachine_Update(void) {
    SystemEvent event = CheckForEvents();
    
    if(event != EVENT_NONE) {
        StateMachine_HandleEvent(event);
    }
    
    // State-specific behavior
    switch(current_state) {
        case STATE_INIT:
            StateMachine_InitState();
            break;
            
        case STATE_IDLE:
            StateMachine_IdleState();
            break;
            
        case STATE_ACTIVE:
            StateMachine_ActiveState();
            break;
            
        case STATE_EMERGENCY_STOP:
            StateMachine_EmergencyState();
            break;
            
        case STATE_ERROR:
            StateMachine_ErrorState();
            break;
            
        case STATE_CALIBRATING:
            StateMachine_CalibratingState();
            break;
            
        case STATE_SLEEP:
            StateMachine_SleepState();
            break;
            
        default:
            break;
    }
    
    // Update system data
    sys_data.state = current_state;
}

void StateMachine_Transition(SystemState new_state) {
    // Exit current state
    switch(current_state) {
        case STATE_ACTIVE:
            // Stop motors when leaving active state
            Motor_Stop(MOTOR_LEFT);
            Motor_Stop(MOTOR_RIGHT);
            break;
            
        default:
            break;
    }
    
    // Record transition
    previous_state = current_state;
    current_state = new_state;
    state_entry_time = millis();
    
    // Log transition (for debugging)
    char log_msg[32];
    sprintf(log_msg, "State: %d -> %d", previous_state, current_state);
    DEBUG_PRINT(log_msg);
}

void StateMachine_HandleEvent(uint8_t event) {
    switch(current_state) {
        case STATE_INIT:
            if(event == EVENT_CALIBRATION_COMPLETE) {
                StateMachine_Transition(STATE_IDLE);
            }
            break;
            
        case STATE_IDLE:
            if(event == EVENT_BUTTON_PRESS) {
                StateMachine_Transition(STATE_ACTIVE);
            } else if(event == EVENT_LOW_BATTERY) {
                StateMachine_Transition(STATE_ERROR);
            }
            break;
            
        case STATE_ACTIVE:
            if(event == EVENT_EMERGENCY_STOP) {
                StateMachine_Transition(STATE_EMERGENCY_STOP);
            } else if(event == EVENT_OBSTACLE_DETECTED && sys_data.sensors.obstacle_distance < 5.0) {
                StateMachine_Transition(STATE_EMERGENCY_STOP);
            } else if(event == EVENT_LOW_BATTERY) {
                StateMachine_Transition(STATE_ERROR);
            } else if(event == EVENT_BUTTON_PRESS) {
                StateMachine_Transition(STATE_IDLE);
            }
            break;
            
        case STATE_EMERGENCY_STOP:
            if(event == EVENT_BUTTON_PRESS) {
                StateMachine_Transition(STATE_IDLE);
            } else if(event == EVENT_ERROR_CLEARED) {
                StateMachine_Transition(STATE_IDLE);
            }
            break;
            
        case STATE_ERROR:
            if(event == EVENT_ERROR_CLEARED) {
                StateMachine_Transition(STATE_IDLE);
            }
            break;
            
        case STATE_CALIBRATING:
            if(event == EVENT_CALIBRATION_COMPLETE) {
                StateMachine_Transition(STATE_IDLE);
            }
            break;
            
        case STATE_SLEEP:
            if(event == EVENT_BUTTON_PRESS) {
                StateMachine_Transition(STATE_IDLE);
            }
            break;
    }
}

SystemEvent CheckForEvents(void) {
    static uint32_t last_button_check = 0;
    
    // Check button press (debounced)
    if(millis() - last_button_check > DEBOUNCE_DELAY) {
        uint8_t buttons = Read_Buttons();
        if(buttons != 0) {
            last_button_check = millis();
            return EVENT_BUTTON_PRESS;
        }
        last_button_check = millis();
    }
    
    // Check for obstacles
    if(sys_data.sensors.obstacle_distance < 5.0) {
        return EVENT_OBSTACLE_DETECTED;
    }
    
    // Check battery
    if(sys_data.sensors.battery_voltage < BATTERY_LOW) {
        return EVENT_LOW_BATTERY;
    }
    
    // Check line/wall lost
    if(sys_data.error_code & ERROR_LINE_LOST) {
        return EVENT_LINE_LOST;
    }
    
    if(sys_data.error_code & ERROR_WALL_LOST) {
        return EVENT_WALL_LOST;
    }
    
    return EVENT_NONE;
}

void StateMachine_InitState(void) {
    static bool calibration_started = false;
    
    if(!calibration_started) {
        Display_ShowMessage("System Initializing", "Please wait...");
        LED_SetColor(LED_CYAN);
        calibration_started = true;
        
        // Start calibration
        sys_data.mode = MODE_CALIBRATION;
        Sensor_Calibrate();
        LineFollower_Calibrate();
        
        // Simulate calibration completion
        _delay_ms(2000);
        StateMachine_HandleEvent(EVENT_CALIBRATION_COMPLETE);
    }
}

void StateMachine_IdleState(void) {
    static uint32_t last_status_update = 0;
    
    // Update status display periodically
    if(millis() - last_status_update > 5000) {
        Display_ShowMessage("System Ready", "Press button");
        last_status_update = millis();
    }
    
    // Blink LED to indicate idle
    static uint32_t last_blink = 0;
    if(millis() - last_blink > 1000) {
        LED_Toggle(1);
        last_blink = millis();
    }
}

void StateMachine_ActiveState(void) {
    // Active state behavior depends on mode
    // This is handled in main loop
    LED_SetColor(LED_GREEN);
}

void StateMachine_EmergencyState(void) {
    static bool emergency_handled = false;
    
    if(!emergency_handled) {
        Display_ShowMessage("EMERGENCY STOP", "Check obstacle");
        LED_SetColor(LED_RED);
        
        // Stop all motors
        Motor_EmergencyStop();
        
        // Sound alarm (if available)
        // Buzzer_On();
        
        emergency_handled = true;
    }
    
    // Flash LEDs
    static uint32_t last_flash = 0;
    if(millis() - last_flash > 200) {
        LED_Toggle(1);
        LED_Toggle(2);
        LED_Toggle(3);
        LED_Toggle(4);
        last_flash = millis();
    }
    
    // Check if obstacle cleared
    if(sys_data.sensors.obstacle_distance > 20.0) {
        emergency_handled = false;
        StateMachine_HandleEvent(EVENT_ERROR_CLEARED);
    }
}

void StateMachine_ErrorState(void) {
    Display_ShowError(sys_data.error_code);
    LED_ErrorFlash(sys_data.error_code);
    
    // Check if error cleared
    if(sys_data.error_code == 0) {
        StateMachine_HandleEvent(EVENT_ERROR_CLEARED);
    }
}

void StateMachine_CalibratingState(void) {
    static uint8_t cal_step = 0;
    
    switch(cal_step) {
        case 0:
            Display_ShowMessage("Calibrating", "IR Sensors");
            Sensor_Calibrate();
            cal_step++;
            break;
            
        case 1:
            Display_ShowMessage("Calibrating", "Ultrasonic");
            // Ultrasonic calibration
            cal_step++;
            break;
            
        case 2:
            Display_ShowMessage("Calibration", "Complete!");
            _delay_ms(1000);
            StateMachine_HandleEvent(EVENT_CALIBRATION_COMPLETE);
            break;
    }
}

void StateMachine_SleepState(void) {
    // Power saving mode
    Display_ShowMessage("Sleep Mode", "Press to wake");
    LED_SetColor(LED_OFF);
    
    // Disable unnecessary peripherals
    // Reduce clock speed
    // etc.
}

SystemState StateMachine_GetCurrentState(void) {
    return current_state;
}

void StateMachine_PrintStatus(void) {
    char status[64];
    sprintf(status, "State: %d, Mode: %d, Errors: 0x%04X", 
            current_state, sys_data.mode, sys_data.error_code);
    DEBUG_PRINT(status);
}

// Button reading function (simplified)
uint8_t Read_Buttons(void) {
    // In real implementation, read from button pins
    // For simulation, return 0 (no button pressed)
    return 0;
}
