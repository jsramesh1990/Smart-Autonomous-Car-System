/**
 * smart_autonomous_car/src/logic/state_machine.h
 * 
 * System state machine controller
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "../config.h"

// State Machine Functions
void StateMachine_Init(void);
void StateMachine_Update(void);
void StateMachine_Transition(SystemState new_state);
void StateMachine_HandleEvent(uint8_t event);
SystemState StateMachine_GetCurrentState(void);
void StateMachine_PrintStatus(void);

// State Machine Events
typedef enum {
    EVENT_NONE,
    EVENT_BUTTON_PRESS,
    EVENT_OBSTACLE_DETECTED,
    EVENT_LINE_LOST,
    EVENT_WALL_LOST,
    EVENT_LOW_BATTERY,
    EVENT_EMERGENCY_STOP,
    EVENT_CALIBRATION_COMPLETE,
    EVENT_ERROR_CLEARED,
    EVENT_MODE_CHANGE
} SystemEvent;

#endif // STATE_MACHINE_H
