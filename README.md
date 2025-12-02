# Smart Autonomous Car System

## Overview
A comprehensive embedded system integrating multiple modules to create an intelligent autonomous vehicle capable of:
- Line following with PID control
- Wall following with ultrasonic sensors
- Obstacle detection and avoidance
- Multi-mode operation
- Real-time status display
- Visual and auditory feedback

## Features
1. **5 Operation Modes:**
   - Manual Control
   - Line Following
   - Wall Following
   - Obstacle Avoidance
   - Intelligent Auto Mode

2. **Sensor Suite:**
   - 4x IR sensors for line detection
   - 2x Ultrasonic sensors (obstacle & wall)
   - Battery voltage monitoring
   - Motor current sensing

3. **Display System:**
   - 16x2 LCD for detailed information
   - 7-segment display for distance
   - 8x LEDs for status indication
   - RGB LED for mode indication

4. **Control System:**
   - Dual PID controllers
   - State machine architecture
   - Emergency stop system
   - Error detection and handling

## Hardware Requirements
- ATmega328P microcontroller
- L298N Motor Driver
- HC-SR04 Ultrasonic sensors
- TCRT5000 IR sensors
- 16x2 LCD Display
- 4-digit 7-segment display
- RGB LED and indicator LEDs
- Push buttons for control
- 12V DC motors with encoders
- 12V Battery with voltage regulator

## Setup Instructions
1. Clone the repository
2. Update pin configurations in `config.h`
3. Build the project: `make all`
4. Flash to microcontroller: `make flash`
5. Connect all hardware according to schematics
6. Power on and select mode

## Testing
```bash
# Run unit tests
make test

# Check system diagnostics
# Press MODE button to cycle through modes
# Press STOP button for emergency stop
