# Smart Autonomous Car - System Design

## Overview
The Smart Autonomous Car is an embedded system that integrates multiple sensors and actuators to create an intelligent vehicle capable of autonomous navigation.

## System Architecture

### Hardware Components
1. **Microcontroller:** ATmega328P
2. **Sensors:**
   - 4x IR Sensors (TCRT5000)
   - 2x Ultrasonic Sensors (HC-SR04)
   - Battery Voltage Sensor
3. **Actuators:**
   - 2x DC Motors with L298N Driver
   - 4x Status LEDs
   - RGB LED
4. **Displays:**
   - 16x2 LCD Display
   - 4-digit 7-Segment Display
5. **Inputs:**
   - 4x Push Buttons
   - Mode Selection Switch

### Software Architecture
