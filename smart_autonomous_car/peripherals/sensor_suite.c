/**
 * smart_autonomous_car/src/peripherals/sensor_suite.c
 * 
 * Sensor interface implementation
 */

#include "sensor_suite.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// Calibration values
static uint16_t ir_calibration_min[4] = {1023, 1023, 1023, 1023};
static uint16_t ir_calibration_max[4] = {0, 0, 0, 0};
static float ultrasonic_calibration = 1.0;

// Ultrasonic timing variables
static volatile uint32_t ultrasonic_start_time = 0;
static volatile uint32_t ultrasonic_end_time = 0;
static volatile bool ultrasonic_measurement_done = false;

// Timer1 for ultrasonic measurement
void Timer1_Init(void) {
    // Timer1 normal mode, prescaler 64
    TCCR1A = 0;
    TCCR1B = (1 << CS11) | (1 << CS10); // Prescaler 64
    TCNT1 = 0;
}

void Sensor_Init(void) {
    // Configure IR sensor pins as input
    SENSOR_DDR &= ~((1 << IR_LEFT_PIN) | (1 << IR_CENTER_LEFT_PIN) | 
                    (1 << IR_CENTER_RIGHT_PIN) | (1 << IR_RIGHT_PIN));
    
    // Configure ultrasonic pins
    SENSOR_DDR |= (1 << ULTRASONIC_TRIG);    // Output
    SENSOR_DDR &= ~(1 << ULTRASONIC_ECHO);   // Input
    
    // Initialize ADC for IR sensors and battery
    // Configure ADC: AVCC reference, right adjust
    ADMUX = (1 << REFS0);
    
    // Enable ADC, prescaler 128 (125kHz at 16MHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // Initialize Timer1 for ultrasonic
    Timer1_Init();
    
    // Enable external interrupt for ultrasonic echo
    EICRA |= (1 << ISC10); // Any logic change on INT1
    EIMSK |= (1 << INT1);  // Enable INT1
    
    sei(); // Enable global interrupts
    
    // Initial calibration
    Sensor_Calibrate();
}

void Sensor_ReadAll(SensorData *data) {
    Sensor_ReadIR(data);
    Sensor_ReadUltrasonic(data);
    Sensor_ReadBattery(data);
    Sensor_ReadTemperature(data);
    
    // Read motor currents
    data->motor_current_left = Motor_ReadCurrent(MOTOR_LEFT);
    data->motor_current_right = Motor_ReadCurrent(MOTOR_RIGHT);
}

void Sensor_ReadIR(SensorData *data) {
    // Read all four IR sensors
    data->ir_left = Sensor_ReadIRRaw(0);
    data->ir_center_left = Sensor_ReadIRRaw(1);
    data->ir_center_right = Sensor_ReadIRRaw(2);
    data->ir_right = Sensor_ReadIRRaw(3);
}

uint16_t Sensor_ReadIRRaw(uint8_t sensor_num) {
    uint8_t mux_channel;
    
    // Select ADC channel based on sensor number
    switch(sensor_num) {
        case 0: mux_channel = 0; break;  // ADC0 for IR left
        case 1: mux_channel = 1; break;  // ADC1 for IR center left
        case 2: mux_channel = 2; break;  // ADC2 for IR center right
        case 3: mux_channel = 3; break;  // ADC3 for IR right
        default: return 0;
    }
    
    // Set ADC channel
    ADMUX = (1 << REFS0) | (mux_channel & 0x0F);
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while(ADCSRA & (1 << ADSC));
    
    // Read result
    uint16_t result = ADC;
    
    // Apply calibration
    if(result < ir_calibration_min[sensor_num]) {
        result = ir_calibration_min[sensor_num];
    }
    if(result > ir_calibration_max[sensor_num]) {
        result = ir_calibration_max[sensor_num];
    }
    
    // Normalize to 0-1023
    result = (result - ir_calibration_min[sensor_num]) * 1023 / 
             (ir_calibration_max[sensor_num] - ir_calibration_min[sensor_num] + 1);
    
    return result;
}

void Sensor_ReadUltrasonic(SensorData *data) {
    data->obstacle_distance = Sensor_ReadDistance(0);  // Front sensor
    data->wall_distance = Sensor_ReadDistance(1);      // Side sensor (simulated)
}

float Sensor_ReadDistance(uint8_t sensor_num) {
    // For simplicity, using one sensor
    // In real implementation, you'd have multiple sensors
    
    // Send trigger pulse
    SENSOR_PORT &= ~(1 << ULTRASONIC_TRIG);
    _delay_us(2);
    SENSOR_PORT |= (1 << ULTRASONIC_TRIG);
    _delay_us(10);
    SENSOR_PORT &= ~(1 << ULTRASONIC_TRIG);
    
    // Wait for echo pulse
    ultrasonic_measurement_done = false;
    uint32_t timeout = 0;
    
    while(!(SENSOR_PIN & (1 << ULTRASONIC_ECHO)) && timeout++ < 1000) {
        _delay_us(1);
    }
    
    if(timeout >= 1000) {
        // Timeout
        sys_data.error_code |= ERROR_ULTRASONIC_TIMEOUT;
        return 400.0; // Maximum distance
    }
    
    // Start timer
    TCNT1 = 0;
    ultrasonic_start_time = TCNT1;
    
    // Wait for echo to go low
    timeout = 0;
    while((SENSOR_PIN & (1 << ULTRASONIC_ECHO)) && timeout++ < 60000) {
        _delay_us(1);
    }
    
    ultrasonic_end_time = TCNT1;
    
    if(timeout >= 60000) {
        // Timeout (object too far or sensor error)
        sys_data.error_code |= ERROR_ULTRASONIC_TIMEOUT;
        return 400.0;
    }
    
    // Calculate distance in cm
    // Timer1 runs at 16MHz/64 = 250kHz, so each tick is 4us
    // Speed of sound is 340m/s = 0.034cm/us
    uint32_t pulse_width = ultrasonic_end_time - ultrasonic_start_time;
    float distance = (pulse_width * 4.0 * 0.034) / 2.0;
    
    // Apply calibration
    distance *= ultrasonic_calibration;
    
    return distance;
}

void Sensor_ReadBattery(SensorData *data) {
    // Read battery voltage from ADC channel 4
    ADMUX = (1 << REFS0) | 0x04; // ADC4
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion
    while(ADCSRA & (1 << ADSC));
    
    // Read ADC value
    uint16_t adc_value = ADC;
    
    // Convert to voltage (assuming voltage divider: 12V -> 5V -> ADC)
    // ADC reference is 5V, 10-bit resolution
    // Voltage divider ratio: R1=10k, R2=4.7k
    float adc_voltage = adc_value * 5.0 / 1023.0;
    data->battery_voltage = adc_voltage * (10000 + 4700) / 4700; // Convert back to battery voltage
    
    // Update battery percentage
    if(data->battery_voltage >= 12600) { // 12.6V = 100%
        sys_data.battery_percent = 100;
    } else if(data->battery_voltage <= 10500) { // 10.5V = 0%
        sys_data.battery_percent = 0;
    } else {
        sys_data.battery_percent = (data->battery_voltage - 10500) * 100 / (12600 - 10500);
    }
    
    // Check for low battery
    if(data->battery_voltage < BATTERY_LOW) {
        sys_data.error_code |= ERROR_LOW_BATTERY;
    }
}

void Sensor_ReadTemperature(SensorData *data) {
    // For simulation, use fixed temperature
    // In real implementation, read from temperature sensor
    data->temperature = 25; // 25°C
}

bool Sensor_DetectLine(void) {
    // Check if any IR sensor detects line
    return (sys_data.sensors.ir_left > LINE_DETECTED_THRESHOLD ||
            sys_data.sensors.ir_center_left > LINE_DETECTED_THRESHOLD ||
            sys_data.sensors.ir_center_right > LINE_DETECTED_THRESHOLD ||
            sys_data.sensors.ir_right > LINE_DETECTED_THRESHOLD);
}

bool Sensor_DetectObstacle(float threshold) {
    return (sys_data.sensors.obstacle_distance < threshold && 
            sys_data.sensors.obstacle_distance > 2.0);
}

bool Sensor_DetectWall(float *distance) {
    // Simulated wall detection using front sensor at an angle
    // In real implementation, you'd have side-facing sensors
    if(sys_data.sensors.obstacle_distance < 30.0 && 
       sys_data.sensors.obstacle_distance > 5.0) {
        if(distance != NULL) {
            *distance = sys_data.sensors.obstacle_distance;
        }
        return true;
    }
    return false;
}

void Sensor_Calibrate(void) {
    Display_ShowMessage("Calibrating", "Sensors...");
    
    // Calibrate IR sensors
    for(uint8_t i = 0; i < 10; i++) {
        for(uint8_t s = 0; s < 4; s++) {
            uint16_t value = Sensor_ReadIRRaw(s);
            
            if(value < ir_calibration_min[s]) {
                ir_calibration_min[s] = value;
            }
            if(value > ir_calibration_max[s]) {
                ir_calibration_max[s] = value;
            }
        }
        _delay_ms(100);
    }
    
    // Calibrate ultrasonic (known distance measurement)
    Display_ShowMessage("Place at 10cm", "from obstacle");
    _delay_ms(2000);
    
    float measured_distance = Sensor_ReadDistance(0);
    if(measured_distance > 5.0 && measured_distance < 20.0) {
        ultrasonic_calibration = 10.0 / measured_distance;
    }
    
    Display_ShowMessage("Calibration", "Complete!");
    _delay_ms(1000);
}

void Sensor_Diagnostics(void) {
    // Check IR sensors
    for(uint8_t i = 0; i < 4; i++) {
        uint16_t value = Sensor_ReadIRRaw(i);
        if(value < 100 || value > 900) {
            sys_data.error_code |= ERROR_IR_SENSOR_FAIL;
        }
    }
    
    // Check ultrasonic
    float distance = Sensor_ReadDistance(0);
    if(distance > 400.0 || distance < 0) {
        sys_data.error_code |= ERROR_ULTRASONIC_TIMEOUT;
    }
}

// Ultrasonic echo interrupt handler
ISR(INT1_vect) {
    if(SENSOR_PIN & (1 << ULTRASONIC_ECHO)) {
        // Rising edge - start timing
        TCNT1 = 0;
    } else {
        // Falling edge - stop timing
        ultrasonic_end_time = TCNT1;
        ultrasonic_measurement_done = true;
    }
}
