/**
 * smart_autonomous_car/src/config.h
 * 
 * System-wide configuration and pin definitions
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stdbool.h>

// Clock Configuration
#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define UART_BAUD ((F_CPU)/(BAUD_RATE*16UL)-1)

// Pin Definitions
// LED Controller
#define LED_PORT PORTB
#define LED_DDR DDRB
#define LED1_PIN PB0
#define LED2_PIN PB1
#define LED3_PIN PB2
#define LED4_PIN PB3
#define RGB_RED_PIN PB4
#define RGB_GREEN_PIN PB5
#define RGB_BLUE_PIN PB6

// Motor Driver
#define MOTOR_PORT PORTC
#define MOTOR_DDR DDRC
#define MOTOR_LEFT_EN PC0  // PWM
#define MOTOR_LEFT_IN1 PC1
#define MOTOR_LEFT_IN2 PC2
#define MOTOR_RIGHT_EN PC3 // PWM
#define MOTOR_RIGHT_IN1 PC4
#define MOTOR_RIGHT_IN2 PC5

// Sensor Suite
#define SENSOR_PORT PORTD
#define SENSOR_DDR DDRD
#define SENSOR_PIN PIND
#define ULTRASONIC_TRIG PD0
#define ULTRASONIC_ECHO PD1
#define IR_LEFT_PIN PD2
#define IR_CENTER_LEFT_PIN PD3
#define IR_CENTER_RIGHT_PIN PD4
#define IR_RIGHT_PIN PD5

// Display Manager
#define LCD_RS_PIN PD6
#define LCD_RW_PIN PD7
#define LCD_EN_PIN PB0
#define LCD_DATA_PORT PORTA
#define LCD_DATA_DDR DDRA
#define LCD_DATA_PIN PINA

#define SEVEN_SEG_PORT PORTC
#define SEVEN_SEG_DDR DDRC
#define SEVEN_SEG_EN1 PC6
#define SEVEN_SEG_EN2 PC7

// Button Inputs
#define BUTTON_PORT PORTB
#define BUTTON_DDR DDRB
#define BUTTON_PIN PINB
#define MODE_BUTTON PB2
#define STOP_BUTTON PB3
#define SPEED_UP_BUTTON PB4
#define SPEED_DOWN_BUTTON PB5

// System Constants
#define MAX_MOTOR_SPEED 255
#define MIN_MOTOR_SPEED 0
#define DEFAULT_SPEED 150
#define BATTERY_LOW 3300  // 3.3V in mV
#define BATTERY_CRITICAL 3000
#define MAX_CURRENT 2000  // 2A in mA
#define ULTRASONIC_TIMEOUT 30000  // 30ms timeout

// PID Controller Parameters
#define LINE_PID_KP 1.5f
#define LINE_PID_KI 0.1f
#define LINE_PID_KD 0.5f
#define WALL_PID_KP 2.0f
#define WALL_PID_KI 0.05f
#define WALL_PID_KD 0.3f
#define PID_MAX_OUTPUT 100.0f
#define PID_MIN_OUTPUT -100.0f

// Sensor Thresholds
#define OBSTACLE_THRESHOLD 15.0f  // cm
#define WALL_FOLLOW_DISTANCE 20.0f // cm
#define LINE_DETECTED_THRESHOLD 500  // Analog value

// Timing Constants
#define DEBOUNCE_DELAY 50     // ms
#define SENSOR_READ_INTERVAL 100 // ms
#define DISPLAY_UPDATE_INTERVAL 1000 // ms
#define DIAGNOSTIC_INTERVAL 5000 // ms

// Error Codes
typedef enum {
    ERROR_NONE = 0x0000,
    ERROR_ULTRASONIC_TIMEOUT = 0x0001,
    ERROR_IR_SENSOR_FAIL = 0x0002,
    ERROR_MOTOR_OVERLOAD_LEFT = 0x0004,
    ERROR_MOTOR_OVERLOAD_RIGHT = 0x0008,
    ERROR_LOW_BATTERY = 0x0010,
    ERROR_HIGH_BATTERY = 0x0020,
    ERROR_LINE_LOST = 0x0040,
    ERROR_WALL_LOST = 0x0080,
    ERROR_SYSTEM_OVERHEAT = 0x0100,
    ERROR_COMMUNICATION = 0x0200
} ErrorCode;

// System States
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_EMERGENCY_STOP,
    STATE_ERROR,
    STATE_CALIBRATING,
    STATE_SLEEP
} SystemState;

// Operation Modes
typedef enum {
    MODE_MANUAL,
    MODE_LINE_FOLLOW,
    MODE_WALL_FOLLOW_LEFT,
    MODE_WALL_FOLLOW_RIGHT,
    MODE_OBSTACLE_AVOID,
    MODE_AUTO_PILOT,
    MODE_CALIBRATION,
    NUM_MODES
} OperationMode;

// LED Colors
typedef enum {
    LED_OFF,
    LED_RED,
    LED_GREEN,
    LED_BLUE,
    LED_YELLOW,
    LED_CYAN,
    LED_MAGENTA,
    LED_WHITE
} LEDColor;

// Motor Directions
typedef enum {
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_STOP,
    DIR_ROTATE_LEFT,
    DIR_ROTATE_RIGHT
} MotorDirection;

// PID Controller Structure
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float derivative;
    float prev_error;
    float max_output;
    float min_output;
} PIDController;

// Sensor Data Structure
typedef struct {
    uint16_t ir_left;
    uint16_t ir_center_left;
    uint16_t ir_center_right;
    uint16_t ir_right;
    float obstacle_distance;  // cm
    float wall_distance;      // cm
    float battery_voltage;    // mV
    int16_t temperature;      // °C
    uint16_t motor_current_left;  // mA
    uint16_t motor_current_right; // mA
} SensorData;

// Motor Control Structure
typedef struct {
    uint8_t left_speed;
    uint8_t right_speed;
    MotorDirection left_dir;
    MotorDirection right_dir;
    uint8_t target_speed;
} MotorControl;

// System Data Structure
typedef struct {
    SystemState state;
    OperationMode mode;
    SensorData sensors;
    MotorControl motors;
    PIDController line_pid;
    PIDController wall_pid;
    uint16_t error_code;
    uint32_t runtime;         // seconds
    uint8_t display_page;
    bool emergency_stop;
    uint8_t battery_percent;
} SystemData;

// Global system data
extern SystemData sys_data;

// Function prototypes
void delay_ms(uint16_t ms);
void delay_us(uint16_t us);
uint32_t millis(void);

#endif // CONFIG_H
