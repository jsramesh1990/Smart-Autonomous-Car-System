#ifndef CONFIG_H
#define CONFIG_H

// System Configuration
#define F_CPU 16000000UL
#define BAUD_RATE 9600

// Pin Definitions
// LEDs
#define LED1_PIN        PD0
#define LED2_PIN        PD1
#define LED3_PIN        PD2
#define LED4_PIN        PD3
#define RGB_RED_PIN     PB0
#define RGB_GREEN_PIN   PB1
#define RGB_BLUE_PIN    PB2

// Motors
#define MOTOR_LEFT_EN   PB3
#define MOTOR_LEFT_IN1  PB4
#define MOTOR_LEFT_IN2  PB5
#define MOTOR_RIGHT_EN  PB6
#define MOTOR_RIGHT_IN1 PB7
#define MOTOR_RIGHT_IN2 PC0

// Sensors
#define ULTRASONIC_TRIG PC1
#define ULTRASONIC_ECHO PC2
#define IR_LEFT_PIN     PC3
#define IR_CENTER_LEFT_PIN PC4
#define IR_CENTER_RIGHT_PIN PC5
#define IR_RIGHT_PIN    PC6

// Display
#define LCD_RS_PIN      PD4
#define LCD_RW_PIN      PD5
#define LCD_EN_PIN      PD6
#define LCD_DATA_PORT   PORTD
#define SEVEN_SEG_PORT  PORTA

// Buttons
#define MODE_BUTTON     PD7
#define STOP_BUTTON     PC7
#define RESET_BUTTON    PB0

// System Constants
#define MAX_SPEED       255
#define MIN_SPEED       0
#define BATTERY_LOW     3300  // 3.3V
#define MAX_CURRENT     2000  // 2A

// Enumerations
typedef enum {
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_EMERGENCY,
    STATE_ERROR
} SystemState;

typedef enum {
    MODE_MANUAL,
    MODE_LINE_FOLLOW,
    MODE_WALL_FOLLOW,
    MODE_OBSTACLE_AVOID,
    MODE_AUTO,
    NUM_MODES
} CarMode;

typedef enum {
    ERROR_NONE = 0,
    ERROR_ULTRASONIC = 1,
    ERROR_IR_SENSORS = 2,
    ERROR_MOTOR_OVERLOAD = 4,
    ERROR_LOW_BATTERY = 8,
    ERROR_LINE_LOST = 16
} ErrorCode;

// Structures
typedef struct {
    float kp, ki, kd;
    float integral;
    float derivative;
    float prev_error;
    float max_output;
    float min_output;
} PID_Controller;

typedef struct {
    uint8_t ir_left;
    uint8_t ir_center_left;
    uint8_t ir_center_right;
    uint8_t ir_right;
    float obstacle_distance;
    float wall_distance;
    float battery_voltage;
} SensorData;

typedef struct {
    SystemState state;
    CarMode mode;
    SensorData sensors;
    uint16_t motor_speed_left;
    uint16_t motor_speed_right;
    uint16_t max_speed;
    PID_Controller line_pid;
    PID_Controller wall_pid;
    uint16_t error_code;
} SystemData;

// Function prototypes
void delay_ms(uint16_t ms);
void System_Init(void);

#endif // CONFIG_H
