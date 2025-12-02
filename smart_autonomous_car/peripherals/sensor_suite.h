/**
 * smart_autonomous_car/src/peripherals/sensor_suite.h
 * 
 * Unified sensor interface for IR and ultrasonic sensors
 */

#ifndef SENSOR_SUITE_H
#define SENSOR_SUITE_H

#include "../config.h"

// Sensor Functions
void Sensor_Init(void);
void Sensor_ReadAll(SensorData *data);
void Sensor_ReadIR(SensorData *data);
void Sensor_ReadUltrasonic(SensorData *data);
void Sensor_ReadBattery(SensorData *data);
void Sensor_ReadTemperature(SensorData *data);
float Sensor_ReadDistance(uint8_t sensor_num);
uint16_t Sensor_ReadIRRaw(uint8_t sensor_num);
bool Sensor_DetectLine(void);
bool Sensor_DetectObstacle(float threshold);
bool Sensor_DetectWall(float *distance);
void Sensor_Calibrate(void);
void Sensor_Diagnostics(void);

#endif // SENSOR_SUITE_H
