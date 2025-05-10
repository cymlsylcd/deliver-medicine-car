#ifndef __TRACK_H
#define __TRACK_H


#include "main.h"
#include <math.h>
#include <stdint.h> // Include this header for uint8_t definition

extern uint8_t Sensor_Left, Sensor_MLeft, Sensor_MMLeft, Sensor_MMMLeft;
extern uint8_t Sensor_MMMRight, Sensor_MMRight, Sensor_MRight, Sensor_Right;
extern float Target_Velocity;
void Get_Sensor_Value(void);
void Init_Sensor_Pins(void);
int  track(void);

#define SENSOR1_PIN l1_Pin 
#define SENSOR1_PORT GPIOD

#define SENSOR2_PIN l2_Pin 
#define SENSOR2_PORT GPIOD

#define SENSOR3_PIN l3_Pin  
#define SENSOR3_PORT GPIOD

#define SENSOR4_PIN l4_Pin  
#define SENSOR4_PORT GPIOD


#define SENSOR5_PIN r4_Pin  // 中右传感器
#define SENSOR5_PORT GPIOB

#define SENSOR6_PIN r3_Pin  // 中右传感器
#define SENSOR6_PORT GPIOB

#define SENSOR7_PIN r2_Pin  // 中右传感器
#define SENSOR7_PORT GPIOB

#define SENSOR8_PIN r1_Pin  // 右传感器
#define SENSOR8_PORT GPIOB


#endif
