/*
 * Motor_Control.h
 *
 *  Created on: May 20, 2021
 *      Author: Nathan Weiss
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_
#endif /* INC_MOTOR_CONTROL_H_ */

#include "stm32f3xx_hal.h"

#define FORWARD 1
#define BACKWARD 0
#define RAD 0.9974556675 // 1 encoder count = 57.15 degrees of rotation
#define Pulley 15.5
#define Radius 100
#define PPR 1000

void Set_Motor_Parameters(uint8_t DIR);
float Read_Encoder_Top(void);
float Read_Encoder_Left(void);
float Read_Encoder_Right(void);
void Kill_Motors(void);
