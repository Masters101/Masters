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
#define count 2.0904 // this counter value is not correct
#define PPR 1000

void Set_Motor_Parameters(uint8_t DIR);
float Read_Encoder_Values(void);
void Kill_Motors(void);
