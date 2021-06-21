/*
 * Motor_Control.c
 *
 *  Created on: May 20, 2021
 *      Author: Nathan Weiss
 */

#include "Motor_Control.h"
#include "math.h"

void Set_Motor_Parameters(uint8_t DIR) // SET MOTOR PARAMETERS --> NEED TO ADD PWM
{
	if (DIR == FORWARD)
	{
		// Left Motor
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,1); // dir1_L = PC9
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0); // dir2_L = PC8

		// Right Motor
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0); // dir1_R = PC7
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,1); // dir2_R = PC6
	}

	else if(DIR == BACKWARD)
	{
		// Left Motor
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0); // dir1_L = PC9
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,1); // dir2_L = PC8

		// Right Motor
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,1); // dir1_R = PC7
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,0); // dir2_R = PC6
	}
}

float Read_Encoder_Top(void)
{
	uint32_t myEnc_top;
	int16_t count1 = 0;
	float current_pos_top;

	// TOP ENCODER
	myEnc_top = (TIM1 -> CNT);
	count1 = (int16_t) myEnc_top/4;
	current_pos_top = count1*((2*M_PI)/PPR)*RAD*Pulley;

	return current_pos_top;
}

float Read_Encoder_Left(void)
{
	uint32_t myEnc_Left;
	float current_pos_left;
	int16_t count = 0;

	// LEFT MOTOR ENCODER
	myEnc_Left = (TIM2 -> CNT);
	count = (int16_t) myEnc_Left/4;
	current_pos_left =  count*((2*M_PI)/PPR)*RAD*Radius;

	return current_pos_left;
}

float Read_Encoder_Right(void)
{
	uint32_t myEnc_Right;
	int16_t count2 = 0;
	float current_pos_right;

	// RIGHT MOTOR ENCODER
	myEnc_Right = (TIM3 -> CNT);
	count2 = (int16_t) myEnc_Right/4;
	current_pos_right = count2*((2*M_PI)/PPR)*RAD*Radius;

	return current_pos_right;
}

void Kill_Motors(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
}

