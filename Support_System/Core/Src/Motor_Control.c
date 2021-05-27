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

float Read_Encoder_Values(void)
{
	uint32_t  myEnc_top;
	float current_pos_top;
	uint32_t myEnc_Left;
	float current_pos_left;
	uint32_t myEnc_Right;
	float current_pos_right;

	// TOP ENCODER
	myEnc_top = ((TIM1 -> CNT) >> 2); // Shifted right by 2 to show 1 tick for each actual encoder tick
	current_pos_top = (float)myEnc_top*((2*M_PI)/PPR)*count;
	// LEFT MOTOR ENCODER
	myEnc_Left = ((TIM2 -> CNT) >> 2); // Shift right by 2 to show 1 tick for each actual encoder tick
	current_pos_left = (float)myEnc_Left*((2*M_PI)/PPR); // needs to be multiplied by a count
	// RIGHT MOTOR ENCODER
	myEnc_Right = ((TIM3 -> CNT) >> 2);
	current_pos_right = (float)myEnc_Right*((2*M_PI)/PPR); // needs to be multiplied by a count, will be the same as left motor

	return (float)current_pos_top, (float)current_pos_left, (float)current_pos_right;
}

void Kill_Motors(void)
{
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,0);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,0);
}

