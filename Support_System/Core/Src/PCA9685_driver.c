/*
 * PCA9685_driver.c
 *
 *  Created on: May 26, 2021
 *      Author: Nathan Weiss
 */

#include "PCA9685_driver.h"
#include "stdbool.h"
#include "math.h"
#include "assert.h"
#include "string.h"


#define PCA9685_REGISTER_MODE1 0x00
#define PCA9685_REGISTER_MODE2 0x01
#define PCA9685_REGISTER_PRESCALER 0xFE

#define LED0_ON_L 0x06
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09

void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address, float frequency)
{
	// Frequency range 24Hz -> 1526Hz
	uint8_t tx[2];
	uint8_t rx[1];
	rx[0] = PCA9685_REGISTER_MODE1;
	uint8_t buffer[1];
	uint8_t prescaler = (uint8_t)roundf(25000000.0f / (4096 * frequency)) - 1; // Data-sheet page 25

	HAL_I2C_Master_Transmit(hi2c,address,PCA9685_REGISTER_MODE1,1,PCA9685_I2C_TIMEOUT);

	HAL_I2C_Master_Transmit(hi2c,address,rx,1,PCA9685_I2C_TIMEOUT);
	HAL_I2C_Master_Receive(hi2c,address,buffer,1, PCA9685_I2C_TIMEOUT);

	uint8_t oldmode = 0x00; //Hard-coded ....buffer[0];
	uint8_t newmode = ((oldmode & 0x7F) | 0x10);
	tx[0] = PCA9685_REGISTER_MODE1;
	tx[1] = newmode;
	HAL_I2C_Master_Transmit(hi2c,address,tx,2,PCA9685_I2C_TIMEOUT); // go to sleep

	tx[0] = PCA9685_REGISTER_PRESCALER;
	tx[1] = prescaler;
	HAL_I2C_Master_Transmit(hi2c,address,tx,2,PCA9685_I2C_TIMEOUT);

	tx[0] = PCA9685_REGISTER_MODE1;
	tx[1] = oldmode;
	HAL_I2C_Master_Transmit(hi2c,address,tx,2,PCA9685_I2C_TIMEOUT);

	HAL_Delay(5);

	tx[1] = (oldmode | 0xA1);
	HAL_I2C_Master_Transmit(hi2c,address,tx,2,PCA9685_I2C_TIMEOUT);
}

void pca9685_set_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t channel, float duty)
{
	assert(duty >= 0.0);
	assert(duty <= 1.0);

	assert(channel >= 0);
	assert(channel <= 15);

	uint16_t on_time;
	uint16_t off_time;

	if (duty == 0.0)
	{
		on_time = 0;
		off_time = 4096;
	}
	else if (duty == 1.0)
	{
		on_time = 4096;
		off_time = 0;
	}
	else if ( duty != (0.0 || 1.0))
	{
		on_time = (unsigned)roundf(4095*duty);
		off_time = 4096 - on_time;
	}

	uint8_t outputBuffer[5] ={LED0_ON_L + 4*channel, on_time, (on_time >> 8), off_time, (off_time >> 8)};
	HAL_I2C_Master_Transmit(hi2c,address,outputBuffer,5,PCA9685_I2C_TIMEOUT);
}
