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

//static const uint16_t CIEL_8_12[] = {
//		0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 18, 20, 21, 23, 25, 27, 28, 30, 32, 34, 36, 37, 39, 41, 43, 45, 47, 49, 52,
//		54, 56, 59, 61, 64, 66, 69, 72, 75, 77, 80, 83, 87, 90, 93, 97, 100, 103, 107, 111, 115, 118, 122, 126, 131,
//		135, 139, 144, 148, 153, 157, 162, 167, 172, 177, 182, 187, 193, 198, 204, 209, 215, 221, 227, 233, 239, 246,
//		252, 259, 265, 272, 279, 286, 293, 300, 308, 315, 323, 330, 338, 346, 354, 362, 371, 379, 388, 396, 405, 414,
//		423, 432, 442, 451, 461, 471, 480, 490, 501, 511, 521, 532, 543, 554, 565, 576, 587, 599, 610, 622, 634, 646,
//		658, 670, 683, 696, 708, 721, 734, 748, 761, 775, 789, 802, 817, 831, 845, 860, 875, 890, 905, 920, 935, 951,
//		967, 983, 999, 1015, 1032, 1048, 1065, 1082, 1099, 1117, 1134, 1152, 1170, 1188, 1206, 1225, 1243, 1262, 1281,
//		1301, 1320, 1340, 1359, 1379, 1400, 1420, 1441, 1461, 1482, 1504, 1525, 1547, 1568, 1590, 1613, 1635, 1658,
//		1681, 1704, 1727, 1750, 1774, 1798, 1822, 1846, 1871, 1896, 1921, 1946, 1971, 1997, 2023, 2049, 2075, 2101,
//		2128, 2155, 2182, 2210, 2237, 2265, 2293, 2322, 2350, 2379, 2408, 2437, 2467, 2497, 2527, 2557, 2587, 2618,
//		2649, 2680, 2712, 2743, 2775, 2807, 2840, 2872, 2905, 2938, 2972, 3006, 3039, 3074, 3108, 3143, 3178, 3213,
//		3248, 3284, 3320, 3356, 3393, 3430, 3467, 3504, 3542, 3579, 3617, 3656, 3694, 3733, 3773, 3812, 3852, 3892,
//		3932, 3973, 4013, 4055, 4095
//};

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

	if (duty == 1.0)
	{
		on_time = 0;
		off_time = 4095;
	}
	else if (duty == 0.0)
	{
		on_time = 4095;
		off_time = 0;
	}
	else if ( duty != (0.0 || 1.0))
	{
		off_time = 0;
		//on_time = CIEL_8_12[(unsigned)roundf(255 * duty)];
		off_time = (unsigned)roundf(4095*duty);;
	}

	uint8_t outputBuffer[5] ={LED0_ON_L + 4*channel, on_time, (on_time >> 8), off_time, (off_time >> 8)};
	HAL_I2C_Master_Transmit(hi2c,address,outputBuffer,5,PCA9685_I2C_TIMEOUT);
}
