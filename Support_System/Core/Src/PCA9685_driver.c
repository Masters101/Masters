/*
 * PCA9685_driver.c
 *
 *  Created on: May 26, 2021
 *      Author: Nathan Weiss % modified from Henri Heimann Tutorial
 */

#include "PCA9685_driver.h"
#include "stdbool.h"
#include "math.h"
#include "assert.h"
#include "string.h"

#define PCA9685_SET_BIT_MASK(BYTE, MASK)		((BYTE) |= (uint8_t)(MASK))
#define PCA9685_CLEAR_BIT_MASK(BYTE, MASK)		((BYTE) &= (uint8_t)(~(uint8_t)(MASK)))
#define PCA9685_READ_BIT_MASK(BYTE, MASK)		((BYTE) & (uint8_t)(MASK))

// Register addresses
volatile typedef enum
{
	PCA9685_REGISTER_MODE1 = 0x00,
	PCA9685_REGISTER_MODE2 = 0x01,
	PCA9685_REGISTER_LED0_ON_L = 0x06,
	PCA9685_REGISTER_ALL_LED_ON_L = 0xfa,
	PCA9685_REGISTER_ALL_LED_ON_H = 0xfb,
	PCA9685_REGISTER_ALL_LED_OFF_L = 0xfc,
	PCA9685_REGISTER_ALL_LED_OFF_H = 0xfd,
	PCA9685_REGISTER_PRESCALER = 0xfe
}pca9685_reister_t;

//Bit masks for mode 1 register
volatile typedef enum
{
	PCA9685_REGISTER_MODE1_SLEEP = (1u << 4u),
	PCA9685_REGISTER_MODE1_RESTART = (1u << 7u)
}pca9685_register_mode1_t;

static const uint16_t CIEL_8_12[] = {
		0, 2, 4, 5, 7, 9, 11, 12, 14, 16, 18, 20, 21, 23, 25, 27, 28, 30, 32, 34, 36, 37, 39, 41, 43, 45, 47, 49, 52,
		54, 56, 59, 61, 64, 66, 69, 72, 75, 77, 80, 83, 87, 90, 93, 97, 100, 103, 107, 111, 115, 118, 122, 126, 131,
		135, 139, 144, 148, 153, 157, 162, 167, 172, 177, 182, 187, 193, 198, 204, 209, 215, 221, 227, 233, 239, 246,
		252, 259, 265, 272, 279, 286, 293, 300, 308, 315, 323, 330, 338, 346, 354, 362, 371, 379, 388, 396, 405, 414,
		423, 432, 442, 451, 461, 471, 480, 490, 501, 511, 521, 532, 543, 554, 565, 576, 587, 599, 610, 622, 634, 646,
		658, 670, 683, 696, 708, 721, 734, 748, 761, 775, 789, 802, 817, 831, 845, 860, 875, 890, 905, 920, 935, 951,
		967, 983, 999, 1015, 1032, 1048, 1065, 1082, 1099, 1117, 1134, 1152, 1170, 1188, 1206, 1225, 1243, 1262, 1281,
		1301, 1320, 1340, 1359, 1379, 1400, 1420, 1441, 1461, 1482, 1504, 1525, 1547, 1568, 1590, 1613, 1635, 1658,
		1681, 1704, 1727, 1750, 1774, 1798, 1822, 1846, 1871, 1896, 1921, 1946, 1971, 1997, 2023, 2049, 2075, 2101,
		2128, 2155, 2182, 2210, 2237, 2265, 2293, 2322, 2350, 2379, 2408, 2437, 2467, 2497, 2527, 2557, 2587, 2618,
		2649, 2680, 2712, 2743, 2775, 2807, 2840, 2872, 2905, 2938, 2972, 3006, 3039, 3074, 3108, 3143, 3178, 3213,
		3248, 3284, 3320, 3356, 3393, 3430, 3467, 3504, 3542, 3579, 3617, 3656, 3694, 3733, 3773, 3812, 3852, 3892,
		3932, 3973, 4013, 4055, 4095
};

void pca9685_init(pca9685_handle_t *handle)
{
	assert(handle->i2c_handle != NULL);

	// Mode registers set to default values(Auto-Increment, Sleep, Open-Drain);
	uint8_t mode1_reg_default_value = 0b00110000u;
	uint8_t mode2_reg_default_value = 0b00000000u;

	if (handle->inverted)
	{
		mode2_reg_default_value |= 0b00010000u;
	}

	uint8_t init_data1[] = { PCA9685_REGISTER_MODE1, mode1_reg_default_value};
	uint8_t init_data2[] = { PCA9685_REGISTER_MODE2, mode2_reg_default_value};

	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, init_data1, 2, PCA9685_I2C_TIMEOUT);
	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, init_data2, 2, PCA9685_I2C_TIMEOUT);

	uint8_t startup_data[4] = {0x00, 0x00, 0x00, 0x10};
	uint8_t transfer[5];
	transfer[0] = PCA9685_REGISTER_ALL_LED_ON_L;

	memcpy(&transfer[1], startup_data, 4);

	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, transfer, sizeof(startup_data) + 1, PCA9685_I2C_TIMEOUT);

	pca9685_set_pwm_frequency(handle, 1000);
	pca9685_wakeup(handle);
}

static void pca9685_sleep(pca9685_handle_t *handle)
{
	uint8_t mode1_reg;

	//if (HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, &PCA9685_REGISTER_MODE1, 1, PCA9685_I2C_TIMEOUT) != HAL_OK)
	//{
	HAL_I2C_Master_Receive(handle->i2c_handle, handle->device_address, &mode1_reg, 1, PCA9685_I2C_TIMEOUT);
	//}

	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
	PCA9685_SET_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);

	uint8_t data[2] = {PCA9685_REGISTER_MODE1_SLEEP, mode1_reg};
	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, data, 2, PCA9685_I2C_TIMEOUT);
}

void pca9685_wakeup(pca9685_handle_t *handle)
{
	uint8_t mode1_reg;
	//if (HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, &PCA9685_REGISTER_MODE1, 1, PCA9685_I2C_TIMEOUT) != HAL_OK)
	//{
	HAL_I2C_Master_Receive(handle->i2c_handle, handle->device_address, &mode1_reg, 1, PCA9685_I2C_TIMEOUT);
	//}

	bool restart_required = PCA9685_READ_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);

	// Clear the restart bit for now and clear the sleep bit
	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
	PCA9685_CLEAR_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_SLEEP);

	uint8_t data[2] = {PCA9685_REGISTER_MODE1_SLEEP, mode1_reg};
	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, data, 2, PCA9685_I2C_TIMEOUT);

	if(restart_required)
	{
		HAL_Delay(1); // Oscillator requires at least 500us to stabilize
		PCA9685_SET_BIT_MASK(mode1_reg, PCA9685_REGISTER_MODE1_RESTART);
		uint8_t data[2] = {PCA9685_REGISTER_MODE1_SLEEP, mode1_reg};
		HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, data, 2, PCA9685_I2C_TIMEOUT);
	}
}

void pca9685_set_pwm_frequency(pca9685_handle_t *handle, float frequency)
{
	assert(frequency >= 24);
	assert(frequency <= 1526);

	// Calculate the prescaler value (datasheet page 25)
	uint8_t prescaler = (uint8_t)roundf(25000000.0f / (4096 * frequency)) - 1;

	pca9685_sleep(handle); //must be in sleep mode to change frequency
	uint8_t prescaler_data[2] = {PCA9685_REGISTER_PRESCALER, prescaler};
	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, prescaler_data, 2, PCA9685_I2C_TIMEOUT); //write new prescaler

	pca9685_wakeup(handle);
}

void pca9685_set_pwm_channel_times(pca9685_handle_t *handle, unsigned channel, unsigned on_time, unsigned off_time)
{
	assert(channel >= 0);
	assert(channel < 16);

	assert(on_time >= 0);
	assert(on_time <= 4096);

	assert(off_time >= 0);
	assert(off_time <= 4096);

	uint8_t data_pwm[4] = { on_time, on_time >> 8u, off_time, off_time >> 8u };
	uint8_t transfer_pwm[5];
	transfer_pwm[0] =  PCA9685_REGISTER_LED0_ON_L + channel * 4;
	memcpy(&transfer_pwm[1], data_pwm, sizeof(data_pwm));
	HAL_I2C_Master_Transmit(handle->i2c_handle, handle->device_address, transfer_pwm, sizeof(data_pwm) + 1, PCA9685_I2C_TIMEOUT);
}

void pca9685_set_channel_duty_cycle(pca9685_handle_t *handle, unsigned channel, float duty_cycle, bool logarithmic)
{
	assert(duty_cycle >= 0.0);
	assert(duty_cycle <= 1.0);

	if (duty_cycle == 0.0)
	{
		pca9685_set_pwm_channel_times(handle, channel, 0, 4096); // Special value for always off
	}
	else if (duty_cycle == 1.0)
	{
		pca9685_set_pwm_channel_times(handle, channel, 4096, 0); // Special value for always on
	}
	else
	{
		unsigned required_on_time;

			if (logarithmic)
			{
				required_on_time = CIEL_8_12[(unsigned)roundf(255 * duty_cycle)];
			}
			else
			{
				required_on_time = (unsigned)roundf(4095 * duty_cycle);
			}

			// Offset on and off times depending on channel to minimise current spikes
			unsigned on_time = (channel == 0) ? 0 : (channel * 256) - 1;
			unsigned off_time = (on_time + required_on_time) & 0xfffu;

			pca9685_set_pwm_channel_times(handle, channel, on_time, off_time);
	}

	}
