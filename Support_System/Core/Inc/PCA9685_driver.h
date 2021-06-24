/*
 * PCA9685_driver.h
 *
 *  Created on: May 26, 2021
 *      Author: Nathan Weiss
 */

#ifndef INC_PCA9685_DRIVER_H_
#define INC_PCA9685_DRIVER_H_

#endif /* INC_PCA9685_DRIVER_H_ */

// I2C Timeout declaration
#ifndef PCA9685_I2C_TIMEOUT
#define PCA9685_I2C_TIMEOUT 1
#endif


#include "stm32f3xx_hal.h"
#include "stdbool.h"

#define PCA9685_I2C_DEFAULT_DEVICE_ADDRESS 0x80

void pca9685_init(I2C_HandleTypeDef *hi2c, uint8_t address, float frequency);
void pca9685_set_pwm(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t channel, float duty);
