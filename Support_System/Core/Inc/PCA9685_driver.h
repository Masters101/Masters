/*
 * PCA9685_driver.h
 *
 *  Created on: May 26, 2021
 *      Author: Nathan Weiss
 */

#ifndef INC_PCA9685_DRIVER_H_
#define INC_PCA9685_DRIVER_H_

#endif /* INC_PCA9685_DRIVER_H_ */


#include "stm32f3xx_hal.h"
#include "stdbool.h"

#ifndef PCA9685_I2C_TIMEOUT
#define PCA9685_I2C_TIMEOUT 1
#endif  // PCA9685_I2C_TIMEOUT

#define PCA9685_I2C_DEFAULT_DEVICE_ADDRESS 0x80

typedef struct{
	I2C_HandleTypeDef *i2c_handle;
	uint16_t device_address;
	bool inverted;
}pca9685_handle_t;

// FUNCTION PROTOTYPES

void pca9685_init(pca9685_handle_t *handle);
void pca9685_wakeup(pca9685_handle_t *handle);
void pca9685_set_pwm_frequency(pca9685_handle_t *handle, float frequency);
void pca9685_set_pwm_channel_times(pca9685_handle_t *handle, unsigned channel,unsigned on_time,unsigned off_time);
void pca9685_set_channel_duty_cycle(pca9685_handle_t *handle, unsigned channel, float duty_cycle, bool logarithmic);
