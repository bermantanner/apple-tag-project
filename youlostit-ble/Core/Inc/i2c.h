/*
 * i2c.h
 *
 *  Created on: Jan 28, 2025
 *      Author: thanh
 */

#include <stm32l475xx.h>

#ifndef I2C_H_
#define I2C_H_

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len);
void i2c_init();

#endif /* I2C_H_ */
