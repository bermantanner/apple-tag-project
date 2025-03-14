/*
 * lsm6dsl.c
 *
 *  Created on: Feb 5, 2025
 *      Author: thanh
 */

#include "i2c.h"
#include "lsm6dsl.h"

#define LSM6DSL_ADDR 0x6A  // 7-bit I2C address
#define WHO_AM_I_REG 0x0F  // 7-bit test address
#define CTRL1_XL 0x10      // Control register for accelerometer
#define INT1_CTRL 0x0D     // Interrupt control register
#define OUTX_L 0x28        // X-axis low byte
#define OUTX_H 0x29        // X-axis high byte
#define OUTY_L 0x2A		   // Y-axis low byte
#define OUTY_H 0x2B		   // Y-axis high byte
#define OUTZ_L 0x2C		   // Z-axis low byte
#define OUTZ_H 0x2D		   // Z-axis high byte

void lsm6dsl_init() {

	// An array to hold the data we want to write to, peripheral address, register address,
    uint8_t data[2];

    // Step 1: Enable accelerometer, 416Hz, ±2g range
    data[0] = CTRL1_XL;
    data[1] = 0x60;  			// 16Hz ODR, high-performance mode
    if (i2c_transaction(LSM6DSL_ADDR, 0, data, 2)) {
        printf("Error: Failed to configure CTRL1_XL\n");
    }

    // Step 2: Enable accelerometer data-ready interrupt on INT1
    data[0] = INT1_CTRL;
    data[1] = 0x01;  			// Enable INT1_DRDY_XL
    if (i2c_transaction(LSM6DSL_ADDR, 0, data, 2)) {
        printf("Error: Failed to configure INT1_CTRL\n");
    }
}


void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {

	// Slave register address, we want to start at OUTX_L
	uint8_t reg = OUTX_L;
	// Since all the register we need to access are adjacent to each other, we can just read 6 bytes of data starting from OUTX_L
	// An array to hold all the x y z data -> OUTX_L, OUTX_H, OUTY_L, OUTY_H, OUTZ_L, OUTZ_H
    uint8_t data[6];

    // Write register address, then read 6 bytes in one transaction,
    if (i2c_transaction(LSM6DSL_ADDR, 0, &reg, 1)) {
        printf("Error: Failed to set register address for reading\n");
        return;
    }
    if (i2c_transaction(LSM6DSL_ADDR, 1, data, 6)) {
        printf("Error: Failed to read accelerometer data\n");
        return;
    }

    // Convert the high and low data into their right position using bit shift
    // X values are in the first 2 elements of data, Y values are in the next 2 and so on
    *x = (int16_t)(data[1] << 8 | data[0]);
    *y = (int16_t)(data[3] << 8 | data[2]);
    *z = (int16_t)(data[5] << 8 | data[4]);
}

