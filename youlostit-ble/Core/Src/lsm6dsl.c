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
#define ACCEL_ODR_1_6_HZ    0x00  // Output Data Rate of 1.6 Hz in low-power mode
#define ACCEL_LOW_POWER_MODE     0x10    // Low-power mode configuration bit
#define CTRL2_G 0x11 // Control register for gyroscope
#define OUTX_L 0x28        // X-axis low byte
#define OUTX_H 0x29        // X-axis high byte
#define OUTY_L 0x2A		   // Y-axis low byte
#define OUTY_H 0x2B		   // Y-axis high byte
#define OUTZ_L 0x2C		   // Z-axis low byte
#define OUTZ_H 0x2D		   // Z-axis high byte
#define LSM6DSL_WAKE_UP_THRESHOLD_MEDIUM  0x03    // Medium sensitivity

// Additional register definitions
#define LSM6DSL_STATUS_REG     0x1E    // Status register
#define LSM6DSL_WAKE_UP_SRC    0x1B    // Wake-up source register

void lsm6dsl_init_movement_detection() {
    uint8_t data[2];

    // Configure accelerometer in low-power mode
    data[0] = CTRL1_XL;
    data[1] = ACCEL_LOW_POWER_MODE | ACCEL_ODR_1_6_HZ;
    if (i2c_transaction(LSM6DSL_ADDR, 0, data, 2)) {
        printf("Error: Failed to configure low-power mode\n");
    }

    // Configure wake-up threshold
    data[0] = LSM6DSL_WAKE_UP_THRESHOLD_MEDIUM;
    data[1] = 0x20;  // Adjust threshold as needed
    if (i2c_transaction(LSM6DSL_ADDR, 0, data, 2)) {
        printf("Error: Failed to set wake-up threshold\n");
    }
}

// Function to check for movement
uint8_t lsm6dsl_check_movement() {
    uint8_t status_reg = LSM6DSL_WAKE_UP_SRC;
    uint8_t status_data[1];

    // Read wake-up source register
    if (i2c_transaction(LSM6DSL_ADDR, 0, &status_reg, 1)) {
        printf("Error: Failed to set register address\n");
        return 0;
    }

    if (i2c_transaction(LSM6DSL_ADDR, 1, status_data, 1)) {
        printf("Error: Failed to read status\n");
        return 0;
    }

    // Check if movement detected (adjust bit mask as per datasheet)
    return (status_data[0] & 0x38) ? 1 : 0;
}

void lsm6dsl_init() {

	// An array to hold the data we want to write to, peripheral address, register address,
    uint8_t data[2];

    // Step 1: Enable accelerometer, 416Hz, ±2g range
    data[0] = CTRL1_XL;
   // data[1] = 0x60;  			// 16Hz ODR, high-performance mode
    data[1] = ACCEL_LOW_POWER_MODE | ACCEL_ODR_1_6_HZ;
    if (i2c_transaction(LSM6DSL_ADDR, 0, data, 2)) {
        printf("Error: Failed to configure CTRL1_XL\n");
    }

    // Step 2: Enable accelerometer data-ready interrupt on INT1
    data[0] = INT1_CTRL;
    data[1] = 0x01;  			// Enable INT1_DRDY_XL
    if (i2c_transaction(LSM6DSL_ADDR, 0, data, 2)) {
        printf("Error: Failed to configure INT1_CTRL\n");
    }

    // Disable gyroscope to save power
    data[0] = CTRL2_G;
    data[1] = 0x00; // Power down gyroscope
    if (i2c_transaction(LSM6DSL_ADDR, 0, data, 2)) {
        printf("Error: Failed to power down gyroscope\n");
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

