/*
 * lsm6dsl.h
 *
 *  Created on: Feb 6, 2025
 *      Author: thanh
 */

#ifndef LSM6DSL_H_
#define LSM6DSL_H_

void lsm6dsl_init();
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);
void lsm6dsl_init_movement_detection();
uint8_t lsm6dsl_check_movement();

#endif /* LSM6DSL_H_ */
