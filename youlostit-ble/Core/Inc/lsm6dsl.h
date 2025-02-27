/*
 * lsm6dsl.h
 *
 *  Created on: Feb 5, 2025
 *      Author: angie
 */
#include <stdint.h>

#ifndef LSM6DSL_H_
#define LSM6DSL_H_

void lsm6dsl_init();
void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z);

#endif /* LSM6DSL_H_ */
