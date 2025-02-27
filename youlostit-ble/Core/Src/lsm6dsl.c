/*
 * lsm6dsl.c
 * Created on: Feb 5, 2025
 * Author: angie
 */

#include "lsm6dsl.h"
#include "i2c.h"
#include <stdio.h>
#include <stdint.h>

#define LSM6DSL_ADDR          0x6A // Slave address for the LSM6DSL

/* Register definitions */
#define WHO_AM_I_REG          0x0F
#define CTRL1_XL              0x10
#define CTRL3_C               0x12
#define OUTX_L_XL             0x28

//Helper functions to encapsulate read/writes with I2C
static uint8_t lsm6dsl_write_reg(uint8_t reg, uint8_t value, uint8_t len) {
    uint8_t data[2] = {reg, value};
    return i2c_transaction(LSM6DSL_ADDR, 0, data, len + 1);
}

static uint8_t lsm6dsl_read_reg(uint8_t reg, uint8_t* value, uint8_t len) {
    uint8_t status = i2c_transaction(LSM6DSL_ADDR, 0, &reg, 1);
    if (status != 0) {
    	printf("LSM6DSL: Error setting up read address (err %d)\n", status);
    	return status;
    }
    return i2c_transaction(LSM6DSL_ADDR, 1, value, len);
}

void lsm6dsl_init() {
    uint8_t status;
    uint8_t who_am_i = 0;

    status = lsm6dsl_read_reg(WHO_AM_I_REG, &who_am_i, 1);
    if (status != 0) printf("LSM6DSL: Error reading WHO_AM_I register (err %d)\n", status);
    //Check WHO_AM_I has the expected value
    if (who_am_i != LSM6DSL_ADDR) printf("LSM6DSL: Unexpected WHO_AM_I value: 0x%02X\n", who_am_i);
    else printf("LSM6DSL: WHO_AM_I = 0x%02X\n", who_am_i);

    status = lsm6dsl_write_reg(CTRL3_C, 0x04, 1);
    if (status != 0) printf("LSM6DSL: Error configuring CTRL3_C (err %d)\n", status);
    else printf("LSM6DSL: CTRL3_C configured (auto-increment enabled)\n");

    status = lsm6dsl_write_reg(CTRL1_XL, 0x60, 1);
    if (status != 0) printf("LSM6DSL: Error writing CTRL1_XL (err %d)\n", status);
    else printf("LSM6DSL: CTRL1_XL configured (0x60)\n");
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t status;
    uint8_t data[6];
    uint8_t reg = OUTX_L_XL;

    status = lsm6dsl_read_reg(reg, data, 6);
    if (status != 0) printf("LSM6DSL: Error reading acceleration data (err %d)\n", status);

    //Store data values in the appropriate location; combine low and high values for each X,Y,Z value
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
}
