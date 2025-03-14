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
#define WAKE_UP_DUR			  0x5C
#define WAKE_UP_THS			  0x5B
#define TAP_CFG               0x58
#define MD1_CFG               0x5E

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
    // Check WHO_AM_I has the expected value
    if (who_am_i != LSM6DSL_ADDR) printf("LSM6DSL: Unexpected WHO_AM_I value: 0x%02X\n", who_am_i);
    else printf("LSM6DSL: WHO_AM_I = 0x%02X\n", who_am_i);

    status = lsm6dsl_write_reg(CTRL3_C, 0x04, 1);
    if (status != 0) printf("LSM6DSL: Error configuring CTRL3_C (err %d)\n", status);
    else printf("LSM6DSL: CTRL3_C configured (auto-increment enabled)\n");

    // Sets the acc ODR to run at the lowest Hz offered, which would be 1.6 Hz
//    status = lsm6dsl_write_reg(CTRL1_XL, 0b10110000, 1);
//    if (status != 0) printf("LSM6DSL: Error writing CTRL1_XL (err %d)\n", status);
//    else printf("LSM6DSL: CTRL1_XL configured (0b10110000)\n");
    status = lsm6dsl_write_reg(CTRL1_XL, 0x60, 1);
    if (status != 0) printf("LSM6DSL: Error writing CTRL1_XL (err %d)\n", status);
    else printf("LSM6DSL: CTRL1_XL configured (0x60)\n");

    // Set duration for inactivity detection to be 60s; x = 60s * 1.6Hz = 96 LSB which is 0x60
//    status = lsm6dsl_write_reg(WAKE_UP_DUR, 0x00, 1);
//	if (status != 0) printf("LSM6DSL: Error writing WAKE_UP_DUR (err %d)\n", status);
//	else printf("LSM6DSL: WAKE_UP_DUR configured (0x01)\n");
//
//	// TODO: Set Activity/Inactivity threshold and enable interrupts; Currently very sensitive, let's how high we can make it
//	status = lsm6dsl_write_reg(WAKE_UP_THS, 0x20, 1);
//	if (status != 0) printf("LSM6DSL: Error writing WAKE_UP_THS (err %d)\n", status);
//	else printf("LSM6DSL: WAKE_UP_THS configured (0x20)\n");
//
//	// Set inactivity configuration; enable interrupts, enable inactivity function with gyro powered down, SLOPE filter, no latched interrupts
//	status = lsm6dsl_write_reg(TAP_CFG, 0b11100000, 1);
//	if (status != 0) printf("LSM6DSL: Error writing TAP_CFG (err %d)\n", status);
//	else printf("LSM6DSL: TAP_CFG configured (0x10)\n");
//
//	// Activity/Inactivity interrupt driven to INT1 pin
//	status = lsm6dsl_write_reg(MD1_CFG, 0x80, 1);
//	if (status != 0) printf("LSM6DSL: Error writing MD1_CFG (err %d)\n", status);
//	else printf("LSM6DSL: MD1_CFG configured (0x80)\n");
//
//	/* EXTI interrupt init */
//	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
//	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
//
//	// INT1 connected to PD11; Set PD11 pin so that it has external interrupt functionality
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	GPIO_InitStruct.Pin = GPIO_PIN_11;
//	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // Interrupt on falling edge
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
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
