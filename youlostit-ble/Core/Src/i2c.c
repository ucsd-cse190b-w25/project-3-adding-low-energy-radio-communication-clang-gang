#include "i2c.h"
#include <stm32l475xx.h>

void i2c_init() {
    /* Enable clocks for GPIOB and I2C2 */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;

    /* Configure PB10 and PB11 for alternate function (I2C2) */
    GPIOB->MODER &= ~((3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->MODER |= ((2U << (10 * 2)) | (2U << (11 * 2)));

    GPIOB->AFR[1] &= ~((0xFU << ((10 - 8) * 4)) | (0xFU << ((11 - 8) * 4)));
    GPIOB->AFR[1] |= ((4U << ((10 - 8) * 4)) | (4U << ((11 - 8) * 4)));

    GPIOB->OTYPER |= (1U << 10) | (1U << 11);
    GPIOB->OSPEEDR |= ((3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->PUPDR &= ~((3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->PUPDR |= ((1U << (10 * 2)) | (1U << (11 * 2)));

    /* Reset and configure I2C2 */
    I2C2->CR1 |= I2C_CR1_SWRST;
    I2C2->CR1 &= ~I2C_CR1_SWRST;

    //Set PRESC = 1, SCLDEL = 0, SDADEL = 0, SCLH = 533, SCLL = 533 all at once; results in I2C clock frequency of around 15 kHz
    I2C2->TIMINGR = 0x10707D15;

    I2C2->CR1 |= I2C_CR1_PE;
}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len) {
    while (I2C2->ISR & I2C_ISR_BUSY){
        if (I2C2->ISR & I2C_ISR_TIMEOUT) return 1;
    }

    I2C2->CR2 = 0;
    I2C2->CR2 |= (address << 1) | (len << I2C_CR2_NBYTES_Pos) | I2C_CR2_AUTOEND;

    if (dir == 0) {
        I2C2->CR2 &= ~I2C_CR2_RD_WRN;
        I2C2->CR2 |= I2C_CR2_START;

        for (uint8_t i = 0; i < len; i++) {
            while (!(I2C2->ISR & I2C_ISR_TXIS)) {
            	if (I2C2->ISR & I2C_ISR_TIMEOUT) return 2;
            }
            I2C2->TXDR = data[i];
        }
    }
    else {
        I2C2->CR2 |= I2C_CR2_RD_WRN;
        I2C2->CR2 |= I2C_CR2_START;

        for (uint8_t i = 0; i < len; i++) {
            while (!(I2C2->ISR & I2C_ISR_RXNE)) {
            	if (I2C2->ISR & I2C_ISR_TIMEOUT) return 3;
            }
            data[i] = I2C2->RXDR;
        }
    }

    while (!(I2C2->ISR & I2C_ISR_STOPF)) {
    	if (I2C2->ISR & I2C_ISR_TIMEOUT) return 4;
    }
    I2C2->ICR = I2C_ICR_STOPCF;

    return 0;
}
