/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
//#include "ble_commands.h"
#include "ble.h"

#include <stdlib.h>

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#include <string.h>

/* Include LED driver */
#include "leds.h"

/* Include Timer driver */
#include "timer.h"

/* Include i2c driver */
#include "i2c.h"

/* Include accelerometer driver */
#include "lsm6dsl.h"

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */

#define MS_PER_CYCLE_READ 200 // Number of ms for each cycle to read accelerometer values
#define VARIANCE_THRESHOLD 500000 //minimum required change in acceleration for movement detection
#define LOST_TIME_LIMIT 60 * 1000 / MS_PER_CYCLE_READ // # of cycles before systems considers device "lost" (hasnt moved)
#define NUM_CYCLES_FOR_MESSAGE 10000 / MS_PER_CYCLE_READ // # of cycles before a message should be sent

int cycles_to_minutes(int cycles, int ms_per_cycle) {
	int total_ms = cycles * ms_per_cycle;
	int total_s = total_ms / 1000;
	int minutes = total_s / 60;

	return minutes;
}

int cycles_to_seconds(int cycles, int ms_per_cycle) {
	int total_ms = cycles * ms_per_cycle;
	int total_s = total_ms / 1000;

	return total_s;
}

//Needed in interrupt handlers
volatile bool isLost = false;
volatile bool readAccel = false;
volatile int lost_counter = 0; // tracks how long motion is below threshold in number of cycles
extern uint8_t deviceName[];
volatile bool timeForMessage = false;
volatile int message_counter = 0;

// Notifies when it's time to read accelerometer values or if it's time for a message
void TIM2_IRQHandler() {
	//Interrupt handler that will fire at the end of each period of TIM2.
	//Note that global variables that are modified in interrupt handlers must be declared as volatile
	if(TIM2->SR & TIM_SR_UIF){
		readAccel = true;
		lost_counter += 1; //this gets reset in main if not lost
		if(isLost) message_counter += 1; // this gets reset in main once a message is sent
		TIM2->SR &= ~TIM_SR_UIF;
	}
}

int _write(int file, char *ptr, int len) {
    int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

// Notifies when it's time to read accelerometer values or if it's time for a message
void LPTIM1_IRQHandler() {
    // Check if the interrupt is caused by the auto-reload match flag (ARRM) or another condition.
    if (LPTIM1->ISR & LPTIM_ISR_ARRM) {
        readAccel = true;
        lost_counter += 1; //this gets reset in main if not lost
        if(isLost) message_counter += 1; // this gets reset in main once a message is sent
        LPTIM1->ICR |= LPTIM_ICR_ARRMCF; // Clear the ARR Match Flag
    }
}

// External interrupt handler that gets triggered when there is a change in inactive/active state (sent by lsm6dsl)
//void EXTI15_10_IRQHandler() {
//	//Check if PE11 pin triggered it
//	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_11) != RESET){
//		isLost = true;
//		leds_set(0b11);
//		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);  // Clear interrupt flag
//	}
//}

int16_t ax, ay, az; //accelerometer values
int16_t prev_x, prev_y, prev_z; //prev acc values

int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI3_Init();

	//RESET BLE MODULE
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);

	ble_init();

	HAL_Delay(10);

	//uint8_t nonDiscoverable = 0;

	// Initially make device nondiscoverable
//	disconnectBLE();
	setDiscoverability(0);
//	catchBLE();

	i2c_init();
	lsm6dsl_init();

	leds_init();
	leds_set(0b00);

	//Device name will be stored here
	char deviceNameString[8];
	strncpy(deviceNameString, (char *) deviceName, sizeof(deviceNameString));

	// Set initial values for acceleration
	lsm6dsl_read_xyz(&ax, &ay, &az);
	prev_x = ax;
	prev_y = ay;
	prev_z = az;

	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN; // Enable the power interface clock
	//PWR->CR1 |= PWR_CR1_LPMS;          // Enable low power sleep mode

	//__disable_irq();

	timer_init_lptim(LPTIM1);
	timer_set_ms_lptim(LPTIM1, MS_PER_CYCLE_READ);


//	timer_init_tim(TIM2);
//	timer_set_ms_tim(TIM2, MS_PER_CYCLE_READ);

	//__enable_irq();


	while (1){
		if(readAccel){
			//leds_set(0b11);

			lsm6dsl_read_xyz(&ax, &ay, &az);

			//Calculates variances
			int var_x = (ax - prev_x) * (ax - prev_x);
			int var_y = (ay - prev_y) * (ay - prev_y);
			int var_z = (az - prev_z) * (az - prev_z);

			// if variances below threshold, assumes no movement and increments lost_counter
			if (var_x < VARIANCE_THRESHOLD && var_y < VARIANCE_THRESHOLD && var_z < VARIANCE_THRESHOLD) {
				//Enter lost mode if enough time has passed
				if (lost_counter >= LOST_TIME_LIMIT) isLost = true;
			}
			// if movement detected, reset counter and exit lost mode
			else {
				lost_counter = 0;
				isLost = false;
			}

			// updates previous acceleration values for next loop iteration
			prev_x = ax;
			prev_y = ay;
			prev_z = az;

			readAccel = false;
		}

		if(!isLost) {
			disconnectBLE();
			setDiscoverability(0);
			leds_set(0b00);
		}
		else {
			setDiscoverability(1);
			leds_set(0b11);
		}

		if(HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
			catchBLE();
		}

//		if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
//			catchBLE();
//		}

		// Send message via bluetooth if enough time has passed since previous message (only when device is lost)
//		if(timeForMessage){
//		  char test_str[20];
//		  sprintf(test_str, "%s%s%d%s", deviceNameString, " lost:", cycles_to_seconds(lost_counter, MS_PER_CYCLE_READ), "s");
//		  updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(test_str), (unsigned char*)test_str);
//		  timeForMessage = false;
//		}
		if (message_counter >= NUM_CYCLES_FOR_MESSAGE) {
			char test_str[20];
			sprintf(test_str, "%s%s%d%s", deviceNameString, " lost:", cycles_to_seconds(lost_counter, MS_PER_CYCLE_READ), "s");
			updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, strlen(test_str), (unsigned char*)test_str);
			message_counter = 0;
		}

		HAL_SuspendTick();
		__WFI();
		SystemClock_Config();
		HAL_ResumeTick();
//		HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	}
}

/**
  * @brief System Clock Configuration
  * @attention This changes the System clock frequency, make sure you reflect that change in your timer
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  // This lines changes system clock frequency
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIO_LED1_GPIO_Port, GPIO_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_CS_GPIO_Port, BLE_CS_Pin, GPIO_PIN_SET);


  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port, BLE_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BLE_INT_Pin */
  GPIO_InitStruct.Pin = BLE_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLE_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_LED1_Pin BLE_RESET_Pin */
  GPIO_InitStruct.Pin = GPIO_LED1_Pin|BLE_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_CS_Pin */
  GPIO_InitStruct.Pin = BLE_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BLE_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
