/*

timer.c*
Created on: Oct 5, 2023
Author: schulman*/

#include "timer.h"

/* Include LED driver */
#include "leds.h"

void timer_init(TIM_TypeDef* timer)
{
  // TODO implement this
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; // Supply power to the timer

  //Stop the timer and clear out any timer state and reset all counters.
  timer->CR1 &= ~TIM_CR1_CEN;
  timer->CNT = 0;

  //Setup the timer to auto-reload when the max value is reached.
  timer->ARR = 0xFFFFFFFF;

  // Enable the timer’s interrupt both internally and in the interrupt controller (NVIC).
  timer->DIER |= TIM_DIER_UIE;

  // You will need to use the NVIC functions NVIC_EnableIRQ, NVIC_SetPriority with the parameter TIM2_IRQn
  NVIC_EnableIRQ(TIM2_IRQn);
  NVIC_SetPriority(TIM2_IRQn, 1); // what priority number ?

  // Setup the clock tree to pass into the timer and divide it down as needed (hint: look at the ENR registers in the RCC peripheral). Note: The default clock speed of your microcontroller after reboot is 4 MHz. You may want to slow this down by setting the dividers in the clock tree so the timer has a slower clock to operate with (Chapter 6).
  timer->PSC = 3999;

  // Enable the timer.
  timer->CR1 |= TIM_CR1_CEN;
}

void timer_reset(TIM_TypeDef* timer)
{
  // Reset timer 2’s (TIM2) counters, but do not reset the entire TIM peripheral. The timer can be in the middle of execution when it is reset and it’s counter will return to 0 when this function is called.
  timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
	timer_reset(timer);
	// Set the period that the timer will fire (in milliseconds). A timer interrupt should be fired for each timer period.
	timer->ARR = period_ms - 1;
}
