/*
 * timer.h
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#ifndef TIMER_H_
#define TIMER_H_

/* Include the type definitions for the timer peripheral */
#include <stm32l475xx.h>

void timer_init_tim(TIM_TypeDef* timer);
void timer_reset_tim(TIM_TypeDef* timer);
void timer_set_ms_tim(TIM_TypeDef* timer, uint16_t period_ms);
void timer_init_lptim(LPTIM_TypeDef *timer);
void timer_reset_lptim(LPTIM_TypeDef *timer);
void timer_set_ms_lptim(LPTIM_TypeDef *timer, uint16_t period_ms);


#endif /* TIMER_H_ */
