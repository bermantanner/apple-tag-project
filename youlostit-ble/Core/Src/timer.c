/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"

const int MAX_PSC_VALUE = 65536;

const int CHOSEN_PSC_VALUE = 7999;

void timer_init(TIM_TypeDef* timer)
{

	//Give the clock to the timer, basically for power
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	//Stop the timer
	timer->CR1 &= ~TIM_CR1_CEN;

	//Clear out any timer state
	timer->SR &= ~TIM_SR_UIF;

	//Reset all counter
	timer->CNT = 0;

	//Enable the timer interrupt internally
	NVIC_EnableIRQ(TIM2_IRQn);
	NVIC_SetPriority(TIM2_IRQn, 1);

	//Enable the timer DMA interrupt event register
	timer->DIER |= TIM_DIER_UIE;

	//Setting clock prescaler to a safe initial value (max)
	timer->PSC = MAX_PSC_VALUE;

	//Enable the timer
	timer->CR1 |= TIM_CR1_CEN;

}

void timer_reset(TIM_TypeDef* timer)
{
	//Reset the counter of the timer to be 0
	timer->CNT = 0;

}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
	// Stopping the timer before making changes
    timer->CR1 &= ~TIM_CR1_CEN;

    // Reset the counter of the timer to be 0
    timer->CNT = 0;

	//We know that the clock is 4MHz, so we can use the prescaler to divide it down to just 1KHz so that it is easier to count
	timer->PSC = CHOSEN_PSC_VALUE;

	//Setting our Auto Reload Register to the period_ms which aligns with our chosen PSC Value
	timer->ARR = period_ms - 1;

	// Restart the timer
    timer->CR1 |= TIM_CR1_CEN;

}


