/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{

	// Turning on clock for GPIO A and GPIO B
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	//Configure PA5 as an output by clearing all bits and setting the mode
	GPIOA->MODER &= ~GPIO_MODER_MODE5;
	GPIOA->MODER |= GPIO_MODER_MODE5_0;

	//Configure PA14 as an output by clearing all bits and setting the mode
	GPIOB->MODER &= ~GPIO_MODER_MODE14;
	GPIOB->MODER |= GPIO_MODER_MODE14_0;

	//Configure the GPIO output as push pull (transistor for high and low) for PA5
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;
	//Configure the GPIO output as push pull (transistor for high and low) for PA14
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

	//Disable the internal pull-up and pull-down resistors for PA5
	GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;
	//Disable the internal pull-up and pull-down resistors for PA14
	GPIOB->PUPDR &= GPIO_PUPDR_PUPD14;


	//Configure the GPIOA to use low speed mode
	GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);
	//Configure the GPIOB to use low speed mode */
	GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

	//Turn off the LED1
	GPIOA->ODR &= ~GPIO_ODR_OD5;
	//Turn off the LED2
	GPIOB->ODR &= ~GPIO_ODR_OD14;


}

void leds_set(uint8_t led)
{
	//Turn off LED1
	GPIOA->ODR &= ~GPIO_ODR_OD5;
	//Turn off LED2
	GPIOB->ODR &= ~GPIO_ODR_OD14;

	//If the value is 1, turn on LED1
	if (led & 0b01) {
		//TURN on LED1
		GPIOA->ODR |= GPIO_ODR_OD5;
	}
	//If the value is 2, turn on lED2
	if (led & 0b10) {
		//Turn on LED2
		GPIOB->ODR |= GPIO_ODR_OD14;
	}

}
