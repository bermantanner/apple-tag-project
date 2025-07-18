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
#include <string.h>
#include <stdint.h>
#include <stdio.h>

/* Include memory map of our MCU */
#include <stm32l475xx.h>

/* Include LED driver */
#include "timer.h"
#include "i2c.h"
#include "lsm6dsl.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int dataAvailable = 0;

SPI_HandleTypeDef hspi3;



void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI3_Init(void);
void privtag_run();

#define MOVEMENT_THRESHOLD 4000     	// Max movement until movement triggered
#define LOST_TIME_THRESHOLD 60000  		// 60 seconds in milliseconds

// Global Variables for states
volatile uint8_t timer_flag = 0;	 	// timer flag which is set by interrupt handler, read/cleared by main loop
volatile uint8_t is_lost = 0;		 	// flag for lost state.
volatile uint32_t time_still = 0;	 	// value for how long device has been still
volatile uint8_t send_message = 0;		// flag to trigger a BLE message every 10 seconds


// Redefine the libc _write() function so you can use printf in your code
int _write(int file, char *ptr, int len) {
	int i = 0;
    for (i = 0; i < len; i++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  //SystemClock_Config();
  SystemClock_FullSpeed_Config();
  //disable_unused_peripherals_register();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI3_Init();

  //RESET BLE MODULE
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RESET_GPIO_Port,BLE_RESET_Pin,GPIO_PIN_SET);
  HAL_Delay(10);

  // Initialize the ble configurations
  ble_init();

//  privtag_run();				   // Call the privtag_run function to start the "application"
  //Initialize peripherals
  	//PWR->CR1 |= PWR_CR1_LPR;

  	i2c_init();
  	lsm6dsl_init();

  	FLASH->ACR &= ~0b111;
  	FLASH->ACR |= 0b000;

  	PWR->CR1 &= ~0b11000000000;
  	PWR->CR1 |=  0b10000000000;

  	while ((PWR->SR2 & PWR_SR2_VOSF) != 0)
  	{
  	    // Wait???
  	}

  	lptim_init();
  	set_low_timer_ms();
  	// x y z variables to hold current accelerations in the x y z acceleration values
  	int16_t x, y, z;

  	//prev_x, prev_y and prev_z variables to hold the previous x y z acceleration values
  	int16_t prev_x = 0, prev_y = 0, prev_z = 0;

  	// delta_x, delta_y, and delta_z variables to hold the changes in the x y z values
  	int32_t delta_x, delta_y, delta_z;

  	// A variable used to hold the magnitude of the total movement from all direction
  	int32_t total_movement;

  	// A flag to determine if the device has moved
  	uint8_t device_moved_flag;

  	// Variable to store the minutes since lost (for project 2)
  	uint8_t minutes_since_lost = 0;

  	// Variable to store the seconds since lost
  	uint32_t seconds_since_lost = 0;

  	// A string to hold the second since lost as a string
  	char seconds_since_lost_str[20];

  	// First disconnect the device, set the discoverability to be false because we are not in lost mode yet, and set the non discoverable flag to be true
  	///SystemClock_FullSpeed_Config();

  	disconnectBLE();
  	setDiscoverability(0);
  	uint8_t nonDiscoverable = 1;
  	standbyBle();

  	disableUselessShit();


  	//SystemClock_LowPower_Config();

  	//disable_unused_peripherals_register();

  	// Hard coded name for the device
  	unsigned char device_name[] = "TaneTag";

  	while (1) {

  		if(!nonDiscoverable && HAL_GPIO_ReadPin(BLE_INT_GPIO_Port,BLE_INT_Pin)){
  			//SystemClock_FullSpeed_Config();

  			catchBLE();

  			//SystemClock_LowPower_Config();
  		}

  		if (timer_flag) { 			       // This triggers every 50 ms
  			timer_flag = 0; 			   // Reset the timer flag
  			lsm6dsl_read_xyz(&x, &y, &z);  // Read acceleration data

  			// Calculate total magnitude of change in movement
  			delta_x = abs(x - prev_x);
  			delta_y = abs(y - prev_y);
  			delta_z = abs(z - prev_z);

  			// Calculate the the total movement
  			total_movement = delta_x + delta_y + delta_z;

  			//Saves the current x y z values as the prev_x, prev_y, and prev_z for the next iteration
  			prev_x = x;
  			prev_y = y;
  			prev_z = z;

  			//If our device's total movement is beyond threshold, update device moved flag.
  			if (total_movement > MOVEMENT_THRESHOLD) {
  				device_moved_flag = 1;
  			}
  			else {
  				device_moved_flag = 0;
  			}


  			//If device DID move.
  			if (device_moved_flag) {
  				is_lost = 0;										 // If device moved, turn is lost mode to be OFF
  				time_still = 0;										 // If device moved, reset the time that it was still to be 0
  				minutes_since_lost = 0;								 // If device moved, reset the minutes since lost to be 0
  				seconds_since_lost = 0;								 // If device moved, reset the seconds since lost to be 0
				disconnectBLE();
  				// If the device is not in nonDiscoverable mode and it moved, then we disconnect the device first, then we set the discoverability to be false, and set the nonDiscoverable flag to be true
  				if (!nonDiscoverable) {
  					//SystemClock_FullSpeed_Config();
  				    setDiscoverability(0);

  				    nonDiscoverable = 1;

  				    //SystemClock_LowPower_Config();
  				}
  				standbyBle();

  			}
  			else {
  			    if (time_still >= LOST_TIME_THRESHOLD && !is_lost) { // If the device has been there for long as the threshold, and it is not currently lost, turn on lost mode
  			        is_lost = 1;
  			        //If the device is in non discoverable mode, then we set the discoverability to be true, and set the nonDiscoverable flag to be false



  			        if (nonDiscoverable) {
  			        	//SystemClock_FullSpeed_Config();

  			            setDiscoverability(1);
  			            nonDiscoverable = 0;

  			            //SystemClock_LowPower_Config();
  			        }
  			    }

  			}


  			if (is_lost) { //LOST MODE!
  				// Calculates the total minutes lost
  				minutes_since_lost = ((time_still - LOST_TIME_THRESHOLD) / LOST_TIME_THRESHOLD) + 1;

  				// Calculates the total seconds lost
  				uint32_t seconds_since_lost = (time_still - LOST_TIME_THRESHOLD) / 1000;

  				// If the send message flag is set, send a message to the user
  				if (send_message) {

  					//SystemClock_FullSpeed_Config();

  					//Build the string to send out
  					unsigned char formatted_str[32];
  					snprintf((char*)formatted_str, sizeof(formatted_str), "%s %us", device_name, seconds_since_lost);

  					// Use strlen to get the actual string length
  					int str_len = strlen((char*)formatted_str);

  					// Send the message to the user
  					updateCharValue(NORDIC_UART_SERVICE_HANDLE, READ_CHAR_HANDLE, 0, str_len, formatted_str);
  					send_message = 0;

  					standbyBle();
  					//SystemClock_LowPower_Config();
  				}
  			}
  		}
  		HAL_SuspendTick();
  		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
  		HAL_ResumeTick();
  	}

}

void LPTIM1_IRQHandler(void)
{
    // Check if Auto-Reload Match interrupt is triggered
    if ((LPTIM1->ISR & LPTIM_ISR_ARRM) != 0) {
        // Clear the interrupt flag
        LPTIM1->ICR = LPTIM_ICR_ARRMCF;

        // Set flag for main loop (equivalent to timer_flag)
        timer_flag = 1;

        // Increment time_still (now 1000 ms per interrupt instead of 50 ms)
        time_still += 2000;

        // Check for 10-second interval
        if ((time_still % 10000) == 0) {
            send_message = 1;
        }
    }
}

void disableUselessShit() {

	// Disable DMA1
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA1EN);

	// Disable DMA2
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_DMA2EN);

	// Disable Flash (Be very careful with this!)
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_FLASHEN);

	// Disable CRC
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_CRCEN);

	// Disable Touch Sensing Controller
	RCC->AHB1ENR &= ~(RCC_AHB1ENR_TSCEN);

	//EVERYTHING WORKS WELL

	// Disable GPIO Ports
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOAEN);
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOBEN);
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOCEN);
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOFEN);
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOGEN);
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOHEN);

	// Disable ADC
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_ADCEN);

	// Disable RNGEN

	RCC->AHB2ENR &= ~(RCC_AHB2ENR_RNGEN);

	// Disable Random Number Generator
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_RNGEN);

	// Everything works

	// Disable QSPI (Quad SPI memory interface)
	RCC->AHB3ENR &= ~(RCC_AHB3ENR_QSPIEN);

	// Disable Flexible Memory Controller (FMC)
	RCC->AHB3ENR &= ~(RCC_AHB3ENR_FMCEN);

	// Bit 29: OPAMPEN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_OPAMPEN);

	// Bit 28: DAC1EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_DAC1EN);
//
//	// Bit 26: Reserved (do not modify)
//
	// Bit 24: CAN1EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_CAN1EN);

	// Disable Window Watchdog Timer
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_WWDGEN);

	// Bit 23: I2C3EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_I2C3EN);

	// Bit 22: I2C2EN (keeping this enabled as per your instruction)

	// Bit 21: I2C1EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_I2C1EN);

	// Bit 20-19: Reserved (do not modify)

	// Bit 18: UART5EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_UART5EN);

	// Bit 17: UART4EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_UART4EN);

	// Bit 16: USART3EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_USART3EN);

	// Bit 15: USART2EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_USART2EN);

	// Bit 14-13: Reserved (do not modify)

	// Bit 11: SPI2EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_SPI2EN);
//
//	// Bit 10-9: Reserved (do not modify)
//
	// Bit 8: TIM7EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_TIM7EN);

	// Bit 7: TIM6EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_TIM6EN);

	// Bit 6: TIM5EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_TIM5EN);

	// Bit 5: TIM4EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_TIM4EN);

	// Bit 4: TIM3EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_TIM3EN);

	// Bit 3: TIM2EN
	RCC->APB1ENR1 &= ~(RCC_APB1ENR1_TIM2EN);

//	// Bit 2-1: Reserved (do not modify)
//
//	// Bit 0: Reserved (do not modify)


	//EVERYTHING WORKKS


	// Disable LPTIM2
	RCC->APB1ENR2 &= ~(RCC_APB1ENR2_LPTIM2EN);

	// Disable Single Wire Protocol (SWPMI1)
	RCC->APB1ENR2 &= ~(RCC_APB1ENR2_SWPMI1EN);

	// Disable LPUART1
	RCC->APB1ENR2 &= ~(RCC_APB1ENR2_LPUART1EN);

	//Everything works

	// Disable DFSDM1
	RCC->APB2ENR &= ~(RCC_APB2ENR_DFSDM1EN);

	// Disable SAI2
	RCC->APB2ENR &= ~(RCC_APB2ENR_SAI2EN);

	// Disable SAI1
	RCC->APB2ENR &= ~(RCC_APB2ENR_SAI1EN);

	// Disable Timers
	RCC->APB2ENR &= ~(RCC_APB2ENR_TIM17EN);
	RCC->APB2ENR &= ~(RCC_APB2ENR_TIM16EN);
	RCC->APB2ENR &= ~(RCC_APB2ENR_TIM15EN);
	RCC->APB2ENR &= ~(RCC_APB2ENR_TIM8EN);
	RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN);

	// Disable USART1
	RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN);

	// Disable SPI1
	RCC->APB2ENR &= ~(RCC_APB2ENR_SPI1EN);

	// Disable SD MMC
	RCC->APB2ENR &= ~(RCC_APB2ENR_SDMMC1EN);

	// Disable Firewall
	RCC->APB2ENR &= ~(RCC_APB2ENR_FWEN);

	// Disable SYSCFG
	RCC->APB2ENR &= ~(RCC_APB2ENR_SYSCFGEN);

	//Everything works

//	// AHB1SMENR (DMA, Flash, CRC, TSC)
//	RCC->AHB1SMENR &= ~(RCC_AHB1SMENR_DMA1SMEN);
//	RCC->AHB1SMENR &= ~(RCC_AHB1SMENR_DMA2SMEN);
//	RCC->AHB1SMENR &= ~(RCC_AHB1SMENR_FLASHSMEN);
//	RCC->AHB1SMENR &= ~(RCC_AHB1SMENR_CRCSMEN);
//	RCC->AHB1SMENR &= ~(RCC_AHB1SMENR_TSCSMEN);
//
//	// AHB2SMENR (GPIO Ports, ADC, RNG)
//	RCC->AHB2SMENR &= ~(RCC_AHB2SMENR_GPIOASMEN);
//	RCC->AHB2SMENR &= ~(RCC_AHB2SMENR_GPIOBSMEN);
//	RCC->AHB2SMENR &= ~(RCC_AHB2SMENR_GPIOCSMEN);
//	RCC->AHB2SMENR &= ~(RCC_AHB2SMENR_GPIOFSMEN);
//	RCC->AHB2SMENR &= ~(RCC_AHB2SMENR_GPIOGSMEN);
//	RCC->AHB2SMENR &= ~(RCC_AHB2SMENR_GPIOHSMEN);
//	RCC->AHB2SMENR &= ~(RCC_AHB2SMENR_ADCSMEN);
//	RCC->AHB2SMENR &= ~(RCC_AHB2SMENR_RNGSMEN);
//
//	// AHB3SMENR (QSPI, FMC)
//	RCC->AHB3SMENR &= ~(RCC_AHB3SMENR_QSPISMEN);
//	RCC->AHB3SMENR &= ~(RCC_AHB3SMENR_FMCSMEN);
//
//	// APB1SMENR1 (OPAMP, DAC, CAN, I2C, UART, SPI, Timers)
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_OPAMPSMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_DAC1SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_CAN1SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_WWDGSMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_I2C3SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_I2C1SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_UART5SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_UART4SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_USART3SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_USART2SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_SPI2SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_TIM7SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_TIM6SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_TIM5SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_TIM4SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_TIM3SMEN);
//	RCC->APB1SMENR1 &= ~(RCC_APB1SMENR1_TIM2SMEN);
//
//	// APB1SMENR2
//	RCC->APB1SMENR2 &= ~(RCC_APB1SMENR2_LPTIM2SMEN);
//	RCC->APB1SMENR2 &= ~(RCC_APB1SMENR2_SWPMI1SMEN);
//	RCC->APB1SMENR2 &= ~(RCC_APB1SMENR2_LPUART1SMEN);
//
//	// APB2SMENR
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_DFSDM1SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_SAI2SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_SAI1SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_TIM17SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_TIM16SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_TIM15SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_TIM8SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_TIM1SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_USART1SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_SPI1SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_SDMMC1SMEN);
//	RCC->APB2SMENR &= ~(RCC_APB2SMENR_SYSCFGSMEN);
//



}

void disable_unused_peripherals_register(void) {

    RCC->AHB1ENR &= ~(
        RCC_AHB1ENR_DMA1EN |    // DMA1
        RCC_AHB1ENR_DMA2EN |    // DMA2
        RCC_AHB1ENR_CRCEN |     // CRC
        RCC_AHB1ENR_TSCEN       // Touch sensing controller
    );

    // Disable AHB2 Peripherals
	RCC->AHB2ENR &= ~(
		RCC_AHB2ENR_GPIOBEN |   // GPIO Port B
		RCC_AHB2ENR_GPIOCEN |   // GPIO Port C
		RCC_AHB2ENR_GPIOFEN |   // GPIO Port F
		RCC_AHB2ENR_GPIOGEN |   // GPIO Port G
		RCC_AHB2ENR_GPIOHEN |   // GPIO Port H
		RCC_AHB2ENR_ADCEN |     // ADC
		RCC_AHB2ENR_RNGEN     // Random Number Generator
	);

	// Disable GPIOB clock
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOBEN);

	// Disable GPIOC clock
	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOCEN);
//
//	// Disable GPIOD clock
//	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIODEN);

//	// Disable GPIOE clock
//	RCC->AHB2ENR &= ~(RCC_AHB2ENR_GPIOEEN);

    // Disable USART clocks
    RCC->APB2ENR &= ~(RCC_APB2ENR_USART1EN);
    RCC->APB1ENR1 &= ~(RCC_APB1ENR1_USART2EN |
                       RCC_APB1ENR1_USART3EN);
    RCC->APB1ENR1 &= ~(RCC_APB1ENR1_UART4EN |
                       RCC_APB1ENR1_UART5EN);

    // Disable I2C clocks
    RCC->APB1ENR1 &= ~(RCC_APB1ENR1_I2C1EN |
                       RCC_APB1ENR1_I2C3EN);

    // Disable SPI clocks
    RCC->APB2ENR &= ~(RCC_APB2ENR_SPI1EN);
    RCC->APB1ENR1 &= ~(RCC_APB1ENR1_SPI2EN);

    // Disable ADC clock
    RCC->AHB2ENR &= ~(RCC_AHB2ENR_ADCEN);

    // Disable DAC clock
    RCC->APB1ENR1 &= ~(RCC_APB1ENR1_DAC1EN);

    // Disable Timer clocks
    RCC->APB1ENR1 &= ~(RCC_APB1ENR1_TIM2EN |
                       RCC_APB1ENR1_TIM3EN |
                       RCC_APB1ENR1_TIM4EN |
                       RCC_APB1ENR1_TIM5EN |
                       RCC_APB1ENR1_TIM6EN |
                       RCC_APB1ENR1_TIM7EN);

    // Disable CAN clock
    RCC->APB1ENR1 &= ~(RCC_APB1ENR1_CAN1EN);

    // Disable USB clock
    RCC->APB1ENR1 &= ~(1 << 26);

}

void SystemClock_FullSpeed_Config(void)
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
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_7;  // 8 MHz
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
