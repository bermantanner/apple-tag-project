/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"

void lptim_init(void) {

	// Enable LSI Ready Interrupt
	RCC->CIER |= RCC_CIER_LSIRDYIE;

    // Step 1: enable LSI
    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY));

    // Step 2: enable LPTIM1 clock
    RCC->APB1ENR1 |= RCC_APB1ENR1_LPTIM1EN;

    // Step 3: Configure LPTIM1 to use LSI clock
    RCC->CCIPR &= ~RCC_CCIPR_LPTIM1SEL; // Clear existing settings
    RCC->CCIPR |= RCC_CCIPR_LPTIM1SEL_0; // Select LSI (01 in the bitfield)

    // Step 4: Enable LPTIM1 to continue running in debug mode
    //DBGMCU->APB1FZR1 &= ~DBGMCU_APB1FZR1_DBG_LPTIM1_STOP;

    // 5. Disable the LPTIM before configuration
    LPTIM1->CR &= ~LPTIM_CR_ENABLE; // Clear ENABLE bit

    // Wait for the timer to be actually disabled
    while(LPTIM1->CR & LPTIM_CR_ENABLE) {}

    // 6. Clear any pending interrupts
    // Clear all LPTIM1 interrupt flags using specific macros
    LPTIM1->ICR = LPTIM_ICR_ARRMCF |    // Clear Auto-Reload Match flag
                  LPTIM_ICR_ARROKCF |    // Clear Auto-Reload OK flag
                  LPTIM_ICR_CMPOKCF |    // Clear Compare OK flag
                  LPTIM_ICR_EXTTRIGCF |  // Clear External Trigger flag
                  LPTIM_ICR_CMPMCF |      // Clear Compare Match flag
                  LPTIM_ICR_ARRMCF |  // Clear Interrupt Enable flag
                  LPTIM_ICR_DOWNCF;      // Clear Down Count flag

    // 7. Configure LPTIM Registers
    LPTIM1->CFGR = 0; // Clear configuration register
    //Try to clear this mannually also set the count

    // 8. Configure Clock Source and Counter Mode
    // CKSEL = 0 (internal clock)
    // COUNTMODE = 0 (update counter on each internal clock pulse)
    // Prescaler = 32 (division ratio of 32, corresponds to binary 101)
    LPTIM1->CFGR = (0b101 << LPTIM_CFGR_PRESC_Pos) | // Prescaler
                   (0 << LPTIM_CFGR_CKSEL_Pos) |      // Internal clock source
                   (0 << LPTIM_CFGR_COUNTMODE_Pos);   // Normal counting mode

    // 9. Set Operating Mode (Continuous Mode)
    // PRELOAD = 0 (immediate update)
    // LPTIM1->CFGR &= ~LPTIM_CFGR_PRELOAD;

    LPTIM1->CNT = 0;

    // 10. Configure Interrupt Registers
    // Enable relevant interrupts
    LPTIM1->IER = LPTIM_IER_ARRMIE; // Auto-reload match interrupt

    // 11. Set Auto-Reload Register (ARR)
    // For 1 kHz with 32 prescaler, assuming 32 kHz LSI
    // LPTIM1->ARR = 1000; // 1 ms period (adjust as needed)

    // 12. Wait for ARROK flag to confirm ARR register update
    // Might be stuck here while(!(LPTIM1->ISR & LPTIM_ISR_ARROK)) {}

    // 13. Clear ARROK flag
    // LPTIM1->ICR |= LPTIM_ICR_ARROKCF;

    // 16. Configure NVIC for LPTIM1 Interrupt
    NVIC_SetPriority(LPTIM1_IRQn, 0);
    NVIC_EnableIRQ(LPTIM1_IRQn);

    // 14. Enable Timer
    // Set ENABLE bit in CR register
    LPTIM1->CR |= LPTIM_CR_ENABLE;

    // Wait for the timer to be actually enabled
    while(!(LPTIM1->CR & LPTIM_CR_ENABLE)) {}

    // 15. Start Continuous Counting
    // Set CNTSTRT bit to start counter
    LPTIM1->CR |= LPTIM_CR_CNTSTRT;

}

void set_low_timer_ms() {
	LPTIM1->ARR = 2000; // 1 ms period (adjust as needed)
}
