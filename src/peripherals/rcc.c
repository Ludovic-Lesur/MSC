/*
 * rcc.c
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#include "rcc.h"

#include "gpio.h"
#include "lptim.h"
#include "lptim_reg.h"
#include "mapping.h"
#include "nvic.h"
#include "pwr_reg.h"
#include "rcc_reg.h"
#include "scb_reg.h"

/*** RCC local macros ***/

#define RCC_TIMEOUT_COUNT	1000000

/*** RCC functions ***/

/* CONFIGURE PERIPHERALs CLOCK PRESCALER AND SOURCES.
 * @param:	None.
 * @return:	None.
 */
void RCC_Init(void) {
	// Prescalers (HCLK, PCLK1 and PCLK2 must not exceed 32MHz).
	RCC -> CFGR &= ~(0b1111 << 4); // HCLK = SYSCLK = 16MHz (HPRE='0000').
	RCC -> CFGR &= ~(0b111 << 8); // PCLK1 = HCLK = 16MHz (PPRE1='000').
	RCC -> CFGR &= ~(0b111 << 11); // PCLK2 = HCLK = 16MHz (PPRE2='000').
	// Peripherals clock source.
	RCC -> CCIPR &= 0xFFF0C3F0; // All peripherals clocked via the corresponding APBx line.
	// Unlock back-up registers.
	RCC -> APB1ENR |= (0b1 << 28); // PWREN='1'.
	PWR -> CR |= (0b1 << 8); // Set DBP bit to unlock back-up registers write protection.
}

/* CONTROL EXTERNAL TCXO.
 * @param:	None.
 * @return:	None.
 */
void RCC_Tcxo(unsigned char tcxo_enable) {
	// Turn TCXO on or off.
	if (tcxo_enable != 0) {
		GPIO_Write(&GPIO_TCXO_POWER_ENABLE, 1);
	}
	else {
		GPIO_Write(&GPIO_TCXO_POWER_ENABLE, 0);
	}
}

/* ENABLE TCXO CONTROL PIN.
 * @param:	None.
 * @return:	None.
 */
void RCC_EnableGpio(void) {
	// Configure TCXO power control pin.
	GPIO_Configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_TCXO_POWER_ENABLE, 0);
}

/* DISABLE TCXO CONTROL PIN.
 * @param:	None.
 * @return:	None.
 */
void RCC_DisableGpio(void) {
	// Disable TCXO power control pin.
	GPIO_Configure(&GPIO_TCXO_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
}

/* CONFIGURE AND USE HSI AS SYSTEM CLOCK (16MHz INTERNAL RC).
 * @param:					None.
 * @return sysclk_on_hsi:	'1' if SYSCLK source was successfully switched to HSI, 0 otherwise.
 */
unsigned char RCC_SwitchToHsi(void) {
	// Set flash latency.
	FLASH_SetLatency(1);
	// Init HSI.
	RCC -> CR |= (0b1 << 0); // Enable HSI (HSI16ON='1').
	// Wait for HSI to be stable.
	unsigned char sysclk_on_hsi = 0;
	unsigned int count = 0;
	while ((((RCC -> CR) & (0b1 << 2)) == 0) && (count < RCC_TIMEOUT_COUNT)) {
		count++; // Wait for HSIRDYF='1' or timeout.
	}
	// Check timeout.
	if (count < RCC_TIMEOUT_COUNT) {
		// Switch SYSCLK.
		RCC -> CFGR &= ~(0b11 << 0); // Reset bits 0-1.
		RCC -> CFGR |= (0b01 << 0); // Use HSI as system clock (SW='01').
		// Wait for clock switch.
		count = 0;
		while ((((RCC -> CFGR) & (0b11 << 2)) != (0b01 << 2)) && (count < RCC_TIMEOUT_COUNT)) {
			count++; // Wait for SWS='01' or timeout.
		}
		// Check timeout.
		if (count < RCC_TIMEOUT_COUNT) {
			// Disable MSI and HSE.
			RCC -> CR &= ~(0b1 << 8); // Disable MSI (MSION='0').
			RCC -> CR &= ~(0b1 << 16); // Disable HSE (HSEON='0').
			// Update flag and frequency.
			sysclk_on_hsi = 1;
		}
	}
	return sysclk_on_hsi;
}

/* CONFIGURE AND USE LSI AS LOW SPEED OSCILLATOR (32kHz INTERNAL RC).
 * @param:					None.
 * @return lsi_available:	'1' if LSI was successfully started, 0 otherwise.
 */
unsigned char RCC_EnableLsi(void) {
	// Enable LSI.
	RCC -> CSR |= (0b1 << 0); // LSION='1'.
	// Wait for LSI to be stable.
	unsigned char lsi_available = 0;
	unsigned int count = 0;
	while ((((RCC -> CSR) & (0b1 << 1)) == 0) && (count < RCC_TIMEOUT_COUNT)) {
		count++; // Wait for LSIRDY='1'.
	}
	// Check timeout.
	if (count < RCC_TIMEOUT_COUNT) {
		// Update flag.
		lsi_available = 1;
	}
	return lsi_available;
}
