/*
 * tim.c
 *
 *  Created on: 22 aug. 2020
 *      Author: Ludo
 */

#include "tim.h"

#include "mapping.h"
#include "nvic.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "tim_reg.h"

/*** TIM local macros ***/

#define TIM2_PWM_FREQUENCY_HZ			10000
#define TIM2_ARR_VALUE					((RCC_HSI_FREQUENCY_KHZ * 1000) / TIM2_PWM_FREQUENCY_HZ)
#define TIM2_NUMBER_OF_CHANNELS			4
#define TIM21_DIMMING_LUT_LENGTH		(TIM2_ARR_VALUE + 2) // 2 more values to store 0 and (ARR+1).
#define TIM21_DIMMING_DELTA_MAX_PERCENT	30

/*** TIM local structures ***/

typedef struct {
	unsigned short tim21_dimming_lut[TIM21_DIMMING_LUT_LENGTH];
	volatile unsigned int tim21_dimming_lut_idx;
	volatile unsigned char tim21_dimming_lut_direction;
	unsigned char tim21_single_blink;
	volatile unsigned char tim21_single_blink_done;
} TIM_Context;

/*** TIM local global variables ***/

// Duty cycle to CCRx value LUT.
static TIM_Context tim_ctx;

/*** TIM local functions ***/

/* TIM21 INTERRUPT HANDLER.
 * @param:	None.
 * @return:	None.
 */
void TIM21_IRQHandler(void) {
	if (((TIM21 -> SR) & (0b1 << 0)) != 0) {
		// Update duty cycles.
		TIM2 -> CCRx[TIM2_CHANNEL_LED_RED] = tim_ctx.tim21_dimming_lut[tim_ctx.tim21_dimming_lut_idx];
		TIM2 -> CCRx[TIM2_CHANNEL_LED_GREEN] = tim_ctx.tim21_dimming_lut[tim_ctx.tim21_dimming_lut_idx];
		TIM2 -> CCRx[TIM2_CHANNEL_LED_BLUE] = tim_ctx.tim21_dimming_lut[tim_ctx.tim21_dimming_lut_idx];
		// Manage index and direction.
		if (tim_ctx.tim21_dimming_lut_direction == 0) {
			// Increment index.
			tim_ctx.tim21_dimming_lut_idx++;
			// Invert direction at end of table.
			if (tim_ctx.tim21_dimming_lut_idx >= (TIM21_DIMMING_LUT_LENGTH - 1)) {
				tim_ctx.tim21_dimming_lut_direction = 1;
			}
		}
		else {
			// Decrement index.
			tim_ctx.tim21_dimming_lut_idx--;
			// Invert direction at the beginning of table.
			if (tim_ctx.tim21_dimming_lut_idx == 0) {
				tim_ctx.tim21_dimming_lut_direction = 0;
				// Auto stop in single mode.
				if (tim_ctx.tim21_single_blink != 0) {
					TIM2_Stop();
					TIM21_Stop();
					tim_ctx.tim21_single_blink_done = 1;
				}
			}
		}
		// Clear flag.
		TIM21 -> SR &= ~(0b1 << 0);
	}
}

/*** TIM functions ***/

/* INIT TIM2 FOR PWM OPERATION.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Init(void) {
	// Enable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 0); // TIM2EN='1'.
	// Configure peripheral.
	TIM2 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM2 -> CNT &= 0xFFFF0000;
	// Set PWM frequency.
	TIM2 -> PSC &= 0xFFFF0000; // Timer input clock is SYSCLK.
	TIM2 -> ARR &= 0xFFFF0000;
	TIM2 -> ARR |= TIM2_ARR_VALUE;
	// Configure channels 2-4 in PWM mode 1 (OCxM='110' and OCxPE='1').
	TIM2 -> CCMR1 |= (0b110 << 12) | (0b1 << 11);
	TIM2 -> CCMR2 |= (0b110 << 12) | (0b1 << 11) | (0b110 << 4) | (0b1 << 3);
	// Disable channels 2-4 by default (CCxE='0').
	TIM2 -> CCER &= 0xFFFFEEEE;
	// Generate event to update registers.
	TIM2 -> EGR |= (0b1 << 0); // UG='1'.
}

/* ENABLE TIM2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Enable(void) {
	// Disable peripheral clock.
	RCC -> APB1ENR |= (0b1 << 0); // TIM2EN='1'.
}


/* DISABLE TIM2 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Disable(void) {
	// Disable peripheral clock.
	RCC -> APB1ENR &= ~(0b1 << 0); // TIM2EN='0'.
}

/* SET CURRENT LED COLOR.
 * @param led_color:	New LED color.
 * @return:				None.
 */
void TIM2_SetLedColor(TIM2_LedColor led_color) {
	// Reset bits.
	TIM2 -> CCER &= 0xFFFFEEEE;
	// Enable channels according to color.
	unsigned char idx = 0;
	for (idx=0 ; idx<TIM2_NUMBER_OF_CHANNELS ; idx++) {
		if ((led_color & (0b1 << idx)) != 0) {
			TIM2 -> CCER |= (0b1 << (4 * idx));
		}
	}
}

/* START PWM GENERATION.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Start(void) {
	// Link GPIOs to timer.
	GPIO_Configure(&GPIO_LED_RED, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_LED_GREEN, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_LED_BLUE, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Enable counter.
	TIM2 -> CNT &= 0xFFFF0000;
	TIM2 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/* STOP PWM GENERATION.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Stop(void) {
	// Disable all channels.
	TIM2 -> CCER &= 0xFFFFEEEE;
	// Disable and reset counter.
	TIM2 -> CR1 &= ~(0b1 << 0); // CEN='0'.
}

/* INIT TIM21 FOR LED BLINKING OPERATION.
 * @param led_blink_period_ms:	LED blink period in ms.
 * @return:						None.
 */
void TIM21_Init(unsigned int led_blink_period_ms) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.
	// Reset timer before configuration.
	TIM21 -> CR1 &= ~(0b1 << 0); // Disable TIM21 (CEN='0').
	TIM21 -> CNT &= 0xFFFF0000; // Reset counter.
	TIM21 -> SR &= 0xFFFFF9B8; // Clear all flags.
	// Reset index.
	tim_ctx.tim21_dimming_lut_idx = 0;
	tim_ctx.tim21_dimming_lut_direction = 0;
	unsigned int idx = 0;
	unsigned int delta = 0;
	unsigned int delta_max = ((TIM21_DIMMING_DELTA_MAX_PERCENT * TIM2_ARR_VALUE) / (100));
	unsigned int delta_max_idx = (TIM21_DIMMING_LUT_LENGTH / 2);
	// Build dimming LUT.
	for (idx=0 ; idx<TIM21_DIMMING_LUT_LENGTH ; idx++) {
		tim_ctx.tim21_dimming_lut[idx] = (TIM2_ARR_VALUE + 1 - idx);
		// Add delta to create a pseudo-exponential curve.
		delta = (idx < delta_max_idx) ? ((delta_max * idx) / (delta_max_idx)) : ((2 * delta_max * (TIM21_DIMMING_LUT_LENGTH - 1 - idx)) / (TIM21_DIMMING_LUT_LENGTH - 1));
		tim_ctx.tim21_dimming_lut[idx] += delta;
	}
	// Configure period.
	TIM21 -> PSC = ((RCC_HSI_FREQUENCY_KHZ / 1000) - 1); // Timer is clocked @1MHz.
	TIM21 -> ARR = (led_blink_period_ms * 1000) / (2 * TIM21_DIMMING_LUT_LENGTH);
	// Generate event to update registers.
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
	// Enable interrupt.
	TIM21 -> DIER |= (0b1 << 0);
}

/* ENABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Enable(void) {
	// Disable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 2); // TIM21EN='1'.
}

/* DISABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Disable(void) {
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 2); // TIM21EN='0'.
}

/* ENABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Start(unsigned char single_blink) {
	// Set mode and reset LUT index.
	tim_ctx.tim21_single_blink = single_blink;
	tim_ctx.tim21_single_blink_done = 0;
	tim_ctx.tim21_dimming_lut_idx = 0;
	tim_ctx.tim21_dimming_lut_direction = 0;
	// Clear flag and enable interrupt.
	TIM21 -> CNT &= 0xFFFF0000;
	TIM21 -> SR &= ~(0b1 << 0); // Clear flag (UIF='0').
	NVIC_EnableInterrupt(IT_TIM21);
	// Enable TIM21 peripheral.
	TIM21 -> CR1 |= (0b1 << 0); // Enable TIM21 (CEN='1').
}

/* STOP TIM21 COUNTER.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Stop(void) {
	// Disable interrupt.
	NVIC_DisableInterrupt(IT_TIM21);
	// Stop TIM21.
	TIM21 -> CR1 &= ~(0b1 << 0); // CEN='0'.
}

/* GET SINGLE BLINK STATUS.
 * @param:							None.
 * @return tim21_single_blink_done: '1' if the single blink is finished, '0' otherwise.
 */
unsigned char TIM21_IsSingleBlinkDone(void) {
	return (tim_ctx.tim21_single_blink_done);
}
