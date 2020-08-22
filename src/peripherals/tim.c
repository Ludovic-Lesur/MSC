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

#define TIM2_PWM_FREQUENCY_HZ			2000
#define TIM2_PWM_DUTY_CYCLE_RANGE		101 // 0 to 100 included.
#define TIM2_NUMBER_OF_CHANNELS			4

#define TIM21_BLINK_LUT_LENGTH_BYTES	(2 * TIM2_PWM_DUTY_CYCLE_RANGE)

/*** TIM local structures ***/

typedef struct {
	unsigned int tim2_ccrx_duty_cycle[TIM2_PWM_DUTY_CYCLE_RANGE];
	unsigned int tim21_channels_mask[TIM2_NUMBER_OF_CHANNELS];
	unsigned int tim21_blink_lut[TIM21_BLINK_LUT_LENGTH_BYTES];
	volatile unsigned int tim21_blink_lut_idx;
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
		TIM2_SetDutyCycle(TIM2_CHANNEL_LED_RED, tim_ctx.tim21_blink_lut[tim_ctx.tim21_blink_lut_idx] & tim_ctx.tim21_channels_mask[TIM2_CHANNEL_LED_RED]);
		TIM2_SetDutyCycle(TIM2_CHANNEL_LED_GREEN, tim_ctx.tim21_blink_lut[tim_ctx.tim21_blink_lut_idx] & tim_ctx.tim21_channels_mask[TIM2_CHANNEL_LED_GREEN]);
		TIM2_SetDutyCycle(TIM2_CHANNEL_LED_BLUE, tim_ctx.tim21_blink_lut[tim_ctx.tim21_blink_lut_idx] & tim_ctx.tim21_channels_mask[TIM2_CHANNEL_LED_BLUE]);
		// Increment index.
		tim_ctx.tim21_blink_lut_idx++;
		if (tim_ctx.tim21_blink_lut_idx >= TIM21_BLINK_LUT_LENGTH_BYTES) {
			tim_ctx.tim21_blink_lut_idx = 0;
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
	TIM2 -> CNT = 0;
	// Set PWM frequency.
	TIM2 -> PSC = ((RCC_GetSysclkKhz() / 1000) - 1); // Timer input clock is 1MHz.
	TIM2 -> ARR = (1000000) / (TIM2_PWM_FREQUENCY_HZ);
	// Build LUT according to ARR value.
	unsigned int duty_cycle = 0;
	for (duty_cycle=0 ; duty_cycle<TIM2_PWM_DUTY_CYCLE_RANGE ; duty_cycle++) {
		tim_ctx.tim2_ccrx_duty_cycle[duty_cycle] = (((TIM2 -> ARR) + 1) * (100 - duty_cycle)) / (100);
	}
	// Configure channels 2-4 in PWM mode 1 (OCxM='110' and OCxPE='1').
	TIM2 -> CCMR1 |= (0b110 << 12) | (0b1 << 11);
	TIM2 -> CCMR2 |= (0b110 << 12) | (0b1 << 11) | (0b110 << 4) | (0b1 << 3);
	// Enable channels 2-4 (CCxE='1').
	TIM2 -> CCER |= (0b1 << 4) | (0b1 << 8) | (0b1 << 12);
	// Reset duty cycles to 0.
	TIM2_SetDutyCycle(TIM2_CHANNEL_LED_RED, 0);
	TIM2_SetDutyCycle(TIM2_CHANNEL_LED_GREEN, 0);
	TIM2_SetDutyCycle(TIM2_CHANNEL_LED_BLUE, 0);
	// Generate event to update registers.
	TIM2 -> EGR |= (0b1 << 0); // UG='1'.
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
	TIM2 -> CR1 |= (0b1 << 0); // CEN='1'.
}

/* START PWM GENERATION.
 * @param:	None.
 * @return:	None.
 */
void TIM2_Stop(void) {
	// Disable and reset counter.
	TIM2 -> CR1 &= ~(0b1 << 0); // CEN='0'.
	TIM2 -> CNT = 0;
}

/* SET PWM DUTY CYCLE.
 * @param channel:		Timer channel to set.
 * @param duty_cycle:	Signal duty cycle in % (between 0 and 100).
 * @return:				None.
 */
void TIM2_SetDutyCycle(TIM2_Channel channel, unsigned int duty_cycle) {
	// Set channel 1 duty cycle.
	TIM2 -> CCRx[channel] = tim_ctx.tim2_ccrx_duty_cycle[duty_cycle];
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
	// Build blink LUT.
	unsigned int idx = 0;
	for (idx=0 ; idx<TIM21_BLINK_LUT_LENGTH_BYTES ; idx++) {
		tim_ctx.tim21_blink_lut[idx] = (idx < TIM2_PWM_DUTY_CYCLE_RANGE) ? idx : (TIM2_PWM_DUTY_CYCLE_RANGE - 1) - (idx - TIM2_PWM_DUTY_CYCLE_RANGE);
	}
	tim_ctx.tim21_blink_lut_idx = 0;
	for (idx=0 ; idx<TIM2_NUMBER_OF_CHANNELS ; idx++) {
		tim_ctx.tim21_channels_mask[idx] = 0;
	}
	// Configure period.
	TIM21 -> PSC = (RCC_GetSysclkKhz()) / (10); // Timer is clocked by SYSCLK (see RCC_Init() function).
	TIM21 -> ARR = (led_blink_period_ms * 10) / (TIM21_BLINK_LUT_LENGTH_BYTES);
	// Generate event to update registers.
	TIM21 -> EGR |= (0b1 << 0); // UG='1'.
	// Enable interrupt.
	TIM21 -> DIER |= (0b1 << 0);
}

/* SET CURRENT LED COLOR.
 * @param led_color:	New LED color.
 * @return:				None.
 */
void TIM21_SetLedColor(TIM2_LedColor led_color) {
	// Update channels mask according to color.
	unsigned char channel_idx = 0;
	for (channel_idx=0 ; channel_idx<TIM2_NUMBER_OF_CHANNELS ; channel_idx++) {
		tim_ctx.tim21_channels_mask[channel_idx] = (led_color & (0b1 << channel_idx)) ? 0xFFFFFFFF : 0;
	}
}

/* ENABLE TIM21 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void TIM21_Start(void) {
	// Reset LUT index.
	tim_ctx.tim21_blink_lut_idx = 0;
	// Clear flag and enable interrupt.
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

