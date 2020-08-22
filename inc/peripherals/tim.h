/*
 * tim.h
 *
 *  Created on: 22 aug. 2020
 *      Author: Ludo
 */

#ifndef TIM_H
#define TIM_H

/*** TIM structures ***/

// Warning: this enum gives the channel index starting from 0.
typedef enum {
	TIM2_CHANNEL_LED_RED = 1, // TIM2_CH2.
	TIM2_CHANNEL_LED_GREEN = 2, // TIM2_CH3.
	TIM2_CHANNEL_LED_BLUE = 3 // TIM2_CH4.
} TIM2_Channel;

// Color bit masks defined as 0b<CH4><CH3><CH2><CH1>
typedef enum {
	TIM2_CHANNEL_MASL_LED_OFF = 0b0000,
	TIM2_CHANNEL_MASK_LED_RED = 0b0010,
	TIM2_CHANNEL_MASK_LED_GREEN = 0b0100,
	TIM2_CHANNEL_MASK_LED_YELLOW = 0b0110,
	TIM2_CHANNEL_MASK_LED_BLUE = 0b1000,
	TIM2_CHANNEL_MASK_LED_MAGENTA = 0b1010,
	TIM2_CHANNEL_MASK_LED_CYAN = 0b1100,
	TIM2_CHANNEL_MASK_LED_WHITE	= 0b1110
} TIM2_LedColor;

/*** TIM functions ***/

void TIM2_Init(void);
void TIM2_Start(void);
void TIM2_Stop(void);
void TIM2_SetDutyCycle(TIM2_Channel channel, unsigned int duty_cycle);

void TIM21_Init(unsigned int led_blink_period_ms);
void TIM21_SetLedColor(TIM2_LedColor led_color);
void TIM21_Start(void);
void TIM21_Stop(void);

#endif /* TIM_H */
