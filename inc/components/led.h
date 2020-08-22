/*
 * led.h
 *
 *  Created on: 22 aug 2020
 *      Author: Ludo
 */

#ifndef LED_H
#define LED_H

/*** LED structures ***/

// Color bit masks defined as 0b<B><G><R>
typedef enum {
	LED_OFF = 0b000,
	LED_COLOR_RED = 0b001,
	LED_COLOR_GREEN = 0b010,
	LED_COLOR_YELLOW = 0b011,
	LED_COLOR_BLUE = 0b100,
	LED_COLOR_MAGENTA = 0b101,
	LED_COLOR_CYAN = 0b110,
	LED_COLOR_WHITE = 0b111
} LED_Color;

/*** LED functions ***/

void LED_SetColor(LED_Color led_color);

#endif /* LED_H */
