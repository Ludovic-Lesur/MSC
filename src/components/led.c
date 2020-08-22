/*
 * led.c
 *
 *  Created on: 22 aug. 2020
 *      Author: Ludo
 */

#include "led.h"

#include "gpio.h"
#include "mapping.h"

/*** LED functions ***/

/* SET LED COLOR.
 * @param led_color:	Color to set.
 * @return:				None.
 */
void LED_SetColor(LED_Color led_color) {
	// Configure GPIO as output.
	GPIO_Configure(&GPIO_LED_RED, GPIO_MODE_OUTPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_LED_GREEN, GPIO_MODE_OUTPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_LED_BLUE, GPIO_MODE_OUTPUT, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Set color.
	switch (led_color) {
	case LED_OFF:
		GPIO_Write(&GPIO_LED_RED, 1);
		GPIO_Write(&GPIO_LED_GREEN, 1);
		GPIO_Write(&GPIO_LED_BLUE, 1);
		break;
	case LED_COLOR_RED:
		GPIO_Write(&GPIO_LED_RED, 0);
		GPIO_Write(&GPIO_LED_GREEN, 1);
		GPIO_Write(&GPIO_LED_BLUE, 1);
		break;
	case LED_COLOR_GREEN:
		GPIO_Write(&GPIO_LED_RED, 1);
		GPIO_Write(&GPIO_LED_GREEN, 0);
		GPIO_Write(&GPIO_LED_BLUE, 1);
		break;
	case LED_COLOR_BLUE:
		GPIO_Write(&GPIO_LED_RED, 1);
		GPIO_Write(&GPIO_LED_GREEN, 1);
		GPIO_Write(&GPIO_LED_BLUE, 0);
		break;
	case LED_COLOR_YELLOW:
		GPIO_Write(&GPIO_LED_RED, 0);
		GPIO_Write(&GPIO_LED_GREEN, 0);
		GPIO_Write(&GPIO_LED_BLUE, 1);
		break;
	case LED_COLOR_CYAN:
		GPIO_Write(&GPIO_LED_RED, 1);
		GPIO_Write(&GPIO_LED_GREEN, 0);
		GPIO_Write(&GPIO_LED_BLUE, 0);
		break;
	case LED_COLOR_MAGENTA:
		GPIO_Write(&GPIO_LED_RED, 0);
		GPIO_Write(&GPIO_LED_GREEN, 1);
		GPIO_Write(&GPIO_LED_BLUE, 0);
		break;
	case LED_COLOR_WHITE:
		GPIO_Write(&GPIO_LED_RED, 0);
		GPIO_Write(&GPIO_LED_GREEN, 0);
		GPIO_Write(&GPIO_LED_BLUE, 0);
		break;
	default:
		break;
	}
}

