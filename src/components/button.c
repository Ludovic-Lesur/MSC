/*
 * button.c
 *
 *  Created on: 22 aug. 2020
 *      Author: Ludo
 */

#include "button.h"

#include "exti.h"
#include "gpio.h"
#include "mapping.h"
#include "nvic.h"

/*** BUTTON functions ***/

/* INIT BUTTON PIN.
 * @param:	None.
 * @return:	None.
 */
void BUTTON_Init(void) {
	// Init GPIO.
	GPIO_Configure(&GPIO_BUTTON, GPIO_MODE_INPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Init interrupt.
	EXTI_ConfigureGpio(&GPIO_BUTTON, EXTI_TRIGGER_RISING_EDGE);
}

/* ENABLE BUTTON INTERRUPT.
 * @param:	None.
 * @return:	None.
 */
void BUTTON_EnableInterrupt(void) {
	// Enable interrupt.
	NVIC_EnableInterrupt(NVIC_IT_EXTI_0_1);
}

/* DISABLE BUTTON INTERRUPT.
 * @param:	None.
 * @return:	None.
 */
void BUTTON_DisableInterrupt(void) {
	// Disable interrupt.
	NVIC_DisableInterrupt(NVIC_IT_EXTI_0_1);
}

/* READ BUTTON STATE.
 * @param:	None.
 * @return:	'1' if the button is currently pressed, '0' otherwise.
 */
unsigned char BUTTON_IsPressed(void) {
	return GPIO_Read(&GPIO_BUTTON);
}
