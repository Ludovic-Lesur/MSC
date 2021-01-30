/*
 * nvic.c
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#include "nvic.h"

#include "nvic_reg.h"
#include "scb_reg.h"

/*** NVIC local global variables ***/

extern unsigned int __Vectors;

/*** NVIC functions ***/

/* INIT VECTOR TABLE ADDRESS.
 * @param:	None.
 * @return:	None.
 */
void NVIC_Init(void) {
	SCB -> VTOR = (unsigned int) &__Vectors;
}

/* ENABLE AN INTERRUPT LINE.
 * @param it_num: 	Interrupt number (use enum defined in 'nvic.h').
 * @return: 		None.
 */
void NVIC_EnableInterrupt(NVIC_InterruptVector it_num) {
	NVIC -> ISER = (0b1 << (it_num & 0x1F));
}

/* DISABLE AN INTERRUPT LINE.
 * @param it_num: 	Interrupt number (use enum defined in 'nvic.h').
 * @return:			None.
 */
void NVIC_DisableInterrupt(NVIC_InterruptVector it_num) {
	NVIC -> ICER = (0b1 << (it_num & 0x1F));
}

/* SET THE PRIORITY OF AN INTERRUPT LINE.
 * @param it_num:	Interrupt number (use enum defined in 'nvic.h').
 * @param priority:	Interrupt priority (0 to 3).
 */
void NVIC_SetPriority(NVIC_InterruptVector it_num, unsigned char priority) {
	// Check parameter.
	if ((priority >= NVIC_PRIORITY_MAX) && (priority <= NVIC_PRIORITY_MIN)) {
		// Reset bits.
		NVIC -> IPR[(it_num >> 2)] &= ~(0xFF << (8 * (it_num % 4)));
		// Set priority.
		NVIC -> IPR[(it_num >> 2)] |= ((priority << 6) << (8 * (it_num % 4)));
	}
}
