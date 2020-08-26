/*
 * button.h
 *
 *  Created on: 22 aug. 2020
 *      Author: Ludo
 */

#ifndef BUTTON_H
#define BUTTON_H

/*** BUTTON functions ***/

void BUTTON_Init(void);
void BUTTON_EnableInterrupt(void);
void BUTTON_DisableInterrupt(void);
unsigned char BUTTON_IsPressed(void);

#endif /* INC_COMPONENTS_BUTTON_H_ */
