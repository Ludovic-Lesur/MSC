/*
 * lptim.h
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#ifndef LPTIM_H
#define LPTIM_H

/*** LPTIM functions ***/

void LPTIM1_Init(unsigned char lptim1_use_lse);
void LPTIM1_Enable(void);
void LPTIM1_Disable(void);
void LPTIM1_DelayMilliseconds(unsigned int delay_ms);

#endif /* LPTIM_H */
