/*
 * rcc.h
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#ifndef RCC_H
#define RCC_H

/*** RCC macros ***/

#define RCC_HSI_FREQUENCY_KHZ	16000
#define RCC_LSI_FREQUENCY_HZ	38000

/*** RCC functions ***/

void RCC_Init(void);
void RCC_Tcxo(unsigned char tcxo_enable);
void RCC_EnableGpio(void);
void RCC_DisableGpio(void);
unsigned char RCC_SwitchToHsi(void);
unsigned char RCC_EnableLsi(void);

#endif /* RCC_H */
