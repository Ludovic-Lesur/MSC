/*
 * dma.h
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#ifndef DMA_H
#define DMA_H

/*** DMA functions ***/

void DMA1_InitChannel3(void);
void DMA1_StartChannel3(void);
void DMA1_StopChannel3(void);
void DMA1_SetChannel3SourceAddr(unsigned int source_buf_addr, unsigned short source_buf_size);
unsigned char DMA1_GetChannel3Status(void);
void DMA1_Disable(void);

#endif /* DMA_H */
