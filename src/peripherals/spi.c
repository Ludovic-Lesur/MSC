/*
 * spi.c
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#include "spi.h"

#include "gpio.h"
#include "lptim.h"
#include "mapping.h"
#include "rcc.h"
#include "rcc_reg.h"
#include "spi_reg.h"

/*** SPI local macros ***/

#define SPI_ACCESS_TIMEOUT_COUNT	1000000

/*** SPI functions ***/

/* CONFIGURE SPI1.
 * @param:	None.
 * @return:	None.
 */
void SPI1_Init(void) {
	// Enable peripheral clock.
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.
	// Configure power enable pins.
	GPIO_Configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_POWER_ENABLE, 0);
	// Configure SCK, MISO and MOSI (first as high impedance).
	GPIO_Configure(&GPIO_SPI1_SCK, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_Configure(&GPIO_SPI1_MOSI, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_Configure(&GPIO_SPI1_MISO, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	// Configure CS pins (first as output low).
	GPIO_Configure(&GPIO_S2LP_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_S2LP_CS, 0);
	// Configure peripheral.
	SPI1 -> CR1 &= 0xFFFF0000; // Disable peripheral before configuration (SPE='0').
	SPI1 -> CR1 |= (0b1 << 2); // Master mode (MSTR='1').
	// Set baud rate according to system clock.
	SPI1 -> CR1 &= ~(0b111 << 3); // Baud rate = PCLK2/2 = SYSCLK/2 = 8MHz.
	SPI1 -> CR1 &= ~(0b1 << 11); // 8-bits format (DFF='0').
	SPI1 -> CR1 &= ~(0b11 << 0); // CPOL='0' and CPHA='0'.
	SPI1 -> CR2 &= 0xFFFFFF08;
	SPI1 -> CR2 |= (0b1 << 2); // Enable output (SSOE='1').
	SPI1 -> CR2 |= (0b1 << 1); // Enable TX DMA requests.
	// Enable peripheral.
	SPI1 -> CR1 |= (0b1 << 6); // SPE='1'.
}

/* ENABLE SPI1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void SPI1_Enable(void) {
	// Enable SPI1 peripheral.
	RCC -> APB2ENR |= (0b1 << 12); // SPI1EN='1'.
	SPI1 -> CR1 |= (0b1 << 6);
	// Configure power enable pins.
	GPIO_Configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_RF_POWER_ENABLE, 0);
	// Configure CS pins (first as output low).
	GPIO_Configure(&GPIO_S2LP_CS, GPIO_MODE_OUTPUT, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	GPIO_Write(&GPIO_S2LP_CS, 0);
}

/* DISABLE SPI1 PERIPHERAL.
 * @param:	None.
 * @return:	None.
 */
void SPI1_Disable(void) {
	// Disable power control pin.
	GPIO_Configure(&GPIO_RF_POWER_ENABLE, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable CS pin.
	GPIO_Configure(&GPIO_S2LP_CS, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_NONE);
	// Disable SPI1 peripheral.
	SPI1 -> CR1 &= ~(0b1 << 6);
	// Clear all flags.
	SPI1 -> SR &= 0xFFFFFEEF;
	// Disable peripheral clock.
	RCC -> APB2ENR &= ~(0b1 << 12); // SPI1EN='0'.
}

/* SWITCH ALL SPI1 SLAVES ON.
 * @param:	None.
 * @return:	None.
 */
void SPI1_PowerOn(void) {
	// Turn S2LP on.
	GPIO_Write(&GPIO_RF_POWER_ENABLE, 1);
	GPIO_Write(&GPIO_S2LP_CS, 1); // CS high (idle state).
	// Enable GPIOs.
	GPIO_Configure(&GPIO_SPI1_SCK, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_SPI1_MOSI, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	GPIO_Configure(&GPIO_SPI1_MISO, GPIO_MODE_ALTERNATE_FUNCTION, GPIO_TYPE_PUSH_PULL, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
	// Wait for power-on.
	LPTIM1_DelayMilliseconds(100);
}

/* SWITCH ALL SPI1 SLAVES OFF.
 * @param:	None.
 * @return:	None.
 */
void SPI1_PowerOff(void) {
	// Turn S2LP off.
	GPIO_Write(&GPIO_RF_POWER_ENABLE, 0);
	GPIO_Write(&GPIO_S2LP_CS, 0); // CS low (to avoid powering slaves via SPI bus).
	// Disable SPI alternate function.
	GPIO_Configure(&GPIO_SPI1_SCK, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_Configure(&GPIO_SPI1_MOSI, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
	GPIO_Configure(&GPIO_SPI1_MISO, GPIO_MODE_ANALOG, GPIO_TYPE_OPEN_DRAIN, GPIO_SPEED_LOW, GPIO_PULL_DOWN);
}

/* SEND A BYTE THROUGH SPI1.
 * @param tx_data:	Data to send (8-bits).
 * @return:			1 in case of success, 0 in case of failure.
 */
unsigned char SPI1_WriteByte(unsigned char tx_data) {
	// Send data.
	*((volatile unsigned char*) &(SPI1 -> DR)) = tx_data;
	// Wait for transmission to complete.
	unsigned int loop_count = 0;
	while ((((SPI1 -> SR) & (0b1 << 1)) == 0) || (((SPI1 -> SR) & (0b1 << 7)) != 0)) {
		// Wait for TXE='1' and BSY='0' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) return 0;
	}
	return 1;
}

/* READ A BYTE FROM SPI1.
 * @param rx_data:	Pointer to byte that will contain the data to read (8-bits).
 * @return:			1 in case of success, 0 in case of failure.
 */
unsigned char SPI1_ReadByte(unsigned char tx_data, unsigned char* rx_data) {
	// Dummy read to DR to clear RXNE flag.
	(*rx_data) = *((volatile unsigned char*) &(SPI1 -> DR));
	// Send dummy data on MOSI to generate clock.
	*((volatile unsigned char*) &(SPI1 -> DR)) = tx_data;
	// Wait for incoming data.
	unsigned int loop_count = 0;
	while (((SPI1 -> SR) & (0b1 << 0)) == 0) {
		// Wait for RXNE='1' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) return 0;
	}
	(*rx_data) = *((volatile unsigned char*) &(SPI1 -> DR));
	// Wait for reception to complete.
	loop_count = 0;
	while ((((SPI1 -> SR) & (0b1 << 0)) != 0) || (((SPI1 -> SR) & (0b1 << 7)) != 0)) {
		// Wait for RXNE='0' and BSY='0' or timeout.
		loop_count++;
		if (loop_count > SPI_ACCESS_TIMEOUT_COUNT) return 0;
	}
	return 1;
}
