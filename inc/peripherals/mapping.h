/*
 * mapping.h
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#ifndef MAPPING_H
#define MAPPING_H

#include "gpio.h"
#include "gpio_reg.h"

#ifdef HW1_0
// Button.
static const GPIO GPIO_BUTTON =					(GPIO) {GPIOA, 0, 0, 0};
// RGB LED.
static const GPIO GPIO_LED_RED =				(GPIO) {GPIOA, 0, 1, 2};
static const GPIO GPIO_LED_GREEN =				(GPIO) {GPIOA, 0, 2, 2};
static const GPIO GPIO_LED_BLUE =				(GPIO) {GPIOA, 0, 3, 2};
// Analog inputs.
static const GPIO GPIO_ADC1_IN4 =				(GPIO) {GPIOA, 0, 4, 0};
static const GPIO GPIO_ADC1_IN5 =				(GPIO) {GPIOA, 0, 5, 0};
static const GPIO GPIO_ADC1_IN6 =				(GPIO) {GPIOA, 0, 6, 0};
static const GPIO GPIO_ADC1_IN7 =				(GPIO) {GPIOA, 0, 7, 0};
// TCXO power control.
static const GPIO GPIO_TCXO_POWER_ENABLE =		(GPIO) {GPIOA, 0, 8, 0};
// S2LP GPIO.
static const GPIO GPIO_S2LP_GPIO0 =				(GPIO) {GPIOA, 0, 12, 0};
// Programming.
static const GPIO GPIO_SWDIO =					(GPIO) {GPIOA, 0, 13, 0};
static const GPIO GPIO_SWCLK =					(GPIO) {GPIOA, 0, 14, 0};
// RF power enable.
static const GPIO GPIO_RF_POWER_ENABLE =		(GPIO) {GPIOB, 1, 0, 0};
// DC-DC enable.
static const GPIO GPIO_DCDC_POWER_ENABLE =		(GPIO) {GPIOB, 1, 1, 0};
// SPI1.
static const GPIO GPIO_SPI1_SCK = 				(GPIO) {GPIOB, 1, 3, 0};
static const GPIO GPIO_SPI1_MISO = 				(GPIO) {GPIOB, 1, 4, 0};
static const GPIO GPIO_SPI1_MOSI = 				(GPIO) {GPIOB, 1, 5, 0};
static const GPIO GPIO_S2LP_CS = 				(GPIO) {GPIOA, 0, 15, 0};
// USART2.
static const GPIO GPIO_USART2_TX =				(GPIO) {GPIOB, 1, 6, 0};
static const GPIO GPIO_USART2_RX =				(GPIO) {GPIOB, 1, 7, 0};
// Test points.
static const GPIO GPIO_TP1 =					(GPIO) {GPIOB, 1, 2, 0};
static const GPIO GPIO_TP2 =					(GPIO) {GPIOA, 0, 10, 0};
#endif

#endif /* MAPPING_H */
