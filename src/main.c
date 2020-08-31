/*
 * main.c
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

// Registers
#include "rcc_reg.h"
// Peripherals.
#include "adc.h"
#include "aes.h"
#include "dma.h"
#include "exti.h"
#include "gpio.h"
#include "iwdg.h"
#include "lptim.h"
#include "mapping.h"
#include "nvic.h"
#include "nvm.h"
#include "pwr.h"
#include "rcc.h"
#include "spi.h"
#include "tim.h"
#include "rtc.h"
#include "usart.h"
// Components.
#include "button.h"
#include "led.h"
#include "s2lp.h"
#include "sigfox_types.h"
#include "sigfox_api.h"
// Applicative.
#include "at.h"
#include "mode.h"
#include "monitoring.h"
#include "sigfox_api.h"

/*** MAIN macros ***/

#define MSC_SIGFOX_PERIOD_SECONDS				600
#define MSC_SIGFOX_UPLINK_DATA_LENGTH_BYTES		10

#define MSC_LED_BLINK_PERIOD_MS					2000

#define MSC_CURRENT_THRESHOLD_LOW_UA			50000
#define MSC_CURRENT_THRESHOLD_HIGH_UA			300000

/*** MAIN structures ***/

#ifdef NM
// State machine.
typedef enum {
	MSC_STATE_MEASURE,
	MSC_STATE_SIGFOX,
	MSC_STATE_LED,
	MSC_STATE_SLEEP,
} MSC_State;

// Sigfox uplink data format.
typedef union {
	unsigned char raw_frame[MSC_SIGFOX_UPLINK_DATA_LENGTH_BYTES];
	struct {
		unsigned solar_voltage_mv : 16;
		unsigned output_voltage_mv : 16;
		unsigned output_current_ua : 24;
		unsigned mcu_voltage_mv : 16;
		unsigned mcu_temperature_degrees : 8;
	} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
} MSC_SigfoxUplinkData;

// Device context.
typedef struct {
	// State machine.
	MSC_State msc_state;
	unsigned int msc_sigfox_timer_seconds;
	// Data.
	unsigned int msc_solar_voltage_mv;
	unsigned int msc_output_voltage_mv;
	unsigned int msc_output_current_ua;
	unsigned int msc_mcu_voltage_mv;
	unsigned char msc_mcu_temperature_degrees;
	// Sigfox.
	MSC_SigfoxUplinkData msc_sigfox_uplink_data;
	unsigned char msc_sigfox_downlink_data[SFX_DOWNLINK_DATA_SIZE_BYTES];
} MSC_Context;
#endif

/*** MAIN global variables ***/

#ifdef NM
static MSC_Context msc_ctx;
#endif

/*** MAIN functions ***/

#ifdef NM
/* MAIN FUNCTION.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init memory.
	NVIC_Init();
	NVM_Enable();
	// Init GPIOs.
	GPIO_Init();
	EXTI_Init();
	// Init clock.
	RCC_Init();
	RCC_SwitchToHsi();
	// Reset RTC before starting oscillators.
	RTC_Reset();
	// Start LSI clock.
	RCC_EnableLsi();
	// Init RTC and timers.
	RTC_Init();
	LPTIM1_Init();
	TIM2_Init();
	TIM21_Init(MSC_LED_BLINK_PERIOD_MS);
	// Unused communication interfaces.
	USART2_Init();
	// Components.
	BUTTON_Init();
	LED_SetColor(LED_OFF);
	// Init context.
	msc_ctx.msc_state = MSC_STATE_MEASURE;
	msc_ctx.msc_sigfox_timer_seconds = MSC_SIGFOX_PERIOD_SECONDS; // To send uplink at start-up.
	// Local variables.
	signed char mcu_temperature_degrees = 0;
	unsigned char mcu_temperature_abs = 0;
	sfx_rc_t tkfx_sigfox_rc = (sfx_rc_t) RC1;
	unsigned int sfx_error = 0;
	// Perform state machine.
	while (1) {
		switch (msc_ctx.msc_state) {
		case MSC_STATE_MEASURE:
			// Perform all analog measurements.
			ADC1_Init();
			ADC1_PerformMeasurements();
			ADC1_Disable();
			ADC1_GetSolarVoltage(&msc_ctx.msc_solar_voltage_mv);
			ADC1_GetOutputVoltage(&msc_ctx.msc_output_voltage_mv);
			ADC1_GetOutputCurrent(&msc_ctx.msc_output_current_ua);
			ADC1_GetMcuVoltage(&msc_ctx.msc_mcu_voltage_mv);
			ADC1_GetMcuTemperature(&mcu_temperature_degrees);
			msc_ctx.msc_mcu_temperature_degrees = 0;
			if (mcu_temperature_degrees < 0) {
				msc_ctx.msc_mcu_temperature_degrees |= 0x80;
				mcu_temperature_abs = (-1) * (mcu_temperature_degrees);
				msc_ctx.msc_mcu_temperature_degrees |= (mcu_temperature_abs & 0x7F);
			}
			else {
				msc_ctx.msc_mcu_temperature_degrees = mcu_temperature_degrees & 0x7F;
			}
			// Compute next state.
			if (msc_ctx.msc_sigfox_timer_seconds >= MSC_SIGFOX_PERIOD_SECONDS) {
				msc_ctx.msc_state = MSC_STATE_SIGFOX;
			}
			else {
				msc_ctx.msc_state = MSC_STATE_LED;
			}
			break;
		case MSC_STATE_SIGFOX:
			IWDG_Reload();
			// Map fields.
			msc_ctx.msc_sigfox_uplink_data.field.solar_voltage_mv = msc_ctx.msc_solar_voltage_mv;
			msc_ctx.msc_sigfox_uplink_data.field.output_voltage_mv = msc_ctx.msc_output_voltage_mv;
			msc_ctx.msc_sigfox_uplink_data.field.output_current_ua = msc_ctx.msc_output_current_ua;
			msc_ctx.msc_sigfox_uplink_data.field.mcu_voltage_mv = msc_ctx.msc_mcu_voltage_mv;
			msc_ctx.msc_sigfox_uplink_data.field.mcu_temperature_degrees = msc_ctx.msc_mcu_temperature_degrees;
			// Stop RTC timer during uplink sequence.
			RTC_StopWakeUpTimer();
			// Send message.
			sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
			if (sfx_error == SFX_ERR_NONE) {
				sfx_error = SIGFOX_API_send_frame(msc_ctx.msc_sigfox_uplink_data.raw_frame, MSC_SIGFOX_UPLINK_DATA_LENGTH_BYTES, msc_ctx.msc_sigfox_downlink_data, 2, 0);
			}
			SIGFOX_API_close();
			// Reset Sigfox timer.
			msc_ctx.msc_sigfox_timer_seconds = 0;
			// Turn peripherals off.
			LPTIM1_Disable();
			// Compute next state.
			msc_ctx.msc_state = MSC_STATE_LED;
			break;
		case MSC_STATE_LED:
			IWDG_Reload();
			// Init required peripheral.
			TIM2_Enable();
			TIM21_Enable();
			// Set color according to thresholds.
			if (msc_ctx.msc_output_current_ua < MSC_CURRENT_THRESHOLD_LOW_UA) {
				// Low range.
				TIM2_SetLedColor(TIM2_CHANNEL_MASK_LED_GREEN);
			}
			else {
				if (msc_ctx.msc_output_current_ua < MSC_CURRENT_THRESHOLD_HIGH_UA) {
					// Mid range.
					TIM2_SetLedColor(TIM2_CHANNEL_MASK_LED_YELLOW);
				}
				else {
					// High range.
					TIM2_SetLedColor(TIM2_CHANNEL_MASK_LED_RED);
				}
			}
			// Start blink.
			TIM2_Start();
			TIM21_Start(1);
			// Wait the end of blink.
			while (TIM21_IsSingleBlinkDone() == 0) {
				// Keep managing RTC.
				if (RTC_GetWakeUpTimerFlag() != 0) {
					// Increment timer.
					RTC_ClearWakeUpTimerFlag();
					msc_ctx.msc_sigfox_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
				}
			}
			// Turn peripherals off.
			TIM2_Disable();
			TIM21_Disable();
			LED_SetColor(LED_OFF);
			LPTIM1_Disable();
			// Compute next state.
			msc_ctx.msc_state = MSC_STATE_SLEEP;
			break;
		case MSC_STATE_SLEEP:
			IWDG_Reload();
			// Start RTC.
			RTC_StartWakeUpTimer(RTC_WAKEUP_PERIOD_SECONDS);
			// Enter sleep mode.
			PWR_EnterStopMode();
			// Check wake-up source.
			if (RTC_GetWakeUpTimerFlag() != 0) {
				// Increment timer.
				RTC_ClearWakeUpTimerFlag();
				msc_ctx.msc_sigfox_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
				// Init delay timer.
				LPTIM1_Enable();
				// Wake-up device.
				msc_ctx.msc_state = MSC_STATE_MEASURE;
			}
			break;
		default:
			// Unknown state.
			msc_ctx.msc_state = MSC_STATE_SLEEP;
			break;
		}
	}
	return 0;
}
#endif

#ifdef ATM
/* MAIN FUNCTION FOR AT MODE.
 * @param: 	None.
 * @return: 0.
 */
int main (void) {
	// Init memory.
	NVIC_Init();
	NVM_Enable();
	// Init GPIOs.
	GPIO_Init();
	EXTI_Init();
	// Init clock.
	RCC_Init();
	RCC_EnableLsi();
	RCC_SwitchToHsi();
	// Init peripherals.
	LPTIM1_Init();
	ADC1_Init();
	USART2_Init();
	// Applicative layers.
	AT_Init();
	// Main loop.
	while (1) {
		AT_Task();
	}
	return 0;
}
#endif

