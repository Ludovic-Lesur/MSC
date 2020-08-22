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

#define MSC_LED_BLINK_PERIOD_MS					1000

/*** MAIN structures ***/

// State machine.
typedef enum {
	MSC_STATE_INIT,
	MSC_STATE_MEASURE,
	MSC_STATE_SIGFOX,
	MSC_STATE_OFF,
	MSC_STATE_SLEEP,
} MSC_State;

// Sigfox uplink data format.
typedef struct {
	union {
		unsigned char raw_frame[MSC_SIGFOX_UPLINK_DATA_LENGTH_BYTES];
		struct {
			unsigned solar_voltage_mv : 16;
			unsigned output_voltage_mv : 16;
			unsigned output_current_ua : 24;
			unsigned mcu_voltage_mv : 16;
			unsigned mcu_temperature_degrees : 8;
		} __attribute__((scalar_storage_order("big-endian"))) __attribute__((packed)) field;
	};
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

/*** MAIN global variables ***/

static MSC_Context msc_ctx;

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
	// Init clock.
	RCC_Init();
	RCC_EnableLsi();
	RCC_SwitchToHsi();
	// Init context.
	msc_ctx.msc_state = MSC_STATE_INIT;
	msc_ctx.msc_sigfox_timer_seconds = 0;
	// Local variables.
	signed char mcu_temperature_degrees = 0;
	unsigned char mcu_temperature_abs = 0;
	sfx_rc_t tkfx_sigfox_rc = (sfx_rc_t) RC1;
	unsigned int sfx_error = 0;
	// Perform state machine.
	switch (msc_ctx.msc_state) {
	case MSC_STATE_INIT:
		// Init peripherals.
		RCC_EnableGpio();
		LPTIM1_Init(0);
		TIM2_Init();
		TIM21_Init(MSC_LED_BLINK_PERIOD_MS);
		DMA1_InitChannel3();
		ADC1_Init();
		USART2_Init();
		SPI1_Init();
		AES_Init();
		// Components.
		S2LP_Init();
		// Compute next state.
		msc_ctx.msc_state = MSC_STATE_MEASURE;
		break;
	case MSC_STATE_MEASURE:
		// Perform all analog measurements.
		ADC1_PerformMeasurements();
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
		break;
	case MSC_STATE_SIGFOX:
		// Map fields.
		msc_ctx.msc_sigfox_uplink_data.field.solar_voltage_mv = msc_ctx.msc_solar_voltage_mv;
		msc_ctx.msc_sigfox_uplink_data.field.output_voltage_mv = msc_ctx.msc_output_voltage_mv;
		msc_ctx.msc_sigfox_uplink_data.field.output_current_ua = msc_ctx.msc_output_current_ua;
		msc_ctx.msc_sigfox_uplink_data.field.mcu_voltage_mv = msc_ctx.msc_mcu_voltage_mv;
		msc_ctx.msc_sigfox_uplink_data.field.mcu_temperature_degrees = msc_ctx.msc_mcu_temperature_degrees;
		// Send message.
		sfx_error = SIGFOX_API_open(&tkfx_sigfox_rc);
		if (sfx_error == SFX_ERR_NONE) {
			sfx_error = SIGFOX_API_send_frame(msc_ctx.msc_sigfox_uplink_data.raw_frame, MSC_SIGFOX_UPLINK_DATA_LENGTH_BYTES, msc_ctx.msc_sigfox_downlink_data, 2, 0);
		}
		SIGFOX_API_close();
		// Compute next state.
		msc_ctx.msc_state = MSC_STATE_OFF;
		break;
	case MSC_STATE_OFF:
		// Disable peripherals.
		LPTIM1_Disable();
		DMA1_Disable();
		ADC1_Disable();
		SPI1_Disable();
		AES_Disable();
		// Turn external components off.
		S2LP_DisableGpio();
		RCC_DisableGpio();
		// Clear EXTI flags.
		EXTI_ClearAllFlags();
		RTC_ClearWakeUpTimerFlag();
		// Enable RTC and accelerometer interrupts.
		NVIC_EnableInterrupt(IT_EXTI_0_1);
		RTC_StartWakeUpTimer(RTC_WAKEUP_PERIOD_SECONDS);
		// Enter stop mode.
		msc_ctx.msc_state = MSC_STATE_SLEEP;
		break;
	case MSC_STATE_SLEEP:
		IWDG_Reload();
		// Enter sleep mode.
		PWR_EnterStopMode();
		// Check wake-up source.
		if (RTC_GetWakeUpTimerFlag() != 0) {
			// Increment timers.
			msc_ctx.msc_sigfox_timer_seconds += RTC_WAKEUP_PERIOD_SECONDS;
		}
		// Check period.
		if (msc_ctx.msc_sigfox_timer_seconds >= MSC_SIGFOX_PERIOD_SECONDS) {
			// Reset timer and wake-up device.
			msc_ctx.msc_sigfox_timer_seconds = 0;
			msc_ctx.msc_state = MSC_STATE_INIT;
		}
		break;
	default:
		// Unknown state.
		msc_ctx.msc_state = MSC_STATE_OFF;
		break;
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
	// Init clock.
	RCC_Init();
	RCC_EnableGpio();
	RCC_EnableLsi();
	RCC_SwitchToHsi();
	// Init peripherals.
	LPTIM1_Init(0);
	TIM2_Init();
	TIM21_Init(MSC_LED_BLINK_PERIOD_MS);
	DMA1_InitChannel3();
	ADC1_Init();
	USART2_Init();
	SPI1_Init();
	AES_Init();
	// Components.
	S2LP_Init();
	// Applicative layers.
	AT_Init();
	// Main loop.
	while (1) {
		PWR_EnterLowPowerSleepMode();
		AT_Task();
	}
	return 0;
}
#endif

