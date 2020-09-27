/*
 * adc.h
 *
 *  Created on: 16 aug. 2020
 *      Author: Ludo
 */

#ifndef ADC_H
#define ADC_H

/*** ADC functions ***/

void ADC1_Init(void);
void ADC1_Disable(void);
void ADC1_PerformMeasurements(void);
void ADC1_GetSolarVoltage(unsigned int* solar_voltage_mv);
void ADC1_GetOutputVoltage(unsigned int* output_voltage_mv);
void ADC1_GetOutputCurrent(unsigned int* output_current_ua);
void ADC1_GetMcuVoltage(unsigned int* supply_voltage_mv);
void ADC1_GetMcuTemperatureComp2(signed char* mcu_temperature_degrees);
void ADC1_GetMcuTemperatureComp1(unsigned char* mcu_temperature_degrees);

#endif /* ADC_H */
