/*
 * thermistor.h
 *
 * Created on: May 14, 2016
 *     Author: jamesliu
 */

#ifndef THERMISTOR_H_
#define THERMISTOR_H_

#include "main.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

typedef struct
{
int8_t celcius;
uint32_t milliohms;
}Thermistor_entry_t;

uint64_t adc_to_nanovolts(uint32_t s, uint32_t n);
uint32_t nanovolts_to_milliohms(uint64_t v);
uint8_t binarySearchUnder(uint32_t mohms, uint8_t low, uint8_t high);
int32_t adc_to_milliCelcius(uint32_t s, uint32_t n);

#endif /* THERMISTOR_H_ */
