/*
 * thermistor.c
 *
 *  Created on: May 14, 2017
 *      Author: James
 *		Note: pull up resistors are 40k+-10k
 */

#include "thermistor.h"

#define VCC3V3_nV 3300000000
#define PULLUP_mOhm 10000000

#define USE_THERMISTOR_LOOKUP_TABLE

#ifdef USE_THERMISTOR_LOOKUP_TABLE
const Thermistor_entry_t NTC_10K_3984B_Table[30] = {
	{-40, 334274000},	{-35, 241323000},	{-30, 176133000},	{-25, 129900000},
	{-20, 96761000},	{-15, 72765000},	{-10, 55218000},	{-5, 42268000},
	{0, 32624000},		{5, 25381000},		{10, 19897000},		{15, 15711000},
	{20, 12493000},		{25, 10000000},		{30, 8056000},		{35, 6529700},
	{40, 5323900},		{45, 4365300},		{50, 3598700},		{55, 2982300},
	{60, 2483800},		{65, 2078700},		{70, 1747700},		{75, 1475900},
	{80, 1251800},		{85, 1066100},		{90, 911590},		{95, 782460},
	{100, 674110},		{105, 582840}
};
#endif

uint64_t adc_to_nanovolts(uint32_t s, uint32_t n){ //aggreagate sum | oversampling count
	uint64_t dividend = VCC3V3_nV * s;
	uint64_t divisor = n * 0xfff;
	return (dividend + divisor/2) / divisor;
}

uint32_t nanovolts_to_milliohms(uint64_t v){
	uint64_t dividend = PULLUP_mOhm * v;
	uint64_t divisor = VCC3V3_nV - v;
	return (uint32_t)((dividend + divisor/2) / divisor);
}

const Thermistor_entry_t * lut = NTC_10K_3984B_Table;

uint8_t binarySearchOver(uint32_t mohms, uint8_t low, uint8_t high){ //higher temp ind
	if(high-low == 1) return high;
	uint8_t middleIndex = (low+high)/2;
	if(mohms <= lut[middleIndex].milliohms){
		return binarySearchOver(mohms, middleIndex, high);
	}else{
		return binarySearchOver(mohms, low, middleIndex);
	}
}

int32_t adc_to_milliCelcius(uint32_t s, uint32_t n){
	uint32_t mohms = nanovolts_to_milliohms(adc_to_nanovolts(s,n));
	uint8_t hiIndex = binarySearchOver(mohms,0,29); //higher temp ind
	int32_t lowmC = lut[hiIndex-1].celcius*1000;
	int32_t mCRange = (lut[hiIndex].celcius-lut[hiIndex-1].celcius)*1000;
	int32_t mORange = lut[hiIndex].milliohms-lut[hiIndex-1].milliohms;
	int32_t deltamO = mohms-lut[hiIndex-1].milliohms;
	int64_t dividend = (int64_t)mCRange * deltamO;
	return lowmC + (int32_t)((dividend + mORange/2)/mORange);
}
