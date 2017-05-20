/*
 * thermistor.c
 *
 *  Created on: May 14, 2017
 *      Author: James
 *		Note: STM32F446 pull up resistors are 40k+-10k
 */

#include "thermistor.h"

#define VCC3V3_nV 3300000000
#define PULLUP_mOhm 10000000

#define USE_THERMISTOR_LOOKUP_TABLE

#ifdef USE_THERMISTOR_LOOKUP_TABLE
const Thermistor_entry_t NTC_4K7_3984B_Table[30] = {
	{-40, 157109000},	{-35, 113422000},	{-30, 82782000},	{-25, 61053000},
	{-20, 45478000}, 	{-15, 34199000},	{-10, 25953000},	{-5, 19866000},
	{0, 15333000}, 		{5, 11929000},		{10, 9351500},		{15, 7384300},
	{20, 5871600}, 		{25, 4700000},		{30, 3786300},		{35, 3069000},
	{40, 2502200}, 		{45, 2051700},		{50, 1691400},		{55, 1401700},
	{60, 1167400}, 		{65, 976970},		{70, 82140},		{75, 693680},
	{80, 588350}, 		{85, 501070},		{90, 428450},		{95, 367750},
	{100, 316830}, 		{105, 273940}
};
const Thermistor_entry_t NTC_5K_3984B_Table[30] = {
	{-40, 167137000}, {-35, 12661000}, {-30, 88066000}, {-25, 64950000},
	{-20, 48381000}, 	{-15, 36382000}, {-10, 27609000}, {-5, 21134000},
	{0, 16312000}, 		{5, 12691000}, 	 {10, 9948400}, 	{15, 7855600},
	{20, 6246400}, 		{25, 5000000}, 	 {30, 4028000}, 	{35, 3264900},
	{40, 2661900}, 		{45, 2182600}, 	 {50, 1799400}, 	{55, 1491100},
	{60, 1241900}, 		{65, 1039300}, 	 {70, 873830}, 	 	{75, 737960},
	{80, 625900}, 		{85, 533050}, 	 {90, 455790}, 	 	{95, 391230},
	{100, 337060}, 		{105, 291420}
};
const Thermistor_entry_t NTC_10k_3435B_Table[30] = {
	{-40, 190953000}, {-35, 145953000}, {-30, 112440000}, {-25, 87285000},
	{-20, 68260000}, 	{-15, 53762000}, 	{-10, 42636000}, 	{-5, 34038000},
	{0, 27348000}, 		{5, 22108000}, 		{10, 17979000}, 	{15, 14706000},
	{20, 12094000}, 	{25, 10000000}, 	{30, 8310800}, 		{35, 6941100},
	{40, 5824900}, 		{45, 4910600}, 		{50, 4158300}, 		{55, 3536200},
	{60, 3019700}, 		{65, 2588800}, 		{70, 2228000}, 		{75, 1924600},
	{80, 1668400}, 		{85, 1451300}, 		{90, 1266700}, 		{95, 1109200},
	{100, 974260}, 		{105, 858330}
};
const Thermistor_entry_t NTC_10K_3984B_Table[30] = {
	{-40, 334274000},	{-35, 241323000},	{-30, 176133000},	{-25, 129900000},
	{-20, 96761000},	{-15, 72765000},	{-10, 55218000},	{-5, 42268000},
	{0, 32624000},		{5, 25381000},		{10, 19897000},		{15, 15711000},
	{20, 12493000},		{25, 10000000},		{30, 8056000},		{35, 6529700},
	{40, 5323900},		{45, 4365300},		{50, 3598700},		{55, 2982300},
	{60, 2483800},		{65, 2078700},		{70, 1747700},		{75, 1475900},
	{80, 1251800},		{85, 1066100},		{90, 911590},			{95, 782460},
	{100, 674110},		{105, 582840}
};
const Thermistor_entry_t NTC_47K_4090B_Table[30] = {
	{-40, 1589068000}, {-35, 1151627000}, {-30, 842790000}, {-25, 622597000},
	{-20, 464110000},  {-15, 348989000}, 	{-10, 264628000}, {-5, 202280000},
	{0, 155823000}, 	 {5, 120932000}, 		{10, 94528000}, 	{15, 74399000},
	{20, 58945000}, 	 {25, 47000000}, 		{30, 37706000}, 	{35, 30429000},
	{40, 24696000}, 	 {45, 20154000}, 		{50, 16534000}, 	{55, 13633000},
	{60, 11296000}, 	 {65, 9404500}, 		{70, 7865200}, 		{75, 6606900},
	{80, 5573500}, 		 {85, 4721000}, 		{90, 4014700}, 		{95, 3427200},
	{100, 2936500}, 	 {105, 2525000}
};
const Thermistor_entry_t NTC_100K_4190B_Table[30] = {
	{-40, 3666299000}, {-35, 2637588000}, {-30, 1916576000}, {-25, 1406111000},
	{-20, 1041184000}, {-15, 777846000}, 	{-10, 586097000},  {-5, 445257000},
	{0, 340942000}, 	 {5, 263054000}, 		{10, 204446000}, 	 {15, 16014000},
	{20, 126087000}, 	 {25, 100000000}, 	{30, 79808000}, 	 {35, 64077000},
	{40, 51745000}, 	 {45, 42021000}, 		{50, 34308000}, 	 {55, 28156000},
	{60, 23222000}, 	 {65, 19246000}, 		{70, 16025000}, 	 {75, 13402000},
	{80, 11258000}, 	 {85, 9495800}, 		{90, 8042000}, 		 {95, 6837300},
	{100, 5835100}, 	 {105, 4997800}
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

int32_t adc_to_microCelcius(uint32_t s, uint32_t n){
	uint32_t mohms = nanovolts_to_milliohms(adc_to_nanovolts(s,n));
	uint8_t hiIndex = binarySearchOver(mohms,0,29); //higher temp ind
	int32_t lowuC = lut[hiIndex-1].celcius*1000000;
	int32_t uCRange = (lut[hiIndex].celcius-lut[hiIndex-1].celcius)*1000000;
	int32_t mORange = lut[hiIndex].milliohms-lut[hiIndex-1].milliohms;
	int32_t deltamO = mohms-lut[hiIndex-1].milliohms;
	int64_t dividend = (int64_t)mCRange * deltamO;
	return lowmC + (int32_t)((dividend + mORange/2)/mORange);
}
