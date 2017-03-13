#ifndef TS_LIB_H
#define TS_LIB_H

#include <math.h>

#include "main.h"
#include "can.h"
#include "serial.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"

//MAX_CHANNEL specifies the number of channels of the mux in use
#define TEMP_CHANNELS 16
#define TEMP_MUXES ((TEMP_CHANNELS-1)/16+1)
#define OVER_TEMPERATURE 4096
#define UNDER_TEMPERATURE 0
#define TEMP_OVERSAMPLING 16

void Temp_begin(ADC_HandleTypeDef* hadc_in);
uint16_t getReading(uint8_t channel);





//TS_READ_CHANNEL_X: sends out a 4bit combination as mux channel selection and reads the analog value into READING
//argument typing:	ADC_HandleTypeDef* ADC
//					uint32_t READING

//TS_SINGLE_READ: reads the signal on the selected mux channel
//argument typing:	ADC_HandleTypeDef* ADC
//					uint32_t READING
//					uint32_t CHANNEL (only takes values of 0 - MAX_CHANNEL)

// //Vdd specifies the input voltage 0V - 3.6V (0 - 4095)
// #define Vdd 4095.0

// //R1 specifies resistance of resistor in voltage divider
// //can add more resistor values for each voltage divider (R2, R3, ...)
// #define R1 10000.0

// //R25 specifies the resistance of the NTC at 25 deg C
// //this value depends on the NTC and is measured
// //this value is used in the Steinhart-Hart Equation
// #define R25 10000.0

// //A, B, C, D specifies the coefficients of Steinhart-Hart Equation
// //taken from http://www.vishay.com/thermistors/ntc-curve-list/
// //for NTCLE413E2103H400
// #define A 0.003354016
// #define B 0.000256524
// #define C 0.00000260597
// #define D 0.0000000632926

// //TS_read_temp_channel: calculates the temperature according to the Steinhart-Hart Equation
// //returns temperature as a float
// float TS_read_temp_channel(ADC_HandleTypeDef hadc1, uint32_t channel);

#endif
