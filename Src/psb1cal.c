/*
 * psb1cal.c
 *
 *  Created on: Jun 12, 2017
 *      Author: jamesliu
 */
#include "psb1cal.h"

//psb1ch1Map formula: (raw-69080.9705882353)/165597.927315084
//the below returns microamps
int32_t psb1ch1Map(int32_t raw){
	return (int32_t)roundivide(((int64_t)raw*1000000000-69080970588235),165597927);
}

//psb1ch0Map formula: (raw+20783)/33038
//the below returns microVolts
int32_t psb1ch0Map(int32_t raw){ //actially from psb0!!!
	return (int32_t)roundivide(((int64_t)raw*1000000000+20783000000000),33038000);
}
