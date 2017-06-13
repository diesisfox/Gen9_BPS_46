/*
 * psb1cal.h
 *
 *  Created on: Jun 12, 2017
 *      Author: jamesliu
 *
 *  Misc. functions that groups specific set of actions in a friendlier way
 */

#ifndef PSB1CAL_H_
#define PSB1CAL_H_
   
#include "main.h"
#include "nodeConf.h"
#include "stm32f4xx_hal.h"

#ifndef PSB_OA
#define PSB_OA		4000000		//uA
#endif
#ifndef PSB_UA
#define PSB_UA		-4000000	//uA
#endif
#ifndef PSB_OV
#define PSB_OV		140000000	//uV
#endif
#ifndef PSB_UV
#define PSB_UV		80000000	//uV
#endif

#define roundivide(a,b) (((((a)<0)&&((b)<0))||(((a)>=0)&&((b)>=0)))?(((a)+((b)/2))/(b)):(((a)-((b)/2))/(b)))

int32_t psb1ch1Map(int32_t raw);
int32_t psb1ch0Map(int32_t raw);

#endif /* PSB1CAL_H_ */
