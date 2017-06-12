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

#define roundivide(a,b) (((((a)<0)&&((b)<0))||(((a)>=0)&&((b)>=0)))?(((a)+((b)/2))/(b)):(((a)-((b)/2))/(b)))

int32_t psb1ch1Map(int32_t raw);
int32_t psb1ch0Map(int32_t raw);

#endif /* PSB1CAL_H_ */
