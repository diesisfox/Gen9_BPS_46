/*
 * nodeMiscHelpers.h
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 *
 *  Misc. functions that groups specific set of actions in a friendlier way
 */

#ifndef NODEMISCHELPERS_H_
#define NODEMISCHELPERS_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "can.h"
#include "serial.h"
#include "../../CAN_ID.h"

// Microsecond delay
// Multiply by 20 for O2 and O3
// Multiply by 16 for O0, O1, and Os
#ifdef DEBUG
#define delayUs(US) 	_delayUS_ASM(US * 16)
#else
#define delayUs(US) 	_delayUS_ASM(US * 20)
#endif

#define _delayUS_ASM(X) \
	asm volatile (	"mov r0,#" #X  "\n"\
			"loop: \n" \
              "sub r0,r0, #1 \n" \
              "cmp r0, #0 \n" \
			"bne loop \n" );

#define node_shutdown()		soft_shutdown(NULL)			// shutdown wrapper

// Following macro expansions are NOT THREAD SAFE!!!
#define selfState 		(nodeState)(selfStatusWord & 0x07)		// Get the state of the node
#define setState(x)		selfStatusWord &= 0xfffffff8; \
						selfStatusWord |= x;


void executeCommand(uint8_t cmd);
void setSelfState(nodeState newState);
nodeState getSelfState();
void soft_shutdown(void(*usr_clbk)());

void assert_bps_fault(uint16_t addr, uint32_t value);
void fault_save_data();

uint8_t valToHex(uint8_t i);
uint8_t HexToVal(uint8_t i);
uint8_t intToDec(uint32_t input, uint8_t *str); //returns length. Only does positives.
void intToHex(uint32_t input, uint8_t *str, int length);

#endif /* NODEMISCHELPERS_H_ */
