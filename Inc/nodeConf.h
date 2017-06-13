/*
 * nodeConf.h
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 *  Edit this file for all node configurations!
 */

/*
 * Node configuration instructions:
 * 1. Make sure all parameters in this files is set properly according to node specifications
 * 2. Implement additional application-layer parsing in do
 * 		- main.c (loc "XXX 1")
 * 3. Implement flush queues in executeCommand()
 * 		- nodeMiscHelpers.c (loc "XXX 2" and loc "XXX 3")
 * 4. Suspend any application layer tasks in shutdown command
 * 		- nodeMiscHelpers.c (loc "XXX 4")
 *
 *
 * If you have any additional tasks/queues/mutexes/semaphores, you should try to add them through CubeMX if possible.
 * As always, ensure that there is sufficient FLASH and HEAP!
 *
 * The task priorities should ALWAYS be set as the following:
 * PRIORITY |	TASK
 * 	HIGH	  watchdogRefresh (if applicable)
 * 	 ^	 	  FreeRTOS_Timer
 *   |		  Can_Processor
 *   |---------------------------------------
 *   |		  Application Layer	Tasks		|
 *   |---------------------------------------
 *  LOW		  IdleTask
 *
 *	Deviation from this priority list may result in unstable node behavior and/or timing failure!
 */

#ifndef NODECONF_H_
#define NODECONF_H_
   
#include "stm32f4xx_hal.h"
#include "../../CAN_ID.h"

//Node handshake settings:
#define HB_Interval		1000		// Node heartbeat send interval	(soft ms)
#define WD_Interval		16			// Watdog timer refresh interval (soft ms) | MUST BE LESS THAN 26!!!
static const uint32_t firmwareString = 0x00000001;      // Firmware Version string
static const uint8_t selfNodeID = bps_nodeID;			// The nodeID of this node
extern uint32_t selfStatusWord;	// Initialize
#define NODE_CONFIGURED

//RTOS Task frequency settings:
#define RT_Interval		50
#define SMT_Interval	1000
#define TMT_Interval	500
#define RT_Broadcast	20			//Multiplier of RT_Interval

//MCP3903 PSB Settings:
#define PSB_OA			4000000		//uA
#define PSB_UA			-4000000	//uA
#define PSB_OV			140000000	//uV
#define PSB_UV			80000000	//uV

//LTC6801 Settings:
#define LTC_TOTAL_IC    3  	// Number of LTC boards in the stack
#define VUV             (1720U)	// LTC Undervoltage set point 2.754V
#define VOV             (2715U)	// LTC Overvoltage set point 4.344V




#endif /* NODECONF_H_ */
