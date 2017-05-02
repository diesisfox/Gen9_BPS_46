/*
 * nodeMiscHelpers.c
 *
 *  Created on: Dec 31, 2016
 *      Author: frank
 */
#include "nodeMiscHelpers.h"
#include "nodeConf.h"

uint32_t 	selfStatusWord;
extern osMutexId 	swMtxHandle;
extern osMessageQId mainCanTxQHandle;
extern osMessageQId mainCanRxQHandle;
extern osTimerId 	HBTmrHandle;

/*
 * Command executer for implementing node command responses
 */
void executeCommand(uint8_t cmd){
	switch(cmd){
	// Hard reset
	case NODE_HRESET:
		NVIC_SystemReset();					// CMSIS System reset function
		break;

	// Soft Reset
	case NODE_RESET:
		node_shutdown();					// Soft shutdown
		NVIC_SystemReset();					// CMSIS System reset function
		break;

	// Clean shutdown
	// NOTE: CAN command processor is still active!
	case NODE_SHUTDOWN:
		if((selfState == ACTIVE) || (selfState == INIT)){
			node_shutdown();						// Soft shutdown if node is active
			xTimerStop(HBTmrHandle, 0);	// Stop the heartbeat timer

			// XXX 4: User must suspend any additional application tasks
			// xTaskSuspend(ApplicationHandle);		// Suspend any active non-CAN tasks
		}
		break;

	// Node start command from shutdown state
	case NODE_START:
		if(selfState == SHUTDOWN){
			setState(INIT);
			// Flush the Rx queue for fresh state on start-up
			xQueueReset(mainCanRxQHandle);
			// XXX 2: Flush the application queues!
			// xQueueReset();

			xTimerReset(HBTmrHandle,portMAX_DELAY);	// Start the heartbeat timer

			// XXX 3: User must resume additional application tasks
			// xTaskResume(ApplicationHandle);			// Resume any application tasks
		}
		break;

	// CC Acknowledgement of node addition attempt
	case CC_ACK:
		if(selfState == INIT){
			setState(ACTIVE);
			static Can_frame_t newFrame;
			newFrame.id = selfNodeID + swOffset;
			newFrame.dlc = CAN_HB_DLC;
			for(int i=0; i<4; i++){
				newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
			}
			bxCan_sendFrame(&newFrame);
		}
		break;

	// CC Negation of node addition attempt
	case CC_NACK:
		if(selfState == INIT){
			setState(SHUTDOWN);
		}
		break;

	default:
		// Do nothing if the command is invalid
		break;
	}
}

/* CHECKED
 * Thread-safe node state accessor
 */
nodeState getSelfState(){
	xSemaphoreTake(swMtxHandle, portMAX_DELAY);
	nodeState ret = (nodeState)(selfStatusWord & 0x07);
	xSemaphoreGive(swMtxHandle);
	return ret;
}

/* CHECKED
 * Thread-safe node state mutator
 */
void setSelfState(nodeState newState){
	xSemaphoreTake(swMtxHandle, portMAX_DELAY);
	selfStatusWord &= 0xfffffff8;	// Clear the current status word
	selfStatusWord |= newState;
	xSemaphoreGive(swMtxHandle);
}

/*
 * Soft shutdown routine that will complete any critical cleanups via callback
 * Assembles node SHUTDOWN statusword CAN frame
 * Flushes CanTx queue via broadcast on bxCAN
 */
void soft_shutdown(void(*usr_clbk)()){
	// Don't care about locking the statusWord here since we are in Critical Area
	setState(SHUTDOWN);

	// User defined shutdown routine
//	usr_clbk();

	// Broadcast node shutdown state to main CAN
	Can_frame_t newFrame;
	newFrame.id = radio_SW;
	newFrame.isExt = 0;
	newFrame.dlc = CAN_HB_DLC;
	for(int i=0; i<4; i++){
		newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}
	bxCan_sendFrame(&newFrame);
	// TODO: Test if bxCan_sendFrame can successfully send the new frame and flush the queue
}

/*
 * Routine for bps line asserting and the saving of data beforehand, with its support functions
 */
void fault_save_data(){
	//stubby
}

void assert_bps_fault(uint16_t addr, uint32_t value){	//addr and value of out of line reading
	//what the name says
	vTaskPrioritySet(NULL, tskIDLE_PRIORITY + configMAX_PRIORITIES - 1);

	fault_save_data();

	// Broadcast bps fault to main CAN
	Can_frame_t newFrame;
	newFrame.id = bpsTrip;
	newFrame.isExt = 0;
	newFrame.isRemote = 0;
	newFrame.dlc = bpsTrip_DLC;
	newFrame.Data[0] = addr >> 8;
	newFrame.Data[1] = addr & 0xf;
	for(int i=0; i<4; i++){
		newFrame.Data[5-i] = (value >> (8*i)) & 0xff;	// Convert uint32_t -> uint8_t
	}
	bxCan_sendFrame(&newFrame);
	osDelay(1);
//	for(;;);

	//assert signal
	HAL_GPIO_WritePin(BSD_GPIO_Port, BSD_Pin, RESET);
}

uint8_t valToHex(uint8_t i){
	return (i<=9 ? '0'+i : 'A'+i-10);
}

uint8_t HexToVal(uint8_t i){//0xff = invalid char
	if(i>='0' && i<='9'){
		return i-'0';
	}else if(i>='A' && i<='F'){
		return i-'A'+10;
	}else if(i>='a' && i<='f'){
		return i-'a'+10;
	}
	return 0xff;
}

uint8_t intToDec(uint32_t input, uint8_t *str){ //returns length. Only does positives.
	uint8_t length = 0;
	uint8_t output[10];
	while(input/10){
		length++;
		output[10-length] = valToHex(input%10);
		input/=10;
	}
	length++;
	output[10-length] = valToHex(input);
	for(int i=0; i<length; i++){
		str[i] = output[10-length+i];
	}
	return length;
}

void intToHex(uint32_t input, uint8_t *str, int length){
	for(int i=0; i<length; i++){
		str[length-1-i]=valToHex(input&0x0F);
		input = input>>4;
	}
}

