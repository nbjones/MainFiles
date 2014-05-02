#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "projdefs.h"
#include "semphr.h"

/* include files. */
#include "vtUtilities.h"
#include "vtI2C.h"
#include "LCDtask.h"
#include "sensorTask.h"
#include "motorTask.h"
#include "I2CTaskMsgTypes.h"
#include "lpc17xx_gpio.h"
#include "messageDefs.h"

#define i2cSTACK_SIZE		(5*configMINIMAL_STACK_SIZE)

typedef struct {
	uint8_t msgType;
	uint8_t moveType;
	uint8_t distance; //in cm
} motorMsg;

static portTASK_FUNCTION_PROTO( vmotorTask, pvParameters );
//start the task
void vStartmotorTask(motorStruct *params ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd) {

	if ((params->inQ = xQueueCreate(20,sizeof(motorMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	if ((retval = xTaskCreate( vmotorTask , ( signed char * ) "motorTask", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendmotorMoveMsg(motorStruct *motorData, uint8_t moveType, uint8_t distance, portTickType ticksToBlock)
{
	motorMsg moveMsg;

	if (motorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	moveMsg.msgType = SENSORTASK_MSG;
	moveMsg.moveType = moveType;
	moveMsg.distance = distance;
	return(xQueueSend(motorData->inQ,(void *) (&moveMsg),ticksToBlock));
}

portBASE_TYPE SendmotorERRORMsg(motorStruct *motorData, uint8_t errorType, portTickType ticksToBlock)
{
	motorMsg errorMsg;

	if (motorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	errorMsg.msgType = errorType;

	return(xQueueSend(motorData->inQ,(void *) (&errorMsg),ticksToBlock));
}

uint8_t getMsgType(motorMsg *Msg)
{
	return(Msg->msgType);
}

uint8_t getMoveType(motorMsg *Msg)
{
	return(Msg->moveType);
}

uint8_t getDistance(motorMsg *Msg)
{
	return(Msg->distance);
}

static portTASK_FUNCTION(vmotorTask, pvParameters) {
	motorStruct *param = (motorStruct *) pvParameters;
	motorMsg msg;

	const uint8_t bc_half_forward[] = {0xBC, 0xA0, 0x20, 0xFF, 0xFF, 0x00};
    const uint8_t ba_15cm_forward[] = {0xBA, 0xA0, 0x20, 0x0F, 0x0F, 0x00};  // 0.5ft
    const uint8_t ba_45cm_forward[] = {0xBA, 0xA0, 0x20, 0x2D, 0x2D, 0x00};  // 1.5ft
    const uint8_t ba_90_left[] = {0xBA, 0xE0, 0x20, 0x12, 0x12, 0x00};
    const uint8_t ba_90_right[] = {0xBA, 0xA0, 0x60, 0x12, 0x12, 0x00};
	uint8_t ba_custom_forward[] = {0xBA, 0xA0, 0x20, 0x2D, 0x2D, 0x00};
	
	const uint8_t *motorCommand = bc_half_forward;
	
	SendLCDPrintMsg(param->lcdData,20,"motorTask Init",portMAX_DELAY);
	
	for( ;; ) {
		//wait forever or until queue has something
		if (xQueueReceive(param->inQ,(void *) &msg,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		switch(getMsgType(&msg)) {
			//only one type of message so far
			case SENSORTASK_MSG: {

				switch(getMoveType(&msg)) {
					case ROVERMOVE_FORWARD_ABSOLUTE:
						motorCommand = ba_custom_forward;
						uint8_t distance = getDistance(&msg);
						ba_custom_forward[3] = distance;
						ba_custom_forward[4] = distance;
						break;
					case ROVERMOVE_FORWARD_CORRECTED:
						motorCommand = bc_half_forward;
						break;
					case ROVERMOVE_TURN_LEFT:
						motorCommand = ba_90_left;
						break;
					case ROVERMOVE_TURN_RIGHT:
						motorCommand = ba_90_right;
						break;
				}
				//current slave address is 0x4F, take note
				if (vtI2CEnQ(param->dev, vtRoverMovementCommand, 0x4F, 6, motorCommand, 3) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendLCDPrintMsg(param->lcdData,20,"SND: Move Command",portMAX_DELAY);
			break;
			}
			case ROVERACK_ERROR: {
				//this is where the arm will re-request the movement ack from the rover
				if (vtI2CEnQ(param->dev, vtRoverMovementCommand, 0x4F, 6, motorCommand, 3) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendLCDPrintMsg(param->lcdData,20,"RSND: Move Command",portMAX_DELAY);
			break;
			}
		}

	}
}

