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
#include "sensorTask.h"
#include "I2CTaskMsgTypes.h"
#include "lpc17xx_gpio.h"
#include "messageDefs.h"

#define i2cSTACK_SIZE		(5*configMINIMAL_STACK_SIZE)

static xQueueHandle staticHandle;
static char cCountBuf1[ 1280 ];

typedef struct {
	uint8_t msgType;
	uint8_t data[4];
} sensorMsg;

static portTASK_FUNCTION_PROTO( vsensorTask, pvParameters );
//start the task
void vStartsensorTask(sensorStruct *params ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd, motorStruct *motorData) {

	if ((params->inQ = xQueueCreate(20,sizeof(sensorMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	staticHandle = params->inQ;
	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	params->motorData = motorData;
	if ((retval = xTaskCreate( vsensorTask , ( signed char * ) "sensorTask", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendsensorGatherMsg(sensorStruct *sensorData)
{
	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	sensorMsg gatherMsg;
	gatherMsg.msgType = GATHER_MSG;
	return(xQueueSend(staticHandle,(void *) (&gatherMsg),portMAX_DELAY));
}

portBASE_TYPE SendsensorMacroOverride(uint8_t state)
{
	sensorMsg newState;
	newState.data[0] = state;
	newState.msgType = MACROSTATE_OVERRIDE;
	return(xQueueSend(staticHandle,(void *) (&newState),portMAX_DELAY));
}

portBASE_TYPE SendmessageCheck(sensorStruct *sensorData)
{
	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	sensorMsg gatherMsg;
	gatherMsg.msgType = sensorData->checkType;
	return(xQueueSend(sensorData->inQ,(void *) (&gatherMsg),portMAX_DELAY));
}

portBASE_TYPE SendsensorValueMsg(sensorStruct *sensorData, uint8_t msgtype, uint8_t length, uint8_t* value, portTickType ticksToBlock)
{
	sensorMsg valueMsg;

	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	valueMsg.msgType = msgtype;
	uint8_t i;
	for( i = 0; i < length; i = i+1) {
		valueMsg.data[i] = value[i];
	}
	return(xQueueSend(sensorData->inQ,(void *) (&valueMsg),ticksToBlock));
}

portBASE_TYPE SendsensorERRORMsg(sensorStruct *sensorData, uint8_t errorType, portTickType ticksToBlock)
{
	sensorMsg errorMsg;

	if (sensorData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	errorMsg.msgType = errorType;

	return(xQueueSend(sensorData->inQ,(void *) (&errorMsg),ticksToBlock));
}

int getMsgType(sensorMsg *Msg)
{
	return(Msg->msgType);
}

uint8_t* getData(sensorMsg *Msg)
{
	return (uint8_t *) &Msg->data[0];
}

void clearData(sensorMsg *Msg)
{
  	int i;
	for (i = 0; i < 4; i = i + 1)
		Msg->data[i] = 0;
}

void algFunction(uint8_t *sensorFrame, uint8_t *algState, uint8_t *moveComm, uint8_t *distance) {
	switch (*algState) {
		case ALG_STOPPED: {
		*moveComm = ROVERMOVE_FORWARD_CORRECTED;
		*algState = ALG_FORWARD;
		break;
		}
		case ALG_FORWARD: {
		if(sensorFrame[2] >= 0x5A && sensorFrame[3] >= 0x5A) {
			*moveComm = ROVERMOVE_FORWARD_ABSOLUTE;
			*distance = 45;
			*algState = ALG_CLEARING;
		} else if(sensorFrame[1] <= 0x5A) {
			*moveComm = ROVERMOVE_TURN_LEFT;
			*algState = ALG_AGAINST_OBSTACLE;
		}
		break;
		}
		case ALG_CLEARING: {
			*moveComm = ROVERMOVE_TURN_RIGHT;
			*algState = ALG_ON_CORNER;
		break;
		}
		case ALG_AGAINST_OBSTACLE: {
			*moveComm = ROVERMOVE_FORWARD_CORRECTED;
			*algState = ALG_FORWARD;
		break;
		}
		case ALG_ON_CORNER: {
		if(sensorFrame[2] <= 0x5A && sensorFrame[3] <= 0x5A) {
			*moveComm = ROVERMOVE_FORWARD_CORRECTED;
			*algState = ALG_FORWARD;
		} else {
			*moveComm = ROVERMOVE_FORWARD_ABSOLUTE;
			*distance = 45;
		}
		break;
		}
	}
}

static portTASK_FUNCTION(vsensorTask, pvParameters) {
	sensorStruct *param = (sensorStruct *) pvParameters;
	sensorMsg msg;
	uint8_t algState = ALG_STOPPED;
	uint8_t macroState = MACROSTATE_IDLE;
	uint8_t moveComm = ROVERMOVE_FORWARD_CORRECTED;

	const uint8_t gatherReq[]= {0xAA};
	const uint8_t gatherCheck[]= {0xAB};
	const uint8_t moveAckCheck[]= {0xBB};
	const uint8_t moveProgCheck[] = {0xCA};
	const uint8_t moveProgCheckCheck[] = {0xCB};
	
	SendLCDPrintMsg(param->lcdData,20,"sensorTask Init",portMAX_DELAY);
	SendLCDStateMsg(param->lcdData,algState, macroState, portMAX_DELAY);
	
	for( ;; ) {
		//wait forever or until queue has something
		if (xQueueReceive(param->inQ,(void *) &msg,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		SendLCDStateMsg(param->lcdData,algState, macroState, portMAX_DELAY);
		switch(getMsgType(&msg)) {
			case GATHER_MSG: {
				//current slave address is 0x4F, take note
					if (vtI2CEnQ(param->dev,vtSensorGatherRequest,0x4F,sizeof(gatherReq),gatherReq,3) != pdTRUE)
						VT_HANDLE_FATAL_ERROR(0);
					SendLCDPrintMsg(param->lcdData,20,"SND: Gather Req",portMAX_DELAY);
			break;
			}
			//this is a check for sensor data called by a timer callback
			case GATHER_CHECK: {
					if (vtI2CEnQ(param->dev,vtSensorGatherCheck,0x4F,sizeof(gatherCheck),gatherCheck,5) != pdTRUE) {
						VT_HANDLE_FATAL_ERROR(0);
					}
					SendLCDPrintMsg(param->lcdData,20,"SND: Gather Chk",portMAX_DELAY);
			break;
			}
			//1 is incoming i2c data, this is where we handle sensor data and call algorithm subroutines... hopefully
			case SENSORVALUE_MSG: {
				uint8_t *sensorFrame = getData(&msg);
				SendLCDPrintMsg(param->lcdData,20,"RCV: Sensor Data",portMAX_DELAY);
				
				uint8_t distance = 0;
				//this is where the movement algorithm will decide what to issue as a command
				switch (macroState) {
					case MACROSTATE_IDLE:
						SendsensorGatherMsg(param);
						SendLCDStateMsg(param->lcdData, algState, macroState, portMAX_DELAY);
						break;
					case MACROSTATE_FINDING_LINE:
						algFunction(sensorFrame, &algState, &moveComm, &distance);
						SendLCDStateMsg(param->lcdData,algState, macroState, portMAX_DELAY);
						SendmotorMoveMsg(param->motorData, moveComm, distance, portMAX_DELAY);
						break;
					case MACROSTATE_RUN_ONE:
						algFunction(sensorFrame, &algState, &moveComm, &distance);
						SendLCDStateMsg(param->lcdData,algState, macroState, portMAX_DELAY);
						SendmotorMoveMsg(param->motorData, moveComm, distance, portMAX_DELAY);
						break;
				}
				
				//SendmotorMoveMsg(param->motorData, moveComm, distance, portMAX_DELAY);
			break;
			}
			case MACROSTATE_OVERRIDE: {
			uint8_t *newState = getData(&msg);
			macroState = newState[0];
			SendLCDPrintMsg(param->lcdData,20,"OVERRIDE OVERRIDE",portMAX_DELAY);
			break;
			}
			//bad/no data from the rover, we need to regather
			case GATHER_ERROR_MSG: {
			  	SendLCDPrintMsg(param->lcdData,20,"RSND: Gather Req",portMAX_DELAY);
				SendsensorGatherMsg(param);

				//COMMENT THIS OUT!!
				//dataPtrSensor = getData(&msg);
				//c = 2; //test
			break;
			}
			case ROVERACK_CHECK: {
				if (vtI2CEnQ(param->dev,vtRoverMovementCommandAckCheck,0x4F,sizeof(moveAckCheck),moveAckCheck,3) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendLCDPrintMsg(param->lcdData,20,"SND: Ack Check",portMAX_DELAY);
			break;
			}
			case ROVERMOVE_CHECK: {
				if (vtI2CEnQ(param->dev,vtRoverMovementProgCheck,0x4F,sizeof(moveProgCheck),moveProgCheck,3) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendLCDPrintMsg(param->lcdData,20,"SND: Move Check",portMAX_DELAY);
				break;
			}
			case ROVERMOVE_CHECKCHECK: {
				if (vtI2CEnQ(param->dev,vtRoverMovementProgCheckCheck,0x4F,sizeof(moveProgCheckCheck),moveProgCheckCheck,6) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendLCDPrintMsg(param->lcdData,20,"SND: Move ChkChk",portMAX_DELAY);
				break;
			}
			case ROVERMOVE_MSG: {
				SendLCDPrintMsg(param->lcdData,20,"RCV: Move Data",portMAX_DELAY);

				
				//this is where the actual movement will be recorded in the map	

				
				dataPtrSensor = getData(&msg);

				// second run data storage
				secondRun[curr] = dataPtrSensor[0];
				curr++;
				secondRun[curr] = dataPtrSensor[1];
				curr++;
				secondRun[curr] = dataPtrSensor[2];
				curr++;
				secondRun[curr] = dataPtrSensor[3];
				curr++;
				secondRun[curr] = dataPtrSensor[4];
				curr++;
				secondRun[curr] = dataPtrSensor[5];
				curr++;
				//c = 1;
				if(dataPtrSensor[1] == 0x01) 
					SendsensorGatherMsg(param);

				
				
			break;
			}
		}

	}
}
//curr = 0;

//dataPtrSensor = 4398383366144;


void vGetMapData(void){
	
	if (dataPtrSensor[1] == 0x00){
 	if(dataPtrSensor[2] > dataPtrSensor[3]){

		//rover has made a right-hand turn, assume obstacle or corner
		  flagRight = 1;
		  flagLeft = 0;
		  flagStraight = 0;
		  //sprintf(uip_appdata, "ctx.lineTo(200,150);\n");
		}
	if (dataPtrSensor[3] > dataPtrSensor[2]){
		 flagLeft = 1;
		 flagRight = 0;
		 flagStraight = 0;
		}
	    //rover has made a left-hand turn, assume obstacle or corner 
	if (dataPtrSensor[3] == dataPtrSensor[2]){
			flagStraight = 1;
			flagLeft = 0;
			flagRight = 0;
		//rover is going straight, take note of distance
		} 
		}
}

