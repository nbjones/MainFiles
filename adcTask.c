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
#include "adcTask.h"
#include "I2CTaskMsgTypes.h"
#include "lpc17xx_gpio.h"

#define i2cSTACK_SIZE		(5*configMINIMAL_STACK_SIZE)

typedef struct {
	uint8_t msgType;
	uint8_t data[28];
} adcMsg;

static portTASK_FUNCTION_PROTO( vadcTask, pvParameters );
//start the task
void vStartadcTask(adcStruct *params ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd) {

	if ((params->inQ = xQueueCreate(50,sizeof(adcMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	portBASE_TYPE retval;
	params->dev = i2c;
	params->lcdData = lcd;
	if ((retval = xTaskCreate( vadcTask , ( signed char * ) "adcTask", i2cSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}
//this is called by the timer, and queues up a message to the task that the timer has fired
portBASE_TYPE SendadcTimerMsg(adcStruct *adcData)
{
	if (adcData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	adcMsg timerMsg;
	timerMsg.msgType = 0;
	return(xQueueSend(adcData->inQ,(void *) (&timerMsg),0));
}

portBASE_TYPE SendadcValueMsg(adcStruct *adcData,uint8_t length,uint8_t* value,portTickType ticksToBlock)
{
	adcMsg tempBuffer;

	if (adcData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	tempBuffer.msgType = 1;
	uint8_t i;
	for( i = 0; i < length; i = i+1) {
		tempBuffer.data[i] = value[i];
	}
	return(xQueueSend(adcData->inQ,(void *) (&tempBuffer),ticksToBlock));
}

int getMsgType(adcMsg *Msg)
{
	return(Msg->msgType);
}
uint8_t* getData(adcMsg *Msg)
{
	return (uint8_t *) &Msg->data[0];
}

static portTASK_FUNCTION(vadcTask, pvParameters) {
	adcStruct *param = (adcStruct *) pvParameters;
	adcMsg msg;
    
	uint8_t buff[20];
	int buffLoc = 0;

	const uint8_t i2caddr[]= {0xAA};

	for( ;; ) {
		//wait forever or until queue has something
		if (xQueueReceive(param->inQ,(void *) &msg,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		switch(getMsgType(&msg)) {
			//0 is timer, send an I2C request out
			case 0: {
				if (vtI2CEnQ(param->dev,vtMS1ADCRequest,0x4F,sizeof(i2caddr),i2caddr,28) != pdTRUE) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			break;
			}
			//1 is incoming i2c data, parse it
			case 1: {
				uint8_t *dataPtr = getData(&msg);
				int i;
				for(i = 1; i < dataPtr[0]; i = i + 1) {
					buff[buffLoc] = dataPtr[i];
					if(buffLoc == 19) {
						SendLCDADC(param->lcdData,20,buff,portMAX_DELAY);
						buffLoc = 0;
					}
					else {
						buffLoc++;
					}
				}
			break;
			}
		}

	}

}

