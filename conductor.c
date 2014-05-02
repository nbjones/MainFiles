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
#include "adcTask.h"
#include "sensorTask.h"
#include "I2CTaskMsgTypes.h"
#include "conductor.h"
#include "lpc17xx_gpio.h"
#include "messageDefs.h"

/* *********************************************** */
// definitions and data structures that are private to this file

// I have set this to a large stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the i2c operations	-- almost certainly too large, see LCDTask.c for details on how to check the size
#define INSPECT_STACK 1
#define baseStack 2
#if PRINTF_VERSION == 1
#define conSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define conSTACK_SIZE		(baseStack*configMINIMAL_STACK_SIZE)
#endif
// end of defs
/* *********************************************** */

/* The i2cTemp task. */
static portTASK_FUNCTION_PROTO( vConductorUpdateTask, pvParameters );

/*-----------------------------------------------------------*/
// Public API
void vStartConductorTask(vtConductorStruct *params,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c,adcStruct *adc, sensorStruct *sensor, motorStruct *motor)
{
	/* Start the task */
	portBASE_TYPE retval;
	params->dev = i2c;
	params->adcData = adc;
	params->sensorData = sensor;
	params->motorData = motor;
	if ((retval = xTaskCreate( vConductorUpdateTask, ( signed char * ) "Conductor", conSTACK_SIZE, (void *) params, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

// End of Public API
/*-----------------------------------------------------------*/

// This is the actual task that is run
static portTASK_FUNCTION( vConductorUpdateTask, pvParameters )
{
	uint8_t rxLen, status;
	uint8_t Buffer[vtI2CMLen];
	uint8_t *valPtr = &(Buffer[0]);
	// Get the parameters
	vtConductorStruct *param = (vtConductorStruct *) pvParameters;
	// Get the I2C device pointer
	vtI2CStruct *devPtr = param->dev;
	// Get the MS1 ADCtask pointer
	adcStruct *adcData = param->adcData;
	// Get the sensorStruct pointer
	sensorStruct *sensorData = param->sensorData;
	// Get the motorStruct pointer
	motorStruct *motorData = param->motorData;
	
	uint8_t recvMsgType;
	uint8_t timeOutCount = 0;

	// Like all good tasks, this should never exit
	for(;;)
	{
		// Wait for a message from an I2C operation
		if (vtI2CDeQ(devPtr,vtI2CMLen,Buffer,&rxLen,&recvMsgType,&status) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}

		// Decide where to send the message 
		//   This just shows going to one task/queue, but you could easily send to
		//   other Q/tasks for other message types
		// This isn't a state machine, it is just acting as a router for messages
		switch(recvMsgType) {
		case vtMS1ADCRequest: {
			GPIO_SetValue(0,0x20000);
		   	SendadcValueMsg(adcData,(*valPtr), valPtr, portMAX_DELAY);
			GPIO_ClearValue(0,0x20000);
			break;
		}
		case vtSensorGatherRequest: {
			GPIO_SetValue(0,0x20000);
			if((rxLen != 3) || (valPtr[0] != 0x00) || (((valPtr[1])&0x17) != valPtr[2]) || (valPtr[1] != 0x01)) //ERROR
				SendsensorERRORMsg(sensorData, GATHER_ERROR_MSG, portMAX_DELAY);
			else {
				timeOutCount = 0;
				sensorData->checkType = GATHER_CHECK;
				if (xTimerStart(sensorData->checkTimerHandle,0) != pdPASS) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			GPIO_ClearValue(0,0x20000);
			break;
		}
		case vtSensorGatherCheck: {
		 	if (timeOutCount > 9){
				if (xTimerStop(sensorData->checkTimerHandle,0) != pdPASS) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendsensorERRORMsg(sensorData, GATHER_ERROR_MSG, portMAX_DELAY);
				break;
			} else if((rxLen != 5) || (valPtr[0] != 0x01) || (((valPtr[1]+valPtr[2]+valPtr[3])&0x17) != valPtr[4])) {//ERROR
				timeOutCount = timeOutCount + 1;
				break;
			} else {
				if (xTimerStop(sensorData->checkTimerHandle,0) != pdPASS) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				timeOutCount = 0;
				SendsensorValueMsg(sensorData, SENSORVALUE_MSG, 4, valPtr, portMAX_DELAY);
			}
			break;
		}
		case vtRoverMovementCommand: {
			if((rxLen != 3) || (valPtr[0] != 0x02) || ((valPtr[1]&0x17) != valPtr[2]) || (valPtr[1] != 0x01)) { //ERROR
				SendmotorERRORMsg(motorData, ROVERACK_ERROR, portMAX_DELAY);
			}
			else {
				timeOutCount = 0;
				sensorData->checkType = ROVERACK_CHECK;
				if (xTimerStart(sensorData->checkTimerHandle,0) != pdPASS) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			}
			break;
		}
		case vtRoverMovementCommandAckCheck: {
			if(timeOutCount > 20) { 
				if (xTimerStop(sensorData->checkTimerHandle,0) != pdPASS) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				SendmotorERRORMsg(motorData, ROVERACK_ERROR, portMAX_DELAY);
				break;
			}
			else if((rxLen != 3) || (valPtr[0] != 0x03) || ((valPtr[1]&0x17) != valPtr[2])) { //ERROR
				timeOutCount = timeOutCount + 1;
				break;
			}
			else {
				if (xTimerStop(sensorData->checkTimerHandle,0) != pdPASS) {
					VT_HANDLE_FATAL_ERROR(0);
				}
				timeOutCount = 0;
				SendsensorValueMsg(sensorData, ROVERMOVE_CHECK, 0, valPtr, portMAX_DELAY);
			}
				break;
		}
		case vtRoverMovementProgCheck: {
			if(valPtr[0] == 0x05) {
				timeOutCount = 0;
				sensorData->checkType = ROVERMOVE_CHECKCHECK;
				if (xTimerStart(sensorData->checkTimerHandle,0) != pdPASS) {
					VT_HANDLE_FATAL_ERROR(0);
				}
			} else {
				SendsensorValueMsg(sensorData, ROVERMOVE_CHECK, 0, valPtr, portMAX_DELAY);
			}	
			break;
		}
		case vtRoverMovementProgCheckCheck: {
			timeOutCount += 1;
			if(valPtr[0] == 0x04) {
				if(valPtr[1] == 0x01) {
					if (xTimerStop(sensorData->checkTimerHandle,0) != pdPASS) {
							VT_HANDLE_FATAL_ERROR(0);
						}
					timeOutCount = 0;
				}
				SendsensorValueMsg(sensorData, ROVERMOVE_MSG, 6, valPtr, portMAX_DELAY);
				sensorData->checkType = ROVERMOVE_CHECK;
			}
			if(timeOutCount == 10) {
				sensorData->checkType = ROVERMOVE_CHECK;
				timeOutCount = 0;
			}
		break;
		}
		default: {
			VT_HANDLE_FATAL_ERROR(recvMsgType);
			break;
		}
		}


	}
}

