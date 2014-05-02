#ifndef MOTORTASK_H
#define MOTORTASK_H
#include "vtI2C.h"
#include "lcdTask.h"

typedef struct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	xQueueHandle inQ;
} motorStruct;

// Maximum length of a message that can be received by this task
#define msgMaxLen   (sizeof(portTickType))

void vStartmotorTask(motorStruct *motorData ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd);
//send message from sensorTask to motorTask
portBASE_TYPE SendmotorMoveMsg(motorStruct *motorData, uint8_t moveType, uint8_t distance, portTickType ticksToBlock);
portBASE_TYPE SendmotorERRORMsg(motorStruct *motorData, uint8_t errorType, portTickType ticksToBlock);
#endif