#ifndef JOHNTASK_H
#define JOHNTASK_H
#include "vtI2C.h"
#include "lcdTask.h"

typedef struct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	xQueueHandle inQ;
} adcStruct;

// Maximum length of a message that can be received by this task
#define msgMaxLen   (sizeof(portTickType))

void vStartadcTask(adcStruct *adcData ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd);
//send message from timer
portBASE_TYPE SendadcTimerMsg(adcStruct *adcData);
//send message from conductor with i2c data from pic
portBASE_TYPE SendadcValueMsg(adcStruct *adcData,uint8_t length,uint8_t* value,portTickType ticksToBlock);
#endif