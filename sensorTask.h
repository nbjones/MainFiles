#ifndef SENSORTASK_H
#define SENSORTASK_H
#include "vtI2C.h"
#include "lcdTask.h"
#include "motorTask.h"
#include "timers.h"
#include "uip.h"



int curr =0;
int fast =1;
int slow =0;
uint8_t* dataPtrSensor; 
int flagRight, flagLeft, flagStraight;
int stateStraight =0;
uint8_t* secondRun[750];
uint8_t* previousData;

//extern uint8_t *dataPtrSensor;
//extern int dataList[10];
//extern int count;

typedef struct {
	vtI2CStruct *dev;
	vtLCDStruct *lcdData;
	motorStruct *motorData;
	xQueueHandle inQ;
	xTimerHandle checkTimerHandle;
	uint8_t checkType;
} sensorStruct;

// Maximum length of a message that can be received by this task
#define msgMaxLen   (sizeof(portTickType))

void vStartsensorTask(sensorStruct *sensorData ,unsigned portBASE_TYPE uxPriority, vtI2CStruct *i2c, vtLCDStruct *lcd, motorStruct *motorData);
//send message from timer
portBASE_TYPE SendmessageCheck(sensorStruct *sensorData);

portBASE_TYPE SendsensorGatherMsg(sensorStruct *sensorData);

portBASE_TYPE SendsensorMacroOverride(uint8_t state);
//send message from conductor with i2c data from pic
portBASE_TYPE SendsensorValueMsg(sensorStruct *sensorData,uint8_t msgtype, uint8_t length,uint8_t* value,portTickType ticksToBlock);
//send message that there was an error on the i2c bus
portBASE_TYPE SendsensorERRORMsg(sensorStruct *sensorData, uint8_t errorType, portTickType ticksToBlock);

void vGetMapData (void);
int secondRunImp(int inc);
#endif