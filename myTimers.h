#ifndef _MY_TIMERS_H
#define _MY_TIMERS_H
#include "lcdTask.h"
#include "i2cTemp.h"
#include "adcTask.h"
#include "sensorTask.h"

#include "timers.h"

void startTimerForLCD(vtLCDStruct *vtLCDdata);
void startTimerForTemperature(vtTempStruct *vtTempdata);
void startTimerForADC(adcStruct *adcData);
xTimerHandle initCheckTimer(sensorStruct *sensorData);
#endif