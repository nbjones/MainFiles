#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

/* include files. */
#include "GLCD.h"
#include "vtUtilities.h"
#include "LCDtask.h"
#include "string.h"
#include "lpc17xx_gpio.h"
#include "messageDefs.h"

#define USE_MS1_CODE 0

// I have set this to a larger stack size because of (a) using printf() and (b) the depth of function calls
//   for some of the LCD operations
// I actually monitor the stack size in the code to check to make sure I'm not too close to overflowing the stack
//   This monitoring takes place if INPSECT_STACK is defined (search this file for INSPECT_STACK to see the code for this) 
#define INSPECT_STACK 1
#define baseStack 3
#if PRINTF_VERSION == 1
#define lcdSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#else
#define lcdSTACK_SIZE		((baseStack+5)*configMINIMAL_STACK_SIZE)
#endif

// definitions and data structures that are private to this file
// Length of the queue to this task
#define vtLCDQLen 50 
// a timer message -- not to be printed
#define LCDMsgTypeTimer 1
// a message to be printed
#define LCDMsgTypePrint 2
// adc data
#define LCDMsgTypeADC 3

#define LCDMsgTypeState 4
// actual data structure that is sent in a message
typedef struct __vtLCDMsg {
	uint8_t msgType;
	uint8_t	length;	 // Length of the message to be printed
	uint8_t buf[vtLCDMaxLen+1]; // On the way in, message to be sent, on the way out, message received (if any)
} vtLCDMsg;

// end of defs

/* definition for the LCD task. */
static portTASK_FUNCTION_PROTO( vLCDUpdateTask, pvParameters );

/*-----------------------------------------------------------*/

void StartLCDTask(vtLCDStruct *ptr, unsigned portBASE_TYPE uxPriority)
{
	if (ptr == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}

	// Create the queue that will be used to talk to this task
	if ((ptr->inQ = xQueueCreate(vtLCDQLen,sizeof(vtLCDMsg))) == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	/* Start the task */
	portBASE_TYPE retval;
	if ((retval = xTaskCreate( vLCDUpdateTask, ( signed char * ) "LCD", lcdSTACK_SIZE, (void*)ptr, uxPriority, ( xTaskHandle * ) NULL )) != pdPASS) {
		VT_HANDLE_FATAL_ERROR(retval);
	}
}

portBASE_TYPE SendLCDTimerMsg(vtLCDStruct *lcdData,portTickType ticksElapsed,portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;
	lcdBuffer.length = sizeof(ticksElapsed);
	if (lcdBuffer.length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	memcpy(lcdBuffer.buf,(char *)&ticksElapsed,sizeof(ticksElapsed));
	lcdBuffer.msgType = LCDMsgTypeTimer;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE SendLCDPrintMsg(vtLCDStruct *lcdData,int length,char *pString,portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	if (length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	lcdBuffer.length = strnlen(pString,vtLCDMaxLen);
	lcdBuffer.msgType = LCDMsgTypePrint;
	strncpy((char *)lcdBuffer.buf,pString,vtLCDMaxLen);
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE SendLCDStateMsg(vtLCDStruct *lcdData,uint8_t algState, uint8_t macroState, portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	lcdBuffer.buf[0] = algState;
	lcdBuffer.buf[1] = macroState;
	lcdBuffer.msgType = LCDMsgTypeState;
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

portBASE_TYPE SendLCDADC(vtLCDStruct *lcdData,int length, uint8_t *value,portTickType ticksToBlock)
{
	if (lcdData == NULL) {
		VT_HANDLE_FATAL_ERROR(0);
	}
	vtLCDMsg lcdBuffer;

	if (length > vtLCDMaxLen) {
		// no room for this message
		VT_HANDLE_FATAL_ERROR(lcdBuffer.length);
	}
	lcdBuffer.length = length;
	lcdBuffer.msgType = LCDMsgTypeADC;
	int i;
	for( i = 0; i < length; i = i+1 ) {
		lcdBuffer.buf[i] = value[i];
	}
	return(xQueueSend(lcdData->inQ,(void *) (&lcdBuffer),ticksToBlock));
}

// Private routines used to unpack the message buffers
//   I do not want to access the message buffer data structures outside of these routines
portTickType unpackTimerMsg(vtLCDMsg *lcdBuffer)
{
	portTickType *ptr = (portTickType *) lcdBuffer->buf;
	return(*ptr);
}

int getMsgType(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->msgType);
} 

int getMsgLength(vtLCDMsg *lcdBuffer)
{
	return(lcdBuffer->length);
}

uint8_t *getValueADC(vtLCDMsg *lcdStruct)
{
	uint8_t *ptr = lcdStruct->buf;
	return ptr;
}

void copyMsgString(char *target,vtLCDMsg *lcdBuffer,int targetMaxLen)
{
	strncpy(target,(char *)(lcdBuffer->buf),targetMaxLen);
}

void initGraph()
{  	
	int i;
	for(i = 0; i < 230; i = i+1) {
		if((230-i)%39 == 0) {
			GLCD_PutPixel(9, i);
			GLCD_PutPixel(8, i);
			}
		GLCD_PutPixel(10, i);
	}
	for(i = 0; i < 320; i = i+1) {
		GLCD_PutPixel(i, 220);
		if((i + 11)%22 == 0) {
			GLCD_PutPixel(i,221);
			GLCD_PutPixel(i,222);
			}
	}
   	GLCD_DisplayString(29, 13, 0, (unsigned char* )"Y: Volts (1V), X: Time (10ms)");
}

void initReadout()
{  	
	int i;
	for(i = 0; i < 320; i = i+1) {
		GLCD_PutPixel(i, 65);
	}
   	GLCD_DisplayString(29, 13, 0, (unsigned char* )"Y: Volts (1V), X: Time (10ms)");
}

// End of private routines for message buffers

// If LCD_EXAMPLE_OP=0, then accept messages that may be timer or print requests and respond accordingly
// If LCD_EXAMPLE_OP=1, then do a rotating ARM bitmap display
#define LCD_EXAMPLE_OP 0
#if LCD_EXAMPLE_OP==1
// This include the file with the definition of the ARM bitmap
#include "ARM_Ani_16bpp.c"
#endif

static unsigned short hsl2rgb(float H,float S,float L);

// This is the actual task that is run
static portTASK_FUNCTION( vLCDUpdateTask, pvParameters )
{
	#if LCD_EXAMPLE_OP==0
	unsigned short screenColor = 0;
	unsigned short tscr;
	unsigned char curLine, curFrame;
	unsigned timerCount = 0;
	#elif LCD_EXAMPLE_OP==1
	unsigned char picIndex = 0;
	#else
	Bad definition
	#endif
	vtLCDMsg msgBuffer;
	vtLCDStruct *lcdPtr = (vtLCDStruct *) pvParameters;

	#ifdef INSPECT_STACK
	// This is meant as an example that you can re-use in your own tasks
	// Inspect to the stack remaining to see how much room is remaining
	// 1. I'll check it here before anything really gets started
	// 2. I'll check during the run to see if it drops below 10%
	// 3. You could use break points or logging to check on this, but
	//    you really don't want to print it out because printf() can
	//    result in significant stack usage.
	// 4. Note that this checking is not perfect -- in fact, it will not
	//    be able to tell how much the stack grows on a printf() call and
	//    that growth can be *large* if version 1 of printf() is used.   
	unsigned portBASE_TYPE InitialStackLeft = uxTaskGetStackHighWaterMark(NULL);
	unsigned portBASE_TYPE CurrentStackLeft;
	float remainingStack = InitialStackLeft;
	remainingStack /= lcdSTACK_SIZE;
	if (remainingStack < 0.10) {
		// If the stack is really low, stop everything because we don't want it to run out
		// The 0.10 is just leaving a cushion, in theory, you could use exactly all of it
		VT_HANDLE_FATAL_ERROR(0);
	}
	#endif

	/* Initialize the LCD and set the initial colors */
	GLCD_Init();
	tscr = White; // may be reset in the LCDMsgTypeTimer code below
	screenColor = Black; // may be reset in the LCDMsgTypeTimer code below
	GLCD_SetTextColor(tscr);
	GLCD_SetBackColor(screenColor);
	GLCD_Clear(screenColor);
	#if USE_MS1_CODE == 1
	initGraph();
	#endif
	initReadout();

	curLine = 3;
	curFrame = 0;
	// This task should never exit
	for(;;)
	{	
		#ifdef INSPECT_STACK   
		CurrentStackLeft = uxTaskGetStackHighWaterMark(NULL);
		float remainingStack = CurrentStackLeft;
		remainingStack /= lcdSTACK_SIZE;
		if (remainingStack < 0.10) {
			// If the stack is really low, stop everything because we don't want it to run out
			VT_HANDLE_FATAL_ERROR(0);
		}
		#endif

		#if LCD_EXAMPLE_OP==0
		// Wait for a message
		if (xQueueReceive(lcdPtr->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		
		//Log that we are processing a message -- more explanation of logging is given later on
		vtITMu8(vtITMPortLCDMsg,getMsgType(&msgBuffer));
		vtITMu8(vtITMPortLCDMsg,getMsgLength(&msgBuffer));

		uint8_t dataPoints[140];
		int i = 0;
		int yVal = 0;

		// Take a different action depending on the type of the message that we received
		switch(getMsgType(&msgBuffer)) {
		case LCDMsgTypePrint: {
			char   lineBuffer[lcdCHAR_IN_LINE+1];
			copyMsgString(lineBuffer,&msgBuffer,lcdCHAR_IN_LINE);
			// clear the line
			GLCD_ClearLn(curLine,1);
			// show the text
			GLCD_DisplayString(curLine,0,1,(unsigned char *)lineBuffer);
			curLine++;
			if (curLine == 10) {
				//GLCD_Clear(screenColor);
				curLine = 3;
			}
			break;
		}
		case LCDMsgTypeState: {
			uint8_t   alg = msgBuffer.buf[0];
			uint8_t   mac = msgBuffer.buf[1];
			char   *macroState;
			char   *algState;
			int ceil_halflen[] = {0,0};
			switch (alg) {
			case ALG_FORWARD:
				algState = "FORWARD";
				ceil_halflen[0] = 4;
				break;
			case ALG_STOPPED:
				algState = "STOPPED";
				ceil_halflen[0] = 4;
				break;
			case ALG_AGAINST_OBSTACLE:
				algState = "AGAINST_OBSTACLE";
				ceil_halflen[0] = 8;
				break;
			case ALG_CLEARING:
				algState = "CLEARING";
				ceil_halflen[0] = 4;
				break;
			case ALG_ON_CORNER:
				algState = "ON_CORNER";
				ceil_halflen[0] = 5;
				break;
			default:
				algState = "ERROR";
				ceil_halflen[0] = 3;
				break;
			}
			switch (mac) {
			case MACROSTATE_IDLE:
				macroState = "IDLE";
				ceil_halflen[1] = 2;
				break;
			case MACROSTATE_FINDING_LINE:
				macroState = "FINDING_LINE";
				ceil_halflen[1] = 6;
				break;
			case MACROSTATE_RUN_ONE:
				macroState = "RUN_ONE";
				ceil_halflen[1] = 4;
				break;
			case MACROSTATE_FINISHED:
				macroState = "FINISHED";
				ceil_halflen[1] = 4;
				break;
			}
			// clear the line
			GLCD_ClearLn(1,1);
			GLCD_ClearLn(0,1);
			// show the text
			GLCD_DisplayString(1,10-ceil_halflen[0],1,(unsigned char *)algState);
			GLCD_DisplayString(0,10-ceil_halflen[1],1,(unsigned char *)macroState);
			break;
		}
		case LCDMsgTypeTimer: {
			// Note: if I cared how long the timer update was I would call my routine
			//    unpackTimerMsg() which would unpack the message and get that value
			timerCount++;
			if (timerCount >= 100) {
				GLCD_ClearLn(curLine,1);
				// show the text
				char lineBuffer2[6] = "Test";
				GLCD_DisplayString(curLine,0,1,(unsigned char *)lineBuffer2);
				curLine++;
				if (curLine == lcdNUM_LINES) {
					curLine = 5;
				}
				timerCount = 0;
			}
			break;
		}
		case LCDMsgTypeADC: {
			if (curFrame == 0) {
				//GLCD_ClearWindow(11,0,309,219,Black); GLCD_ClearWindow(10,0,1,220,White);
			}
			int XZERO = 11 + (curFrame*60);
			int YZERO = 220;
			if(curFrame == 4)
				curFrame = 0;
			else
				curFrame++;

			GPIO_SetValue(0,0x10000);
			uint8_t *ptr = getValueADC(&msgBuffer);
			for( i = 0; i < getMsgLength(&msgBuffer); i = i+1) {
				yVal = ptr[i]/2;
				GLCD_ClearWindow(XZERO + i*3, YZERO - dataPoints[curFrame*20+i],1,1,Black);
				GLCD_ClearWindow(XZERO + i*3, YZERO - yVal,1,1,White);
				GLCD_PutPixel(XZERO + i*3, YZERO - yVal);
				dataPoints[curFrame*20+i] = yVal;
		    }
			GPIO_ClearValue(0,0x10000);
			//handle writing frame of ADC data as graph
			break;
		}
		default: {
			// In this configuration, we are only expecting to receive timer messages
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
			break;
		}
		} // end of switch()

		// Here is a way to do debugging output via the built-in hardware -- it requires the ULINK cable and the
		//   debugger in the Keil tools to be connected.  You can view PORT0 output in the "Debug(printf) Viewer"
		//   under "View->Serial Windows".  You have to enable "Trace" and "Port0" in the Debug setup options.  This
		//   should not be used if you are using Port0 for printf()
		// There are 31 other ports and their output (and port 0's) can be seen in the "View->Trace->Records"
		//   windows.  You have to enable the prots in the Debug setup options.  Note that unlike ITM_SendChar()
		//   this "raw" port write is not blocking.  That means it can overrun the capability of the system to record
		//   the trace events if you go too quickly; that won't hurt anything or change the program execution and
		//   you can tell if it happens because the "View->Trace->Records" window will show there was an overrun.
		//vtITMu16(vtITMPortLCD,screenColor);

		#elif 	LCD_EXAMPLE_OP==1
		// In this alternate version, we just keep redrawing a series of bitmaps as
		//   we receive timer messages
		// Wait for a message
		if (xQueueReceive(lcdPtr->inQ,(void *) &msgBuffer,portMAX_DELAY) != pdTRUE) {
			VT_HANDLE_FATAL_ERROR(0);
		}
		if (getMsgType(&msgBuffer) != LCDMsgTypeTimer) {
			// In this configuration, we are only expecting to receive timer messages
			VT_HANDLE_FATAL_ERROR(getMsgType(&msgBuffer));
		}
  		/* go through a  bitmap that is really a series of bitmaps */
		picIndex = (picIndex + 1) % 9;
		GLCD_Bmp(99,99,120,45,(unsigned char *) &ARM_Ani_16bpp[picIndex*(120*45*2)]);
		#else
		Bad setting
		#endif	
	}
}

// Convert from HSL colormap to RGB values in this weird colormap
// H: 0 to 360
// S: 0 to 1
// L: 0 to 1
// The LCD has a funky bitmap.  Each pixel is 16 bits (a "short unsigned int")
//   Red is the most significant 5 bits
//   Blue is the least significant 5 bits
//   Green is the middle 6 bits
static unsigned short hsl2rgb(float H,float S,float L)
{
	float C = (1.0 - fabs(2.0*L-1.0))*S;
	float Hprime = H / 60;
	unsigned short t = Hprime / 2.0;
	t *= 2;
	float X = C * (1-abs((Hprime - t) - 1));
	unsigned short truncHprime = Hprime;
	float R1, G1, B1;

	switch(truncHprime) {
		case 0: {
			R1 = C; G1 = X; B1 = 0;
			break;
		}
		case 1: {
			R1 = X; G1 = C; B1 = 0;
			break;
		}
		case 2: {
			R1 = 0; G1 = C; B1 = X;
			break;
		}
		case 3: {
			R1 = 0; G1 = X; B1 = C;
			break;
		}
		case 4: {
			R1 = X; G1 = 0; B1 = C;
			break;
		}
		case 5: {
			R1 = C; G1 = 0; B1 = X;
			break;
		}
		default: {
			// make the compiler stop generating warnings
			R1 = 0; G1 = 0; B1 = 0;
			VT_HANDLE_FATAL_ERROR(Hprime);
			break;
		}
	}
	float m = L - 0.5*C;
	R1 += m; G1 += m; B1 += m;
	unsigned short red = R1*32; if (red > 31) red = 31;
	unsigned short green = G1*64; if (green > 63) green = 63;
	unsigned short blue = B1*32; if (blue > 31) blue = 31;
	unsigned short color = (red << 11) | (green << 5) | blue;
	return(color); 
}
