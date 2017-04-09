// Interpreter.c
// Interpreter for UART
// TA: Daniel Leach

#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"
#include <stdint.h>
#include "../LiuWareTM4C123Lab3/ST7735.h"
#include "../LiuWareTM4C123Lab3/UART.h"
#include "../LiuWareTM4C123Lab3/OS.h"
#include "../LiuWareTM4C123Lab3/ADC.h"
#include "../LiuWareTm4C123Lab3/Interpreter.h"

#define NULLCHAR '\0'

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
//----- Internal Helper Functions -----
int StringToInt(char * stringNum, int len);
int strLengthByNullCount(char * stringNum);
int strLengthBySpaceCount(char * stringNum);
void OutCRLF(void);
//----- UART Defines ----------
#define MAX_UART_IN_LEN 100

//----- Global ------
char uartString[MAX_UART_IN_LEN];  // UART puts string into here

extern unsigned long numCreated;   // number of foreground threads created
extern int numThreads;
extern int debugBlocked;
extern unsigned long NumSamples;
extern Sema4Type mutexLCD;
extern Sema4Type CurrentSize;
extern Sema4Type RoomLeft;
extern Sema4Type FIFOmutex;
extern Sema4Type mailSend;
extern Sema4Type mailAck; 

#define FS 400              // producer/consumer sampling
#define RUNLENGTH (5*FS)   // display results and quit when NumSamples==RUNLENGTH
/*---------------------ProcessCommand---------------------
* Function: ProcessCommand is a busy wait function that waits on the UART_InString to be available. 
*	Once ready, we process the string from UART. 
*/
void ProcessCommand(){
	/* variables */
		// LCD - allows UART to output to LCD
	int lineNum; 
	int deviceNum;
		// ADC
	int adcChannel; 
	//int adcFreq; - legacy from lab 1, the frequency is controlled by ADC_Collect_Init and we never re-implimented it
	/* Get Command */
	UART_OutString("$"); //linux style prompting
	UART_InString(uartString,MAX_UART_IN_LEN-1); //busy wait for UART input
	OutCRLF(); //styling for interpreter terminal
	
	/* Command Processing */
	switch(uartString[0]) {
		/* Case A */
		case 'A':
		case 'a':
			if(uartString[3] == 'S' || uartString[3] == 's') {
				adcChannel = StringToInt(uartString+13, strLengthByNullCount(uartString+13)); 
				ADC_Init(adcChannel);
				UART_OutUDec(ADC_In());
				OutCRLF();				
			}
			break; 	
		/* Case C */
		case 'C':
		case 'c': 
			OS_ClearMsTime();
			break;
		case 'D':
		case 'd':
			UART_OutString("RUNLENGTH ");
			UART_OutUDec(RUNLENGTH); 
			OutCRLF();
					UART_OutString("NumSamples ");
			UART_OutUDec(NumSamples); 
			OutCRLF();
			UART_OutString("debugBlocked ");
			UART_OutUDec(debugBlocked); 
			OutCRLF();
			UART_OutString("mutexLCD ");
			UART_OutUDec(mutexLCD.Value); 
			OutCRLF();
			UART_OutString("CurrentSize ");
			UART_OutUDec(CurrentSize.Value); 
			OutCRLF();
		UART_OutString("RoomLeft ");
			UART_OutUDec(RoomLeft.Value); 
			OutCRLF();
		UART_OutString("FIFOmutex ");
			UART_OutUDec(FIFOmutex.Value); 
			OutCRLF();
		UART_OutString("mailSend ");
			UART_OutUDec(mailSend.Value); 
			OutCRLF();
		UART_OutString("mailAck ");
			UART_OutUDec(mailAck.Value); 
			OutCRLF();
			break;
		/* Case H */ 
		case 'H':
		case 'h':
			UART_OutString("     messageString -d 0 -l 0 Hi");
			OutCRLF();
			UART_OutString("     messageDec -d 0 -l 0 Hi");
			OutCRLF();
			UART_OutString("     clearOSTime");
			OutCRLF();
			UART_OutString("     readOSTime");
			OutCRLF();
			UART_OutString("     adcSample -c 0");
			OutCRLF();
			UART_OutString("     Threads Made(NumCreated)");
			OutCRLF();
			UART_OutString("     NumThreads Alive (numThreads)");
			OutCRLF();
			UART_OutString("     Debug");
			OutCRLF();
			break;
		/* Case M */ 
		case 'M':
		case 'm': 
			if(uartString[7] == 'S' || uartString[7] == 's') {
				lineNum = uartString[22] - '0'; 
				deviceNum = uartString[17] - '0';
				ST7735_MessageString(deviceNum, lineNum, uartString+24);
			}
			if(uartString[7] == 'D' || uartString[7] == 'd') {
				lineNum = uartString[19] - '0'; 
				deviceNum = uartString[14] - '0';
				int tempInputNum = StringToInt(uartString+21, strLengthByNullCount(uartString+21)); 
				ST7735_MessageDec(deviceNum, lineNum, tempInputNum);
			}
			break; 
		case 'N':
		case 'n':
			UART_OutString("num threads alive: ");
			UART_OutUDec(numThreads); 
			OutCRLF();
			break;
		case 'R':
		case 'r':
			UART_OutUDec(OS_Time());
			//UART_OutUDec(OS_MsTime()/1000);
			OutCRLF();
			break;
		case 'T':
		case 't':
			UART_OutString("num threads created: ");
			UART_OutUDec(numCreated); 
			OutCRLF();
	} //end switch
}

/***************** StringToInt **************
* Input: string and length of string
* Output: int version of string
*/
int StringToInt(char * stringNum, int len) {
	int i, dec = 0; 
	for(i=0; i<len; i++){
		dec = dec * 10 + ( stringNum[i] - '0' );
	}
	return dec; 
}

/***************** strLengthByNullCount **************
* Input: string
* Output: length of string or the logical count of characters before null
*/
int strLengthByNullCount(char * stringNum) {
	int i = 0;
	while (*(stringNum + i) != NULLCHAR) {
		i++;
	}
	return i; 
}

/***************** strLengthBySpaceCount **************
* Input: string
* Output: length of word or the logical count of characters before space
*/
int strLengthBySpaceCount(char * stringNum) {
	int i = 0;
	while (*(stringNum + i) != ' ') {
		i++; 
	}
	return i; 
}

//---------------------OutCRLF---------------------
// Output a CR,LF to UART to go to a new line
// Input: none
// Output: none
void OutCRLF(void){
  UART_OutChar(CR);
  UART_OutChar(LF);
}
