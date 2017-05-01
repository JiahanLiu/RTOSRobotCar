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
#include "../LiuWareTm4C123Lab3/JimString.h"
#include "../LiuWareTm4C123Lab3/Filter.h"

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
	/* Get Command */
	UART_OutString("$"); //linux style prompting
	UART_InString(uartString,MAX_UART_IN_LEN-1); //busy wait for UART input
	OutCRLF(); //styling for interpreter terminal
	
	/* Command Processing */
	switch(uartString[0]) {
		/* Case A */
		case 'A':
		case 'a':
			break; 	
		/* Case C */
		case 'C':
		case 'c': 
			OS_ClearMsTime();
			break;
		case 'D':
		case 'd':
//			numCreated += OS_AddThread(&FileSystemTest,128,1);
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
