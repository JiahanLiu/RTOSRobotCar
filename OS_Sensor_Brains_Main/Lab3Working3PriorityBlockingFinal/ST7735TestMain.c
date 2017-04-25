// Lab3.c
// Calls Lab 1 Main
// TA: Daniel  Leach
// PORT/Timer Uses:
/*
	PORT B: debugging when each thread is in use
	PORT F: switches
	ADC Sequencer 2: ADC_In();
	ADC Sequencer 3: ADC Collect();
	SysTick: Thread Switching
	PendSV: Thread Switching when Kill
	Timer4: Sleep Decrement + OS_Time
	Timer1: Periodic Thread 1
	Timer0: Periodic Thread 2
	Timer2: ADC
	
	New Timers
	Timer 0: IR
	Timer 1: IR
	Timer 2: IR
	Timer 3: IR
	Timer 4: CAN
	Timer 5: Sleep Decrement + OS_Time
	//Timer 6: Periodic Thread for Pulse
*/


#include <stdio.h>
#include <stdint.h>
#include "../LiuWareTM4C123Lab3/PLL.h"
#include "../LiuWareTM4C123Lab3/OS.h"
#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"
#include "../LiuWareTM4C123Lab3/ST7735.h"
#include "../LiuWareTM4C123Lab3/UART.h"
#include "../LiuWareTm4C123Lab3/ADC.h"
#include "../LiuWareTm4C123Lab3/Interpreter.h"
#include "../LiuWareTm4C123Lab3/Timer0A.h"
#include "../LiuWareTm4C123Lab3/Timer1.h"
#include "../LiuWareTM4C123Lab3/LEDS.h" 
#include "../LiuWareTM4C123Lab3/Filter.h" 
#include "../LiuWareTM4C123Lab3/CAN/can0.h"
#include "../LiuWareTM4C123Lab3/CAN/Timer4.h" //used for sleep decrement + OS Time

#define PB2  (*((volatile unsigned long *)0x40005010))
#define PB3  (*((volatile unsigned long *)0x40005020))
#define PB4  (*((volatile unsigned long *)0x40005040))
#define PB5  (*((volatile unsigned long *)0x40005080))
#define PB7  (*((volatile unsigned long *)0x40005200))
	
#define PERIOD1MS 80000

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void PortB_Init(void); 

#define FS 400              // producer/consumer sampling
#define RUNLENGTH (2000 * FS )   // display results and quit when NumSamples==RUNLENGTH

//data variables
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
unsigned long DataLost = 0;     // data sent by Producer, but not received by Consumer

//debug variables
unsigned long numCreated = 0;   // number of foreground threads created
int numThreads = 0;

//------------------Interpreter--------------------------------
// UART background ISR performs serial input/output
// Two software fifos are used to pass I/O data to foreground
// The interpreter runs as a foreground thread
// The UART driver should call OS_Wait(&RxDataAvailable) when foreground tries to receive
// The UART ISR should call OS_Signal(&RxDataAvailable) when it receives data from Rx
// Similarly, the transmit channel waits on a semaphore in the foreground
// and the UART ISR signals this semaphore (TxRoomLeft) when getting data from fifo
// Modify your intepreter from Lab 1, adding commands to help debug 
// Interpreter is a foreground thread, accepts input from serial port, outputs to serial port
// inputs:  none
// outputs: none

void Interpreter(void) { //lab 1 thread				
	while(1) {
		ProcessCommand();
	}
}
//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 

//--------------End of Interpreter-----------------------------

// 20-sec finite time experiment duration 
#define PERIOD TIME_500US   // DAS 2kHz sampling period in system time units
unsigned long xBottomLeft[64],yBottomLeft[64];           // input and output arrays for FFT
unsigned long xTopLeft[64],yTopLeft[64];           // input and output arrays for FFT
unsigned long xTopRight[64],yTopRight[64];           // input and output arrays for FFT
unsigned long xBottomRight[64],yBottomRight[64];           // input and output arrays for FFT

//------------------Display on Screen 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
	unsigned long myId = OS_Id(); 
  ST7735_Message(1,0,"NumCreated =",numCreated); 
  OS_Sleep(50);     // set this to sleep for 50msec
  ST7735_Message(1,1,"PIDWork     =",PIDWork);
  ST7735_Message(1,2,"DataLost    =",DataLost); 
  ST7735_Message(1,3,"Jitter 0.1us=",getMaxJitterT1()); 
  OS_Kill();  // done, OS does not return from a Kill
} 

//************SW1Push*************
// Called when SW1 Button pushed
// Adds another foreground task
// background threads execute once and return
void SW1Push(void){
	numCreated += OS_AddThread(&ButtonWork,100,2);
	OS_Kill();
}
//************SW2Push*************
// Called when SW2 Button pushed, Lab 3 only
// Adds another foreground task
// background threads execute once and return
void SW2Push(void){
  numCreated += OS_AddThread(&ButtonWork,100,2);
	OS_Kill();
}
//--------------------End of Display on Screen 2------------------------

//------------------Hardware Triggered ADC--------------------------------
// hardware timer-triggered ADC sampling at 400Hz
// Producer runs as part of ADC ISR
// Producer uses fifo to transmit 400 samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement

//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 400Hz, started by your ADC_Collect
// The timer triggers the ADC, creating the 400Hz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 12-bit sample 
// sends data to the consumer, runs periodically at 400Hz
// inputs:  none
// outputs: none
int result[3];
void Producer(IR_Data_Type data){  
  if(NumSamples < RUNLENGTH){   // finite time run
    NumSamples++;               // number of samples
    if(OS_Fifo_Put(data) == -1){ // send to consumer
      DataLost++;
    }
  } 
}
void Display(void); 
void StateMachine(void);

int calibrationChartBottomLeft[15][2] = {
	{10,2857},
	{15,1961},
	{20,1493},
	{25,1231},
	{30,1044},
	{35,879},
	{40,783},
	{45,688},
	{50,552},
	{55,441},
	{60,345},
	{65,251},
	{70,225},
	{75,201},
	{80,154}
};

int calibrationChartTopLeft[15][2] = {
	{10,2886},
	{15,1972},
	{20,1544},
	{25,1233},
	{30,1022},
	{35,884},
	{40,764},
	{45,669},
	{50,585},
	{55,471},
	{60,374},
	{65,326},
	{70,255},
	{75,200},
	{80,176}
};

int calibrationChartTopRight[15][2] = {
	{10,2832},
	{15,2009},
	{20,1539},
	{25,1249},
	{30,1059},
	{35,889},
	{40,759},
	{45,640},
	{50,573},
	{55,472},
	{60,440},
	{65,385},
	{70,312},
	{75,287},
	{80,201}
};

int calibrationChartBottomRight[15][2] = {
	{10,2932},
	{15,2096},
	{20,1615},
	{25,1326},
	{30,1092},
	{35,952},
	{40,763},
	{45,572},
	{50,442},
	{55,346},
	{60,297},
	{65,259},
	{70,220},
	{75,172},
	{80,153}
};

int adcToDistance(int adcValue, int sensorNumber) {
	if(sensorNumber == 0) {
		for(int i = 0; i < 14; i++) {
			if(adcValue <= calibrationChartBottomLeft[i][1])
			{
				if(adcValue > calibrationChartBottomLeft[i+1][1])
				{
						//linear interpolate
						return -5*(adcValue-calibrationChartBottomLeft[i][1])/(calibrationChartBottomLeft[i][1]-calibrationChartBottomLeft[i+1][1])+calibrationChartBottomLeft[i][0];
				}
			}else{
				return 10;
			}
		}
		return 80;
	} else if(sensorNumber == 1) {
		for(int i = 0; i < 14; i++) {
			if(adcValue <= calibrationChartTopLeft[i][1])
			{
				if(adcValue > calibrationChartTopLeft[i+1][1])
				{
						//linear interpolate
						return -5*(adcValue-calibrationChartTopLeft[i][1])/(calibrationChartTopLeft[i][1]-calibrationChartTopLeft[i+1][1])+calibrationChartTopLeft[i][0];
				}
			}else{
				return 10;
			}
		}
	} else if(sensorNumber == 2) {
		for(int i = 0; i < 14; i++) {
			if(adcValue <= calibrationChartTopRight[i][1])
			{
				if(adcValue > calibrationChartTopRight[i+1][1])
				{
						//linear interpolate
						return -5*(adcValue-calibrationChartTopRight[i][1])/(calibrationChartTopRight[i][1]-calibrationChartTopRight[i+1][1])+calibrationChartTopRight[i][0];
				}
			}else{
				return 10;
			}
		}
	} else if(sensorNumber == 3) {
				for(int i = 0; i < 14; i++) {
			if(adcValue <= calibrationChartBottomRight[i][1])
			{
				if(adcValue > calibrationChartBottomRight[i+1][1])
				{
						//linear interpolate
						return -5*(adcValue-calibrationChartBottomRight[i][1])/(calibrationChartBottomRight[i][1]-calibrationChartBottomRight[i+1][1])+calibrationChartBottomRight[i][0];
				}
			}else{
				return 10;
			}
		}
	} 
}
//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
void Consumer(void){ 
IR_Data_Type data;
IR_Data_Type DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
unsigned long t;                  // time in 2.5 ms
unsigned long myId = OS_Id(); 
	DataLost = 0; 
  IR_Sensor_Init(FS, &Producer); // start ADC sampling, channel 5, PD2, 400 Hz
  numCreated += OS_AddThread(&Display,128,1);
	//numCreated += OS_AddThread(&StateMachine,128,1);	
  while(NumSamples < RUNLENGTH - 32) { 
    for(t = 0; t < 64; t++){   // collect 64 ADC samples
      data = OS_Fifo_Get();    // get from producer - if we get stuck here then display may have been killed
			xBottomLeft[t] = data.BottomLeft;             // real part is 0 to 4095, imaginary part is 0
			xTopLeft[t] = data.TopLeft;
			xTopRight[t] = data.TopRight;
			xBottomRight[t] = data.BottomRight;
    }
    cr4_fft_64_stm32(yBottomLeft,xBottomLeft,64);  // complex FFT of last 64 ADC values
		cr4_fft_64_stm32(yTopLeft,xTopLeft,64);  // complex FFT of last 64 ADC values
		cr4_fft_64_stm32(yTopRight,xTopRight,64);  // complex FFT of last 64 ADC values
		cr4_fft_64_stm32(yBottomRight,xBottomRight,64);  // complex FFT of last 64 ADC values
    DCcomponent.BottomLeft = yBottomLeft[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
		DCcomponent.TopLeft = yTopLeft[0]&0xFFFF;
		DCcomponent.TopRight = yTopRight[0]&0xFFFF;
    DCcomponent.BottomRight = yBottomRight[0]&0xFFFF;
		OS_MailBox_Send(DCcomponent); // called every 2.5ms*64 = 160ms
  }
	LEDS = RED;
  OS_Kill();  // never called
}
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){
IR_Data_Type data;
IR_Data_Type distance;
unsigned long myId2 = OS_Id(); 
  ST7735_Message(0,1,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
  while(NumSamples < RUNLENGTH - 32) {
		IR_Data_Type tempData;
    tempData = OS_MailBox_Recv();
    data = tempData;
		distance.BottomLeft = adcToDistance(data.BottomLeft, 0);               // calibrate your device so voltage is in mV
		distance.TopLeft = adcToDistance(data.TopLeft, 1);
		distance.TopRight = adcToDistance(data.TopRight, 2);
		distance.BottomRight = adcToDistance(data.BottomRight, 3);
    //ST7735_Message(0,2,"v(mV) =",distance.BottomLeft);
		//ST7735_Message(0,3,"v(mV) =",distance.TopLeft);
		//ST7735_Message(0,4,"v(mV) =",distance.TopRight);
		//ST7735_Message(0,5,"v(mV) =",distance.BottomRight);
		
		UART_OutString("DISTANCE -----------");
		OutCRLF();
		UART_OutUDec(distance.BottomLeft);
		OutCRLF();
		UART_OutUDec(distance.TopLeft);
		OutCRLF();
		UART_OutUDec(distance.TopRight);
		OutCRLF();
		UART_OutUDec(distance.BottomRight);
		OutCRLF();
		
		/*
		UART_OutString("ADC VAlue -----------");
		OutCRLF();
		UART_OutUDec(data.BottomLeft);
		OutCRLF();
		UART_OutUDec(data.TopLeft);
		OutCRLF();
		UART_OutUDec(data.TopRight);
		OutCRLF();
		UART_OutUDec(data.BottomRight);
		OutCRLF(); 
		*/
  }
	LEDS = RED;
  OS_Kill();  // never called
} 

int inBetween(int value, int low, int high) {
	if(value >= low && value < high) {
		return 1;
	}
	return 0;
}

uint8_t XmtData[8];
uint8_t RcvData[8];
uint32_t RcvCount=0;
uint32_t lastState = 1;
void StateMachine(void){ 
IR_Data_Type data;
IR_Data_Type distance;
unsigned long myId2 = OS_Id(); 
  while(NumSamples < RUNLENGTH - 32) {
		IR_Data_Type tempData;
    tempData = OS_MailBox_Recv();
    data = tempData;
		distance.BottomLeft = adcToDistance(data.BottomLeft, 0);               // calibrate your device so voltage is in mV
		distance.TopLeft = adcToDistance(data.TopLeft, 1);
		distance.TopRight = adcToDistance(data.TopRight, 2);
		distance.BottomRight = adcToDistance(data.BottomRight, 3);
		
		/*
		UART_OutString("Start -----------");
		OutCRLF();
		UART_OutUDec(distance.BottomLeft);
		OutCRLF();
		UART_OutUDec(distance.TopLeft);
		OutCRLF();
		UART_OutUDec(distance.TopRight);
		OutCRLF();
		UART_OutUDec(distance.BottomRight);
		OutCRLF();
		*/
		
		//CAN
		
		//straight case
		if(inBetween(distance.BottomLeft, 15, 30) && inBetween(distance.BottomRight, 15, 30) 
			&& inBetween(distance.TopLeft, 25, 55) && inBetween(distance.TopLeft, 25, 55)) {
			if(lastState == 1) {				
				XmtData[0] = 1;//PF0<<1;  // 0 or 2
				CAN0_SendData(XmtData);
				OS_Sleep(1);
			}
			lastState = 1;
		//slight left
		} else if(inBetween(distance.BottomLeft, 10, 15)  
			&& inBetween(distance.TopLeft, 25, 55) && inBetween(distance.TopLeft, 25, 55)) {
			if(lastState == 2) {				
				XmtData[0] = 2;//PF0<<1;  // 0 or 2
				CAN0_SendData(XmtData);
				OS_Sleep(1);
			}
			lastState = 2;
		//slight right
		} else if(inBetween(distance.BottomRight, 10, 15)  
			&& inBetween(distance.TopLeft, 25, 55) && inBetween(distance.TopLeft, 25, 55)) {
			if(lastState == 3) {				
				XmtData[0] = 3;//PF0<<1;  // 0 or 2
				CAN0_SendData(XmtData);
				OS_Sleep(1);
			}
			lastState = 3;
		//big left
		} else if(inBetween(distance.TopLeft, 55, 81)) { //must put 81 not 80 because of bounding in inBetween
			if(lastState == 4) {				
				XmtData[0] = 4;//PF0<<1;  // 0 or 2
				CAN0_SendData(XmtData);
				OS_Sleep(1);
			}
			lastState = 4;
		//big right
		} else if(inBetween(distance.TopRight, 55, 81)) { //must put 81 not 80 because of bounding in inBetween
			if(lastState == 5) {				
				XmtData[0] = 5;//PF0<<1;  // 0 or 2
				CAN0_SendData(XmtData);
				OS_Sleep(1);
			}
			lastState = 5;
		} else if(inBetween(distance.TopLeft, 10, 15) && inBetween(distance.TopLeft, 10, 15)) {
			if(lastState == 0) {				
				XmtData[0] = 0;//PF0<<1;  // 0 or 2
				CAN0_SendData(XmtData);
				OS_Sleep(1);
			}
			lastState = 0;
		//slight left
		}
		
  }
	LEDS = RED;
  OS_Kill();  // never called
} 


//-----------------End of Hardware Triggered ADC---------------------------

//--------------------------PID------------------------------
// foreground thread that runs without waiting or sleeping
// it executes a digital controller 
//******** PID *************** 
// foreground thread, runs a PID controller
// never blocks, never sleeps, never dies
// inputs:  none
// outputs: none
// Jiahan Liu: Lab 2.2 Does only once
short IntTerm;     // accumulated error, RPM-sec
short PrevError;   // previous error, RPM
short Coeff[3];    // PID coefficients
short Actuator;
void PID(void){ 
short err;  // speed error, range -100 to 100 RPM
unsigned long myId = OS_Id(); 
  PIDWork = 0;
  IntTerm = 0;
  PrevError = 0;
  Coeff[0] = 384;   // 1.5 = 384/256 proportional coefficient
  Coeff[1] = 128;   // 0.5 = 128/256 integral coefficient
  Coeff[2] = 64;    // 0.25 = 64/256 derivative coefficient*
  while(NumSamples < RUNLENGTH) { 
    for(err = -1000; err <= 1000; err++){    // made-up data
      Actuator = PID_stm32(err,Coeff)/256;
    }
    PIDWork++;        // calculation finished
  }
  for(;;){
		PB3 ^= 0x08;
	}          // done
}
//--------------------------End of PID------------------------------

extern unsigned int pulsePeriodPing6_7;
extern int pingPB6_7_ready;
void PingTest(void) {
	Timer0A_Init();
	StartPingSensorPB6_7();
}

uint8_t sequenceNum=0;  
void CanSendMessage(void) {
  XmtData[0] = 'H';//PF0<<1;  // 0 or 2
  XmtData[1] = 'I';//PF4>>2;  // 0 or 4
  XmtData[2] = 'J';//0;       // unassigned field
  XmtData[3] = 'I';//sequenceNum;  // sequence count
  XmtData[4] = 'M';
  XmtData[5] = 'B';
  XmtData[6] = 'O';
  XmtData[7] = 'O';
  CAN0_SendData(XmtData);
  sequenceNum++;
}
void CANTest() {
  while(1){
    if(CAN0_GetMailNonBlock(RcvData)){
      RcvCount++;
    }
	}	
}

void postLauntInits(){
	long sr = StartCritical();
	CAN0_Open();
	//OS_AddPeriodicThread(&CanSendMessage,FS*2,2);           // 1 ms, higher priority
	//Timer4_Init(&UserTask, 1600000); // initialize timer3 (10 Hz) // Sleep
	EndCritical(sr);
	UART_Init();
	UART_OutString("Hello Lab 6.0.1");
	OutCRLF();
	OS_Kill();
}


//*******************final user main DEMONTRATE THIS TO TA**********
int main(void){
  OS_Init();           // initialize, disable interrupts
	
 
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
	FilterWork = 0;
  numCreated = 0;
//********initialize communication channels
  OS_MailBox_Init();		
  OS_Fifo_Init(32);

//*******attach background tasks***********
  //OS_AddSW1Task(&SW1Push,2); //not stuck
  //OS_AddSW2Task(&SW2Push,2);  // add this line in Lab 3
	
	ST7735_InitRDivided(INITR_REDTAB);
	///periodic test
	
	//create initial foreground threads
	
	numCreated += OS_AddThread(&postLauntInits,128,1);
	//numCreated += OS_AddThread(&Interpreter,128,5);
	//numCreated += OS_AddThread(&CANTest,128,2);
	numCreated += OS_AddThread(&Consumer,128,2); 
	//numCreated += OS_AddThread(&PingTest,128,2);  // Lab 3, make this lowest priority
  numCreated += OS_AddThread(&PID,128,6);  // Lab 3, make this lowest priority
	//numCreated += OS_AddThread(&ReadIRSensor,128,2);

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
