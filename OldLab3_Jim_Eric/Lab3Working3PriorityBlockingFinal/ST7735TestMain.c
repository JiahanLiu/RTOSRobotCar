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
#include "../LiuWareTM4C123Lab3/LEDS.h" 
#include "../LiuWareTM4C123Lab3/Filter.h" 
#include "../LiuWareTM4C123Lab3/Timer3.h"

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
//#define FS 5
//#define RUNLENGTH (10)   // display results and quit when NumSamples==RUNLENGTH

//data variables
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
unsigned long DataLost = 0;     // data sent by Producer, but not received by Consumer

//debug variables
unsigned long numCreated = 0;   // number of foreground threads created
int numThreads = 0;
int debugBlocked = 0;

//------------------Task 5--------------------------------
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
		PB5 ^= 0x20;
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
//--------------end of Task 5-----------------------------

// 20-sec finite time experiment duration 
#define PERIOD TIME_500US   // DAS 2kHz sampling period in system time units
long x[64],y[64];           // input and output arrays for FFT

void PortB_Init(void){ 
	unsigned long volatile delay;
  SYSCTL_RCGCGPIO_R |= 0x02;       // activate port B
  delay = SYSCTL_RCGC2_R;
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTB_DIR_R |= 0x3C;    // make PB5-2 output heartbeats
  GPIO_PORTB_AFSEL_R &= ~0x3C;   // disable alt funct on PB5-2
  GPIO_PORTB_DEN_R |= 0x3C;     // enable digital I/O on PB5-2
  GPIO_PORTB_PCTL_R = ~0x00FFFF00;
  GPIO_PORTB_AMSEL_R &= ~0x3C;      // disable analog functionality on PB
} 

//------------------Task 1--------------------------------
// 2 kHz sampling ADC channel 1, using software start trigger
// background thread executed at 2 kHz
// 60-Hz notch high-Q, IIR filter, assuming fs=2000 Hz
// y(n) = (256x(n) -503x(n-1) + 256x(n-2) + 498y(n-1)-251y(n-2))/256 (2k sampling)
// y(n) = (256x(n) -476x(n-1) + 256x(n-2) + 471y(n-1)-251y(n-2))/256 (1k sampling)
long Filter(long data){
static long x[6]; // this MACQ needs twice
static long y[6];
static unsigned long n=3;   // 3, 4, or 5
  n++;
  if(n==6) n=3;     
  x[n] = x[n-3] = data;  // two copies of new data
  y[n] = (256*(x[n]+x[n-2])-503*x[n-1]+498*y[n-1]-251*y[n-2]+128)/256;
  y[n-3] = y[n];         // two copies of filter outputs too
  return y[n];
} 
//******** DAS *************** 
// background thread, calculates 60Hz notch filter
// runs 2000 times/sec
// samples channel 4, PD3,
// inputs:  none
// outputs: none
unsigned long DASoutput;
void DAS(void){ 
unsigned long input;  
	unsigned long myId = OS_Id(); 
  if(NumSamples < RUNLENGTH){   // finite time run
    PB2 ^= 0x04;
    input = ADC_In();           // channel set when calling ADC_Init
    PB2 ^= 0x04;
    DASoutput = Filter(input);
    FilterWork++;        // calculation finished
    PB2 ^= 0x04;
		TIMER0A_doJitterTimer();
  }
	//OS_Kill();	
}
//--------------end of Task 1-----------------------------

//------------------Task 2--------------------------------
// background thread executes with SW1 button
// one foreground task created with button push
// foreground treads run for 2 sec and die
// ***********ButtonWork*************
void ButtonWork(void){
	unsigned long myId = OS_Id(); 
  PB3 ^= 0x08;
  ST7735_Message(1,0,"NumCreated =",numCreated); 
  PB3 ^= 0x08;
  OS_Sleep(50);     // set this to sleep for 50msec
  ST7735_Message(1,1,"PIDWork     =",PIDWork);
  ST7735_Message(1,2,"DataLost    =",DataLost); 
  ST7735_Message(1,3,"Jitter 0.1us=",TIMER0A_getMaxJitter()); 
  PB3 ^= 0x08;
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
//--------------end of Task 2-----------------------------

//------------------Task 3--------------------------------
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
void Producer(unsigned long data){  
  if(NumSamples < RUNLENGTH){   // finite time run
    NumSamples++;               // number of samples
    if(OS_Fifo_Put(data) == -1){ // send to consumer
      DataLost++;
    }
		//if(NumSamples < 4) {
			//UART_OutUDec(data);
			//OutCRLF();
			if(NumSamples % 3 == 1) {
				result[0] = data;
			}
			if(NumSamples % 3 == 2) {
				result[1] = data;
			}
			if(NumSamples % 3 == 0){
				result[2] = data;
				//UART_OutString("Median: ");
				//OutCRLF();
				UART_OutUDec(20000/Median(result[0], result[1], result[2])); //accurate for close distances
				OutCRLF();
			}
		//}
  } 
}
void Display(void); 

//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
void Consumer(void){ 
unsigned long data,DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
unsigned long t;                  // time in 2.5 ms
unsigned long myId = OS_Id(); 
	DataLost = 0; 
  ADC_Collect_Init(5, FS, &Producer); // start ADC sampling, channel 5, PD2, 400 Hz
  numCreated += OS_AddThread(&Display,128,1); 
  while(NumSamples < RUNLENGTH - 32) { 
    PB4 = 0x10;
    for(t = 0; t < 64; t++){   // collect 64 ADC samples
      data = OS_Fifo_Get();    // get from producer - if we get stuck here then display may have been killed
      x[t] = data;             // real part is 0 to 4095, imaginary part is 0
    }
    PB4 = 0x00;
    cr4_fft_64_stm32(y,x,64);  // complex FFT of last 64 ADC values
    DCcomponent = y[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
    OS_MailBox_Send(DCcomponent); // called every 2.5ms*64 = 160ms
		//LEDS = COLORWHEEL[(wheelCounter++) % WHEELSIZE];
  }
	//LEDS = RED;
  OS_Kill();  // never called
}
//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){ 
unsigned long data,voltage;
unsigned long myId2 = OS_Id(); 
  ST7735_Message(0,1,"Run length = ",(RUNLENGTH)/FS);   // top half used for Display
  while(NumSamples < RUNLENGTH - 32) { 
    data = OS_MailBox_Recv();
    voltage = 3000*data/4095;               // calibrate your device so voltage is in mV
    PB5 = 0x20;
    ST7735_Message(0,2,"v(mV) =",voltage);  
    PB5 = 0x00;
		//LEDS = COLORWHEEL[(wheelCounter++) % WHEELSIZE];
  }
	//LEDS = RED;
  OS_Kill();  // never called
} 

//--------------end of Task 3-----------------------------

//------------------Task 4--------------------------------
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

extern unsigned int pulsePeriod;
extern int pingPB2_3_ready;
void PingTest(void) {
	UART_OutString("Ping Test Started");
	OutCRLF();
	Timer3_Init();
	StartPingSensorPB2_3();
	while(1) {
		if(pingPB2_3_ready == 1) {
			pingPB2_3_ready = 0;
			UART_OutUDec(pulsePeriod); //accurate for close distances
			OutCRLF();
			StartPingSensorPB2_3();
		}
	}
}

void postLauntInits() {
	UART_Init();
	UART_OutString("Hello Lab 6.0.1");
	OutCRLF();
	OS_Kill();
}

void PB3High() {
	PB3 ^= 0x08;
	OS_Kill();
}

//--------------end of Task 4-----------------------------

//*******************final user main DEMONTRATE THIS TO TA**********
int main(void){
  OS_Init();           // initialize, disable interrupts
  //PortB_Init();
	
	ST7735_InitRDivided(INITR_REDTAB);
 
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
	FilterWork = 0;
  numCreated = 0;
//********initialize communication channels
  OS_MailBox_Init();		
  OS_Fifo_Init(32);    // ***note*** 4 is not big enough*****

//*******attach background tasks***********
  OS_AddSW1Task(&PB3High,2); //not stuck
  //OS_AddSW2Task(&SW2Push,2);  // add this line in Lab 3

  //ADC_Init(4);  // sequencer 3, channel 4, PD3, sampling in DAS()
  //OS_AddPeriodicThread(&DAS,PERIOD,0); // 2 kHz real time sampling of PD3 -> input = ADC_In(); 
	
	//create initial foreground threads
	
	numCreated += OS_AddThread(&postLauntInits,128,1);
	//numCreated += OS_AddThread(&Interpreter,128,5);
	//numCreated += OS_AddThread(&Consumer,128,2); 
	numCreated += OS_AddThread(&PingTest,128,2);  // Lab 3, make this lowest priority
  numCreated += OS_AddThread(&PID,128,6);  // Lab 3, make this lowest priority
	
	/*
	NumCreated += OS_AddThread(&PID,128,5); //not suck 
	NumCreated += OS_AddThread(&PID,128,5); //not suck 
	NumCreated += OS_AddThread(&PID,128,5); //not suck 
	NumCreated += OS_AddThread(&PID,128,5); //not suck 
	NumCreated += OS_AddThread(&PID,128,5); //not suck 
	//NumCreated += OS_AddThread(&PID,128,5); //not suck 
	//NumCreated += OS_AddThread(&PID,128,5); //not suck 
	*/
	//NumCreated += OS_AddThread(&PID,128,2); //not suck 

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
