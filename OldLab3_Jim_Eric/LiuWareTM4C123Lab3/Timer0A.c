// Timer0A.c
// Runs on LM4F120/TM4C123
// Use Timer0A in periodic mode to request interrupts at a particular
// period.
// Daniel Valvano
// September 11, 2013

/* 
Used for add periodic thread
 */
#include <stdint.h>
#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"
#include "../LiuWareTM4C123Lab3/OS.h"  
#include "../LiuWareTM4C123Lab3/Timer0A.h"

//---------------- Debug Lights -------------------
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define WHEELSIZE 8           // must be an integer multiple of 2	
//const long COLORWHEEL[WHEELSIZE] = {RED, RED+GREEN, GREEN, GREEN+BLUE, BLUE, BLUE+RED, RED+GREEN+BLUE, 0};
extern const long COLORWHEEL[WHEELSIZE];
int triggered = 0;

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
/********* PeriodicTask0 *******
* call back function to be performed at periodic time
*******************************/
void (*PeriodicTask0)(void);   // user function

/* Jitter Counter Variables */
uint32_t MaxJitterT0;             // largest time jitter between interrupts in usec
#define JITTERSIZET0 64
unsigned long const JitterSizeT0=JITTERSIZET0;
unsigned long JitterHistogramT0[JITTERSIZET0]={0,};
int FilterWorkT0; 
/* Timer Variables */
uint32_t PERIODT0; 
uint32_t TIMER0A_ThreadPriority; 

// ***************** Timer0A_Init ****************
// Activate TIMER0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq), 32 bits
// Outputs: none
void Timer0A_Init(void(*task)(void), uint32_t period, uint32_t priority){
	long sr;
  sr = StartCritical(); 
	TIMER0A_ThreadPriority = priority;
	// Original
  SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate TIMER0
  PeriodicTask0 = task;          // user function
  TIMER0_CTL_R = 0x00000000;    // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period-1;    // 4) reload value
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|(priority << 29); // 8) priority
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
	// -Jitter
	FilterWorkT0 = 0;
	PERIODT0 = period;
	MaxJitterT0 = 0;
	// -End Jitter
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable TIMER0A
  EndCritical(sr);
}
void Timer0A_Handler(void){
  TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer0A timeout
	
	/*
	if(triggered != 1) {
		//LEDS = 0;
	}	
	if(OS_AddThread(PeriodicTask0,100,TIMER0A_ThreadPriority)){
				//NumCreated++; 
	} else {
		//LEDS = RED;
		triggered = 1; 
	} */
  (*PeriodicTask0)();                // execute user task
}

/********* doJitterTimer0 ********
* Public Jitter fucntion that needs to be placed in callback function
* to be used. This increases freedom for user to use or not
**********************************/
void TIMER0A_doJitterTimer(void){  
	unsigned static long LastTime;  // time at previous ADC sample
	unsigned long thisTime;         // time at current ADC sample
	long jitter;                    // time between measured and expected, in us
	thisTime = OS_Time();       // current time, 12.5 ns
	FilterWorkT0++;        // calculation finished
	if(FilterWorkT0>1){    // ignore timing of first interrupt
		unsigned long diff = OS_TimeDifference(LastTime,thisTime);
		if(diff>PERIODT0){
			jitter = (diff-PERIODT0+4)/8;  // in 0.1 usec
		}else{
			jitter = (PERIODT0-diff+4)/8;  // in 0.1 usec
		}
		if(jitter > MaxJitterT0){
			MaxJitterT0 = jitter; // in usec
		}       // jitter should be 0
		if(jitter >= JitterSizeT0){ // for histogram
			jitter = JITTERSIZET0-1;
		}
		JitterHistogramT0[jitter]++; 
	}
	LastTime = thisTime;
}

/********* getJitterTimer0 ********
* Getter for doMaxJitterT0 if used.
**********************************/
uint32_t TIMER0A_getMaxJitter(void) {
	return MaxJitterT0;
}
