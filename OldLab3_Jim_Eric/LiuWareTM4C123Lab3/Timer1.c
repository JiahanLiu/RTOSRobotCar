// Timer1.c
// Runs on LM4F120/TM4C123
// Use TIMER1 in 32-bit periodic mode to request interrupts at a periodic rate
// Daniel Valvano
// May 5, 2015

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
  Program 7.5, example 7.6

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"
#include "../LiuWareTM4C123Lab3/OS.h"
#include "../LiuWareTM4C123Lab3/Timer1.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask)(void);   // user function

uint32_t MaxJitterT1;             // largest time jitter between interrupts in usec
#define JITTERSIZET1 64
unsigned long const JitterSizeT1=JITTERSIZET1;
unsigned long JitterHistogramT1[JITTERSIZET1]={0,};
int FilterWorkT1; 
uint32_t PERIODT1; 

// ***************** TIMER1_Init ****************
// Activate TIMER1 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void Timer1_Init(void(*task)(void), uint32_t period, uint32_t priority){
	long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x02;   // 0) activate TIMER1
  PeriodicTask = task;          // user function
  TIMER1_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER1_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER1_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER1_TAILR_R = period-1;    // 4) reload value
  TIMER1_TAPR_R = 0;            // 5) bus clock resolution
  TIMER1_ICR_R = 0x00000001;    // 6) clear TIMER1A timeout flag
  TIMER1_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI5_R = (NVIC_PRI5_R&0xFFFF00FF)|(priority << 13); // 8) priority
// interrupts enabled in the main program after all devices initialized
// vector number 37, interrupt number 21
		// Jitter
	FilterWorkT1 = 0;
	PERIODT1 = period;
	MaxJitterT1 = 0;
	// -End Jitter
  NVIC_EN0_R = 1<<21;           // 9) enable IRQ 21 in NVIC
  TIMER1_CTL_R = 0x00000001 + TIMER_CTL_TASTALL + TIMER_CTL_TBSTALL;    // 10) enable TIMER1A
	TIMER1_CTL_R &= ~TIMER_CTL_RTCEN;//disable RTCEN
	EndCritical(sr);
}

void Timer1A_Handler(void){
  TIMER1_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER1A timeout
  (*PeriodicTask)();                // execute user task
}
void getJitterTimer1(void){  
	unsigned static long LastTime;  // time at previous ADC sample
	unsigned long thisTime;         // time at current ADC sample
	long jitter;                    // time between measured and expected, in us
	thisTime = OS_Time();       // current time, 12.5 ns
	FilterWorkT1++;        // calculation finished
	if(FilterWorkT1>1){    // ignore timing of first interrupt
		unsigned long diff = OS_TimeDifference(LastTime,thisTime);
		
		if(diff>PERIODT1){
			jitter = (diff-PERIODT1+4)/8;  // in 0.1 usec
		}else{
			jitter = (PERIODT1-diff+4)/8;  // in 0.1 usec
		}
		if(jitter > MaxJitterT1){
			MaxJitterT1 = jitter; // in usec
		}       // jitter should be 0
		if(jitter >= JitterSizeT1){ // for histogram
			jitter = JITTERSIZET1-1;
		}
		JitterHistogramT1[jitter]++; 
	}
	LastTime = thisTime;
}
uint32_t getMaxJitterT1(void) {
	return MaxJitterT1;
}
