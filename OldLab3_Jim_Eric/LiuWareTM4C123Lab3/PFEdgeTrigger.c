// EdgeInterrupt.c
// Runs on LM4F120 or TM4C123
// Request an interrupt on the falling edge of PF4 (when the user
// button is pressed) and increment a counter in the interrupt.  Note
// that button bouncing is not addressed.
// Daniel Valvano
// May 3, 2015

/* This example accompanies the book
   "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
   ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2015
   Volume 1, Program 9.4
   
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Volume 2, Program 5.6, Section 5.5

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

// user button connected to PF4 (increment counter on falling edge)

#include <stdint.h>
#include "../LiuWareTM4C123Lab3/PFEdgeTrigger.h"
#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"
#include "../LiuWareTM4C123Lab3/OS.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PF0Task)(void);   // user function
void (*PF4Task)(void);   // user function

//---------------- Debug Lights -------------------
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define WHEELSIZE 8           // must be an integer multiple of 2	
//const long COLORWHEEL[WHEELSIZE] = {RED, RED+GREEN, GREEN, GREEN+BLUE, BLUE, BLUE+RED, RED+GREEN+BLUE, 0};
extern const long COLORWHEEL[WHEELSIZE];

extern int NumCreated;

void NothingTask() {
}

Sema4Type mutexPFInit;
int bPFNeedInit = 1; 
int bNothingAssignedToPF0 = 1; //if nothing is assigned we don't want add thread to be running - bug
int bNothingAssignedToPF4 = 1;
int PF0Priority;
int PF4Priority; 

void PFEdge_Init(uint32_t priority){  
	if (bPFNeedInit == 1) {
		bPFNeedInit = 0;
		PF0Task = NothingTask;
		PF4Task = NothingTask;
		//Original Code
		long sr = StartCritical(); // We may want to Add PTCL to 0
		SYSCTL_RCGCGPIO_R |= 0x00000020; // (a) activate clock for port F
		GPIO_PORTF_LOCK_R = 0x4C4F434B;
		GPIO_PORTF_CR_R = 0x11;
		GPIO_PORTF_DIR_R &= ~0x11;
		GPIO_PORTF_DEN_R |= 0x11;
		GPIO_PORTF_PUR_R |= 0x11;
		GPIO_PORTF_IS_R &= 0x11;
		GPIO_PORTF_IBE_R &= 0x11;
		GPIO_PORTF_IEV_R &= 0x11;
		GPIO_PORTF_ICR_R = 0x11;
		GPIO_PORTF_IM_R |= 0x11;
		NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF) | (priority << 21);
		NVIC_EN0_R = 0x40000000;
		EndCritical(sr);
	}
}

void PF0Edge_Task(void(*task)(void), uint32_t priority) {
	PF0Task = task;
	PF0Priority = priority;
	bNothingAssignedToPF0 = 0;
}

void PF4Edge_Task(void(*task)(void), uint32_t priority) {
	PF4Task = task;
	PF4Priority = priority; 
	bNothingAssignedToPF4 = 0;
}

void GPIOPortF_Handler(void){
	if(GPIO_PORTF_RIS_R&0x10) {
		GPIO_PORTF_ICR_R |= 0x10;      // acknowledge flag4
		if(OS_MsTime() > 250 && !bNothingAssignedToPF4){ // debounce
			if(OS_AddThread(PF4Task,100,PF4Priority)){
				NumCreated++; 
			} else {
				LEDS = RED;
			}
    OS_ClearMsTime();  // at least 20ms between touches
		}
	}
	
	if(GPIO_PORTF_RIS_R&0x01) {
		GPIO_PORTF_ICR_R |= 0x01;      // acknowledge flag0		
		if(OS_MsTime() > 250 && !bNothingAssignedToPF0){ // debounce
			if(OS_AddThread(PF0Task,100,PF0Priority)){
				NumCreated++; 
			} else {
				LEDS = RED;
			}
    OS_ClearMsTime();  // at least 20ms between touches
		}
	}
}
