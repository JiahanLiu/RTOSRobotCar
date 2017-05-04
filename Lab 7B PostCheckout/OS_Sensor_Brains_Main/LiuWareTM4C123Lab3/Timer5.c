// Timer0A.c
// Use Timer5 in periodic mode to request interrupts at a particular
// period.
// Last modified 2/10/17 Team Cheetah
// TA: Daniel Leach

#include <stdint.h>
#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"
#include "../LiuWareTM4C123Lab3/Timer5.h" //used for sleep decrement + OS Time

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask)(void);   // user function

// ***************** Timer5A_Init ****************
// Activate TIMER5 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq), 32 bits
// Outputs: none
void Timer5_Init(void(*task)(void), uint32_t period, uint32_t priority){
  long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x20;   // 0) activate TIMER5
	PeriodicTask = task;          // user function
  TIMER5_CTL_R = 0x00000000;    // 1) disable TIMER5A during setup
  TIMER5_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER5_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER5_TAILR_R = period-1;    // 4) reload value
  TIMER5_TAPR_R = 0;            // 5) bus clock resolution
  TIMER5_ICR_R = 0x00000001;    // 6) clear TIMER4A timeout flag
  TIMER5_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI23_R = (NVIC_PRI23_R&0xFFFFFF00)|0x00000020; // 8) priority 1
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
  NVIC_EN2_R = 1<<28;           // 9) enable IRQ 92 in NVIC
  TIMER5_CTL_R = 0x00000001;    // 10) enable TIMER4
  EndCritical(sr);
}

void Timer5A_Handler(void) {
    TIMER5_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer5A timeout
	  (*PeriodicTask)();                // execute user task
}
