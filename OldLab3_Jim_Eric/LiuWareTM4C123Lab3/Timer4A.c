// Timer0A.c
// Use Timer0A in periodic mode to request interrupts at a particular
// period.
// Last modified 2/10/17 Jiahan Liu, Eric Li
// Jiahan Liu, jl57566
// Eric Li, ecl625
// TA: Daniel Leach

// Used For

#include <stdint.h>
#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"

#define PF2   (*((volatile uint32_t *)0x40025010))
#define PF3   (*((volatile uint32_t *)0x40025020))

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void (*PeriodicTask4)(void);   // user function

// ***************** Timer0A_Init ****************
// Activate TIMER0 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq), 32 bits
// Outputs: none
void Timer4A_Init(void(*task)(void), uint32_t period, uint32_t priority){
	long sr;
  sr = StartCritical(); 
  SYSCTL_RCGCTIMER_R |= 0x16;   // 0) activate TIMER4
  PeriodicTask4 = task;          // user function
  TIMER4_CTL_R = 0x00000000;    // 1) disable TIMER1A during setup
  TIMER4_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER4_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER4_TAILR_R = period-1;    // 4) reload value
  TIMER4_TAPR_R = 0;            // 5) bus clock resolution
  TIMER4_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER4_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI17_R = (NVIC_PRI17_R&0xFF00FFFF)|(priority << 21); // 8) priority
// interrupts enabled in the main program after all devices initialized
// vector number 35, interrupt number 19
  NVIC_EN2_R = 1<<(70 - 32 * 2);           // 9) enable IRQ 70 in NVIC
  TIMER4_CTL_R = 0x00000001;    // 10) enable TIMER0A
  EndCritical(sr);
}
void Timer4A_Handler(void){
  TIMER4_ICR_R = TIMER_ICR_TATOCINT;// acknowledge timer0A timeout
  (*PeriodicTask4)();                // execute user task
}
