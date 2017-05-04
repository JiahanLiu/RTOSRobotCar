// Timer3.c
// Runs on LM4F120/TM4C123
// Use Timer3 in 32-bit periodic mode to request interrupts at a periodic rate
// Daniel Valvano
// March 20, 2014

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2013
  Program 7.5, example 7.6

 Copyright 2013 by Jonathan W. Valvano, valvano@mail.utexas.edu
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
#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"
#include "../LiuWareTM4C123Lab3/OS.h"
#include "../LiuWareTM4C123Lab3/Timer3.h"
#include "../LiuWareTM4C123Lab3/LEDS.h"

#define PB3  (*((volatile unsigned long *)0x40005020))

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

// ***************** Timer3_Init ****************
// Activate Timer3 interrupts to run user task periodically
// Inputs:  task is a pointer to a user function
//          period in units (1/clockfreq)
// Outputs: none
void PortBActivation() {
	SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
	while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?	
                                   // configure PB2 as T0CCP0
	GPIO_PORTB_AMSEL_R &= ~0x4;     // disable analog functionality on PB2
	GPIO_PORTB_AMSEL_R &= ~0x8;     // disable analog functionality on PB3
}
void Timer3_Init(){
	long sr;
  sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x08;   // 0) activate TIMER3
	SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
	while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
	GPIO_PORTB_DIR_R &= ~0x4;       // make PB2 in	
	GPIO_PORTB_DIR_R |= 0x8;       // make PB3 out
	GPIO_PORTB_AFSEL_R |= 0x4;      // enable alt funct on PB2
  GPIO_PORTB_AFSEL_R &= ~0x8;   // disable alt funct on PB3
	GPIO_PORTB_DEN_R |= 0x4;        // enable digital I/O on PB2
	GPIO_PORTB_DEN_R |= 0x8;        // enable digital I/O on PB3	
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFF0FF)+0x00000700;  // configure PB2 as T0CCP0
	GPIO_PORTB_PCTL_R &= ~0x0000F000; //PB3 Port
  GPIO_PORTB_AMSEL_R &= ~0x4;     // disable analog functionality on PB2
	GPIO_PORTB_AMSEL_R &= ~0x8;     // disable analog functionality on PB3
		
	TIMER3_CTL_R = 0x00000000;    // 1) disable TIMER3A during setup
  TIMER3_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER3_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);   // 3) configure for edge capture mode
  TIMER3_CTL_R &= ~(TIMER_CTL_TAEVENT_BOTH|0x0C); //both edges
	TIMER3_TAILR_R = TIMER_TAILR_M;    // 4) reload value (max 32bits)
	TIMER3_IMR_R |= TIMER_IMR_CAEIM;    // 7) arm timeout interrupt
  //TIMER3_TAPR_R = 0;            // 5) bus clock resolution
  TIMER3_ICR_R = TIMER_ICR_CAECINT;// clear timer0A capture match flag
	
  NVIC_PRI8_R = (NVIC_PRI8_R&0x00FFFFFF)|0x80000000; // 8) priority 4
// interrupts enabled in the main program after all devices initialized
// vector number 51, interrupt number 35
  NVIC_EN1_R = 1<<(35-32);      // 9) enable IRQ 35 in NVIC
	
  TIMER3_CTL_R &= ~TIMER_CTL_RTCEN;//disable RTCEN
	TIMER3_CTL_R |= TIMER_CTL_TAEN + TIMER_CTL_TASTALL + TIMER_CTL_TBSTALL;// enable timer0A 16-b, +edge timing, interrupts
	
	EndCritical(sr);
}

unsigned int lastTime = 0;
unsigned int nowTime = 0;
unsigned int pulsePeriod = 0;
int pingPB2_3_ready = 0;

void Timer3A_Handler(void){
  long sr;
  sr = StartCritical();
	TIMER3_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER3A timeout
	if((GPIO_PORTB_DATA_R & 0x4) == 0x4)//rising edge
	{
		pulsePeriod = 0;
		TIMER3_CTL_R = 0x0;    // 1) disable TIMER3A during setup
	  TIMER3_TAILR_R = TIMER_TAILR_M;    // 4) reload value
	  TIMER3_CTL_R |= TIMER_CTL_TAEN;    // 10) enable TIMER3A
	}else{ //falling edge
		pulsePeriod = TIMER_TAILR_M - TIMER3_TAR_R;
		pingPB2_3_ready = 1;
	}	
	EndCritical(sr);
}

void StartPingSensorPB2_3() {
	long sr;
  sr = StartCritical();
	PB3 = 0x8; //create pulse
	DelayWait10us(1);
	PB3 = 0;
	EndCritical(sr);
}
