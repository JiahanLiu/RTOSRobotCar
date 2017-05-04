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
#include "../LiuWareTM4C123Lab3/LEDS.h"

#define PB7  (*((volatile unsigned long *)0x40005200))
#define NVIC_EN0_INT19          0x00080000  // Interrupt 19 enable

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

void Timer0A_Init(){
	long sr = StartCritical();
  SYSCTL_RCGCTIMER_R |= 0x01;// activate timer0
  SYSCTL_RCGCGPIO_R |= 0x02; // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};// ready?
  GPIO_PORTB_DIR_R &= ~0x40;       // make PB6 in
	GPIO_PORTB_DIR_R |= 0x80;    // make PB7 output heartbeats
  GPIO_PORTB_AFSEL_R |= 0x40;      // enable alt funct on PB6
	GPIO_PORTB_AFSEL_R &= ~0x80;   // disable alt funct on PB5-2
  GPIO_PORTB_DEN_R |= 0x40;        // enable digital I/O on PB6
	GPIO_PORTB_DEN_R |= 0x80;     // enable digital I/O on PB5-2
                                   // configure PB6 as T0CCP0
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)+0x07000000;
	GPIO_PORTB_PCTL_R &= ~0xF0000000;
  GPIO_PORTB_AMSEL_R &= ~0x40;     // disable analog functionality on PB6
	GPIO_PORTB_AMSEL_R &= ~0x80;      // disable analog functionality on PB
	GPIO_PORTB_PDR_R |= 0x80;
  TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
  TIMER0_CFG_R = TIMER_CFG_16_BIT; // configure for 16-bit timer mode
                                   // configure for capture mode, default down-count settings
  TIMER0_TAMR_R = (TIMER_TAMR_TACMR|TIMER_TAMR_TAMR_CAP);
                                   // configure for rising edge event
  //TIMER0_CTL_R &= ~(TIMER_CTL_TAEVENT_POS|0xC);
	TIMER0_CTL_R |= TIMER_CTL_TAEVENT_BOTH;
	TIMER0_TAPR_R = 0xFF;		//max 8 bit prescale 
  TIMER0_TAILR_R = 0x00FFFFFF;  // max start value
  TIMER0_IMR_R |= TIMER_IMR_CAEIM; // enable capture match interrupt
  TIMER0_ICR_R = TIMER_ICR_CAECINT;// clear timer0A capture match flag
  TIMER0_CTL_R |= TIMER_CTL_TAEN + TIMER_CTL_TASTALL + TIMER_CTL_TBSTALL;  // enable timer0A 16-b, +edge timing, interrupts
                                   // Timer0A=priority 2
	TIMER0_CTL_R &= ~TIMER_CTL_RTCEN;//disable RTCEN
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x40000000; // top 3 bits
  NVIC_EN0_R = NVIC_EN0_INT19;     // enable interrupt 19 in NVIC
	PB7 = 0;
  EndCritical(sr);
}

unsigned int lastTimePing6_7 = 0;
unsigned int nowTimePing6_7 = 0;
unsigned int pulsePeriodPing6_7 = 0;
int pingPB6_7_ready = 0;
uint32_t static First = 0;

void Timer0A_Handler(void){
	long sr;
  sr = StartCritical();
	TIMER0_ICR_R = TIMER_ICR_CAECINT;// acknowledge timer0A timeout
	if((GPIO_PORTB_DATA_R & 0x40) == 0x40)//rising edge
	{
		  First = TIMER0_TAR_R;
	}else{//falling edge
		pulsePeriodPing6_7 = First - TIMER0_TAR_R;
		pingPB6_7_ready = 1;
		//reset timer
		TIMER0_CTL_R &= ~TIMER_CTL_TAEN; // disable timer0A during setup
		TIMER0_TAILR_R = 0x00FFFFFF;  // max start value
		TIMER0_CTL_R |= TIMER_CTL_TAEN;
	}
//	pulsePeriodPing6_7 = (First - TIMER0_TAR_R)&0x00FFFFFF;
//	First = TIMER0_TAR_R;
//	pingPB6_7_ready = 1;
	EndCritical(sr);
}

/*
if((GPIO_PORTB_DATA_R & 0x40) == 0x40)//rising edge
	{
		pulsePeriodPing6_7 = 0;
		TIMER0_CTL_R = 0x0;    // 1) disable TIMER3A during setup
	  TIMER0_TAILR_R = TIMER_TAILR_M;    // 4) reload value
	  TIMER0_CTL_R |= TIMER_CTL_TAEN + TIMER_CTL_TASTALL + TIMER_CTL_TBSTALL;;    // 10) enable TIMER3A
	}else { //falling edge
		pulsePeriodPing6_7 = 0x00FFFFFF & (TIMER_TAILR_M - TIMER0_TAR_R);
		pingPB6_7_ready = 1;
	}	
*/

void StartPingSensorPB6_7() {
	long sr;
  sr = StartCritical();
	PB7 = 0x80; //create pulse
	DelayWait10us(1);
	PB7 = 0;
	EndCritical(sr);
}
