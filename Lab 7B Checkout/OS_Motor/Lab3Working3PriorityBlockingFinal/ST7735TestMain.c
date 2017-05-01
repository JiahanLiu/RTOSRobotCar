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
	Timer 6: Periodic Thread for Pulse
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
#include "../LiuWareTM4C123Lab3/PWM.h"
#include "../LiuWareTM4C123Lab3/CAN/Timer4.h" //used for sleep decrement + OS Time

#define PF0       (*((volatile uint32_t *)0x40025004))
#define PF1       (*((volatile uint32_t *)0x40025008))
#define PF2       (*((volatile uint32_t *)0x40025010))
#define PF3       (*((volatile uint32_t *)0x40025020))
#define PF4       (*((volatile uint32_t *)0x40025040))
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
	
#define PERIOD1MS 80000

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void PortB_Init(void); 

#define FS 400              // producer/consumer sampling
#define RUNLENGTH (20 * FS )   // display results and quit when NumSamples==RUNLENGTH

//data variables
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
unsigned long DataLost = 0;     // data sent by Producer, but not received by Consumer

//debug variables
unsigned long numCreated = 0;   // number of foreground threads created
int numThreads = 0;

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
	}
}

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
    }
    PIDWork++;        // calculation finished
  }
  for(;;){
	}          // done
}

uint8_t XmtData[8];
uint8_t RcvData[8];
uint32_t RcvCount=0;
uint8_t sequenceNum=0;  
void UserTask(void){
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

#define SERVOMAX 2375
#define SERVOMID 1875
#define SERVOMIN 1275
#define SERVODELTA 100
// SW2 cycles through 12 positions
// mode=0 1875,...,1275
// mode=1 1870,...,2375
uint32_t Steering;     // 625 to 3125
uint32_t SteeringMode; // 0 for increase, 1 for decrease
#define POWERMIN 400
#define POWERMAX 12400
#define POWERDELTA 2000
uint32_t Power;
void DelayWait10msMOTOR(uint32_t n);

void PortF_Init(void){
  SYSCTL_RCGCGPIO_R |= 0x20;       // activate port F
  while((SYSCTL_PRGPIO_R&0x0020) == 0){};// ready?
  GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_CR_R = 0x01;      // enable commit for PF0

  GPIO_PORTF_DIR_R &= ~0x11;       // make PF4 input
  GPIO_PORTF_DIR_R |= 0x0E;        // make PF3-1 output (PF3-1 built-in LEDs)
  GPIO_PORTF_AFSEL_R &= ~0x1F;     // disable alt funct on PF3-1
  GPIO_PORTF_DEN_R |= 0x1F;        // enable digital I/O on PF3-1
                                   // configure PF3-1 as GPIO
  GPIO_PORTF_PUR_R |=0x11;          // pullup
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFF00000)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;          // disable analog functionality on PF
  LEDS = 0;                        // turn all LEDs off
}

void WaitForTouch(void){
  while((PF0==0x01)&&(PF4==0x10)){};  // wait for switch
  DelayWait10msMOTOR(2); // debounce
  while((PF0!=0x01)||(PF4!=0x10)){}; // wait for both release
  DelayWait10msMOTOR(2); // debounce
}

void MotorTest() {
	Power = 5;
  Steering = SERVOMID;  // 20ms period 1.5ms pulse
  SteeringMode = 0;
  Left_Init(12500, Power,0);          // initialize PWM0, 100 Hz
  Right_Init(12500, 12500-Power,1);   // initialize PWM0, 100 Hz
  Servo_Init(25000, Steering);   
	Power = POWERMIN;     // PWMclock at 1.25MHz
  while(1){
    while((PF0==0x01)&&(PF4==0x10)){};
    if(PF0==0){
      if(SteeringMode){
        if(Steering >= SERVOMIN+SERVODELTA){
          Steering = Steering - SERVODELTA;
        }else{
          Steering = SERVOMID; // go to center and
          SteeringMode = 0;    // switch direction
        }
      }else{
        Steering = Steering + SERVODELTA;
        if(Steering > SERVOMAX){
          Steering = SERVOMID; // go to center and
          SteeringMode = 1;    // switch direction
        }
      }
      Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
      PF1 ^= 0x02;
    }
    if((PF0==0x01)&&(PF4==0)){
      Power = Power + POWERDELTA;
      if(Power > POWERMAX){
        Power = POWERMIN;      // go back to minimum
      }
      PF2 ^= 0x04;
      Left_Duty(Power,0);       // 400 to 12400 (positive logic)
    //  Right_Duty(Power,0);       // 400 to 12400 (positive logic)
      Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
    }
    WaitForTouch();
  }
}

void setPowerForwardSlowest() {
	Power = POWERMIN;     // PWMclock at 1.25MHz
	Power = Power + POWERDELTA;
	Left_Duty(Power,0);       // 400 to 12400 (positive logic)
	Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
}

void setPowerBackwardSlowest() {
	Power = POWERMAX;     // PWMclock at 1.25MHz
	Power = Power + POWERDELTA;
	Left_DutyB(Power,0);       // 400 to 12400 (positive logic)
	Right_DutyB(12500-Power,0);  // 12400 to 400 (negative logic)
}

void stop() {
	Power = POWERMIN;     // PWMclock at 1.25MHz
	Left_Duty(Power,0);       // 400 to 12400 (positive logic)
//  Right_Duty(Power,0);       // 400 to 12400 (positive logic)
	Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
}

void straight() {
	Steering = SERVOMID;
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
	setPowerForwardSlowest();
}

void slightLeft() {
	Steering = SERVOMID + SERVODELTA/2;
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
	setPowerForwardSlowest();
}

void slightRight() {
	Steering = SERVOMID - SERVODELTA/2;
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
	setPowerForwardSlowest();

}

void HardLeft() {
	Steering = SERVOMAX - SERVODELTA;
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
	setPowerForwardSlowest();		
	
		Power = POWERMIN;
	Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
		Power = POWERMIN + 2 * POWERDELTA;
	Left_Duty(Power,0);       // 400 to 12400 (positive logic)
}

void HardRight() {
	Steering = SERVOMIN;
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
	setPowerForwardSlowest();
	Power = POWERMIN;
	Left_Duty(Power,0);       // 400 to 12400 (positive logic)
			Power = POWERMIN + POWERDELTA * 2;
	Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
}

void BackLeft() {
	Steering = SERVOMID;
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
	setPowerBackwardSlowest();
}

void BackRight() {
	Steering = SERVOMID - SERVODELTA;
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
	setPowerBackwardSlowest();
}

void MotorTestLab7B() {
	Power = 5;
  Steering = SERVOMID;  // 20ms period 1.5ms pulse
  SteeringMode = 0;
  Left_Init(12500, Power,0);          // initialize PWM0, 100 Hz
  Right_Init(12500, 12500-Power,1);   // initialize PWM0, 100 Hz
  Servo_Init(25000, Steering);   
	Power = POWERMIN;     // PWMclock at 1.25MHz
	Power = Power + POWERDELTA;
	Left_Duty(Power,0);       // 400 to 12400 (positive logic)
//  Right_Duty(Power,0);       // 400 to 12400 (positive logic)
	Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
	while(1){
	  if(CAN0_GetMailNonBlock(RcvData)){
			switch(RcvData[0]) {
				case 0: //stop
					stop();
					break;
				case 1:
					straight();
					break;
				case 2:
					slightLeft();
					break;
				case 3:
					slightRight();
					break;
				case 4:
					HardLeft();
					break;
				case 5:
					HardRight();
					break;
				case 6: 
					BackLeft();
					break;
				case 7:
					BackRight();				
			}
		}
	}
	LEDS = RED;
	while(1) {}
}

void MotorTestLab7ALeftTurn() {
	uint32_t LeftPower = 5;
  uint32_t RightPower = 5;
	Steering = SERVOMID;  // 20ms period 1.5ms pulse
  SteeringMode = 0;
  Left_Init(12500, Power,0);          // initialize PWM0, 100 Hz
  Right_Init(12500, 12500-Power,1);   // initialize PWM0, 100 Hz
  Servo_Init(25000, Steering);   
	Power = POWERMIN;     // PWMclock at 1.25MHz
	Power = Power + POWERDELTA;
	Left_Duty(Power,0);       // 400 to 12400 (positive logic)
//  Right_Duty(Power,0);       // 400 to 12400 (positive logic)
	Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
	
	//start 

	Steering = SERVOMAX; //max is left
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX	
	while(1){};
	while(1){
	  if(CAN0_GetMailNonBlock(RcvData)){
			if(RcvData[0] == 'L') {
				Steering = SERVOMIN;
				Servo_Duty(Steering);    // SERVOMIN to SERVOMAX
			}
			if(RcvData[0] == 'R') {
				Power= POWERMIN;
				Left_Duty(Power,0);       // 400 to 12400 (positive logic)
				Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
			}
			if(RcvData[0] == 'S') {
				Power= POWERMIN;
				Left_Duty(Power,0);       // 400 to 12400 (positive logic)
				Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
			}							
    }
	}
	LEDS = RED;
	while(1) {}
}


void MotorTestLab7AFinal() {
	uint32_t LeftPower = 5;
  uint32_t RightPower = 5;
	Steering = SERVOMID;  // 20ms period 1.5ms pulse
  SteeringMode = 0;
  Left_Init(12500, Power,0);          // initialize PWM0, 100 Hz
  Right_Init(12500, 12500-Power,1);   // initialize PWM0, 100 Hz
  Servo_Init(25000, Steering);   
	Power = POWERMIN;     // PWMclock at 1.25MHz
	Power = Power + POWERDELTA;
	Left_Duty(Power,0);       // 400 to 12400 (positive logic)
//  Right_Duty(Power,0);       // 400 to 12400 (positive logic)
	Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
	Steering = SERVOMID; //max is left
	Servo_Duty(Steering);    // SERVOMIN to SERVOMAX	
	while(1){
	  if(CAN0_GetMailNonBlock(RcvData)){
			if(RcvData[0] == 'L') {
				Steering = SERVOMAX; //max is left
				Servo_Duty(Steering);    // SERVOMIN to SERVOMAX	
			}
			if(RcvData[0] == 'R') {
				Power= POWERMIN;
				Left_Duty(Power,0);       // 400 to 12400 (positive logic)
				Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
			}
			if(RcvData[0] == 'S') {
				Power= POWERMIN;
				Left_Duty(Power,0);       // 400 to 12400 (positive logic)
				Right_Duty(12500-Power,1);  // 12400 to 400 (negative logic)
			}							
    }
	}
	LEDS = RED;
	while(1) {}
}


void postLauntInits(){
	long sr = StartCritical();
	CAN0_Open();
	//Timer4_Init(&UserTask, 1600000); // initialize timer3 (10 Hz) // Sleep
	EndCritical(sr);
	PortF_Init();
	UART_Init();
	UART_OutString("Hello Lab 6.0.1");
	OutCRLF();
	OS_Kill();
}

//--------------end of Task 4-----------------------------

//*******************final user main DEMONTRATE THIS TO TA**********
int main(void){
  OS_Init();           // initialize, disable interrupts
	//Timer0A_Init();
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

  //ADC_Init(4);  // sequencer 3, channel 4, PD3, sampling in DAS()
  //OS_AddPeriodicThread(&DAS,PERIOD,0); // 2 kHz real time sampling of PD3 -> input = ADC_In(); 

	//OS_AddPeriodicThread(&PingTest,PERIOD1MS*500,0);
	//create initial foreground threads
	
	numCreated += OS_AddThread(&postLauntInits,128,1);
	//numCreated += OS_AddThread(&Interpreter,128,5);
	//numCreated += OS_AddThread(&CANTest,128,2);
	//numCreated += OS_AddThread(&Consumer,128,2); 
	numCreated += OS_AddThread(&MotorTestLab7B,128,2);  // Lab 3, make this lowest priority
	//numCreated += OS_AddThread(&MotorTestLab7AFinal,128,2);  // Lab 3, make this lowest priority
  numCreated += OS_AddThread(&PID,128,6);  // Lab 3, make this lowest priority

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}

// Subroutine to wait 10 msec
// Inputs: None
// Outputs: None
// Notes: ...
void DelayWait10msMOTOR(uint32_t n){uint32_t volatile time;
  while(n){
    time = 727240*2/91;  // 10msec
    while(time){
      time--;
    }
    n--;
  }
}
