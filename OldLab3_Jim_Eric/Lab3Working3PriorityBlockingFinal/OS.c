// OS.c
// Uses time to keep time for our real time OS. 
// Last modified 2/10/17 Jiahan Liu, Eric Li
// Jiahan Liu, jl57566
// Eric Li, ecl625
// TA: Daniel Leach

#include "../inc/tm4c123gh6pm.h"
#include <stdint.h>
#include "OS.h"  
#include "PLL.h"
#include "Timer4A.h"
#include "Timer0A.h"
#include "PF4EdgeTrigger.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode 
void StartOS(void);
void doSleepDecrement(void);

#define TIME_1MS    80000  
#define NUMTHREADS 10
#define STACKSIZE 100
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define WHEELSIZE 8           // must be an integer multiple of 2	

struct tcb{
	int32_t *sp;
	struct tcb *next; 
	struct tcb *prev;
	unsigned long id; //index in tcb[]
	int32_t sleep; //sleep counter
};
typedef struct tcb tcbType; 
tcbType tcbs[NUMTHREADS+1];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS+1][STACKSIZE];
// helper vars for threads
int firstThreadEver = 1; //initally true (1) that we are in the state that the next add would be first thread 
unsigned long idQueue[NUMTHREADS+1];
int idQueueHead = 0;
int idQueueTail = 0;
const long COLORWHEEL[WHEELSIZE] = {RED, RED+GREEN, GREEN, GREEN+BLUE, BLUE, BLUE+RED, RED+GREEN+BLUE, 0};


int idQueuePop() {
	if(idQueueHead == idQueueTail) {
		return -1;
	}
	int oldHead = idQueueHead;
	idQueueHead = (idQueueHead +1) % (NUMTHREADS + 1); 
	return idQueue[oldHead]; 
}	

int idQueuePush(unsigned long input) {
	if( (idQueueTail + 1) % (NUMTHREADS + 1) == idQueueHead) {
		return -1;
	}
	int oldTail = idQueueTail;
	idQueueTail = (idQueueTail + 1) % (NUMTHREADS + 1); 
	idQueue[oldTail] = input; 
	return 1; 
}

void doSleepDecrement() {
	int status = StartCritical(); //critical section because tcb objects are global
	tcbType *oldRunPt = RunPt;
	tcbType *currentRunPt = RunPt;
	do {
		if(0 != currentRunPt->sleep) {
			(currentRunPt->sleep)--; 
		}	
		currentRunPt = currentRunPt->next;
	} while(oldRunPt != currentRunPt); 
	EndCritical(status); 
}

void initHardFault (void) {
  SYSCTL_RCGCGPIO_R |= 0x20;       // activate port F
  while((SYSCTL_PRGPIO_R&0x0020) == 0){};// ready?
  GPIO_PORTF_DIR_R |= 0x0E;        // make PF3-1 output (PF3-1 built-in LEDs)
  GPIO_PORTF_AFSEL_R &= ~0x0E;     // disable alt funct on PF3-1
  GPIO_PORTF_DEN_R |= 0x0E;        // enable digital I/O on PF3-1
                                   // configure PF3-1 as GPIO
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFFF0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;          // disable analog functionality on PF
  LEDS = 0;                        // turn all LEDs off
}

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: serial, ADC, systick, LaunchPad I/O and timers 
// input:  none
// output: none
void OS_Init(void) {
	DisableInterrupts(); 
	PLL_Init(Bus80MHz);  
	NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xC0000000; // priority 6 for SysTick
	NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00E00000; // priority 7 for PendSV
	//lab 2.2
	//initialize idQueue[]
	for(int i = 0; i < NUMTHREADS + 1; i++) {
		idQueuePush(i); 
	}
	//sleep decrement timer
	int tempPriority = 4;
	Timer4A_Init(&doSleepDecrement, TIME_1MS, tempPriority);
	initHardFault(); 
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long pValue) {
	DisableInterrupts();
	(semaPt->Value) = pValue;
	EnableInterrupts();
}

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
void OS_Wait(Sema4Type *semaPt) {
	DisableInterrupts();
	while((semaPt->Value) == 0) { //enable interrupts can occur here so we don't get stuck - <= for blocking
		EnableInterrupts();
		DisableInterrupts();
	}
	(semaPt->Value) = (semaPt->Value) - 1;
	EnableInterrupts();
}

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt) {
	DisableInterrupts();
	(semaPt->Value) = (semaPt->Value) + 1;
	EnableInterrupts(); 
}

// ******** OS_bWait ************
// Lab2 spinlock, set to 0
// Lab3 block if less than zero
// input:  pointer to a binary semaphore
// output: none
void OS_bWait(Sema4Type *semaPt) {
	DisableInterrupts();
	while((semaPt->Value) == 0) { //enable interrupts can occur here so we don't get stuck
		EnableInterrupts();
		DisableInterrupts();
	}
	(semaPt->Value) = 0;
	EnableInterrupts();
}

// ******** OS_bSignal ************
// Lab2 spinlock, set to 1
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a binary semaphore
// output: none
void OS_bSignal(Sema4Type *semaPt) {
	DisableInterrupts();
	(semaPt->Value) = 1;
	EnableInterrupts(); 
}

//******** OS_AddThread *************** 
// add a foregound thread to the scheduler
// Inputs: pointer to a void/void foreground task
//         number of bytes allocated for its stack
//         priority, 0 is highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// stack size must be divisable by 8 (aligned to double word boundary)
// In Lab 2, you can ignore both the stackSize and priority fields
// In Lab 3, you can ignore the stackSize fields
int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority) {
	
	static int iLED = 0;
  LEDS = COLORWHEEL[iLED&(WHEELSIZE-1)];
  iLED = iLED + 1;
	
	int32_t status; 
	status = StartCritical();
	int nextFreeID = idQueuePop();
	if(-1 == nextFreeID) {
		return 0;
	}
	if(0 == firstThreadEver) {
		//old prev tcb[].next
		tcbType* oldNextPt = (RunPt -> next); 
		//set Old Next Node's prev
		oldNextPt->prev = &tcbs[nextFreeID]; 
		//set Current Node's next
		RunPt->next = &tcbs[nextFreeID]; 
		//set up tcb
		SetInitialStack(nextFreeID);
		Stacks[nextFreeID][STACKSIZE-2] = (int32_t)(task); //PC
		tcbs[nextFreeID].next = oldNextPt;///&tcbs[nextFreeID];
		tcbs[nextFreeID].prev = RunPt;
		tcbs[nextFreeID].id = nextFreeID;
		tcbs[nextFreeID].sleep = 0;
	} 
	if(1 == firstThreadEver) {
		//set flag to permanently 0
		firstThreadEver = 0;
		//set up tcb
		SetInitialStack(nextFreeID);
		Stacks[nextFreeID][STACKSIZE-2] = (int32_t)(task); //PC
		tcbs[nextFreeID].next = &tcbs[nextFreeID];
		tcbs[nextFreeID].prev = &tcbs[nextFreeID];
		tcbs[nextFreeID].id = nextFreeID;
		tcbs[nextFreeID].sleep = 0;
		//init run pointer
		RunPt = &tcbs[nextFreeID];
	}
	EndCritical(status);
		return 1;
}

void SetInitialStack(uint32_t i) {
	tcbs[i].sp = &Stacks[i][STACKSIZE-16]; //thread stack pointer
	Stacks[i][STACKSIZE - 1] = 0x01000000; //Thumb bit
	Stacks[i][STACKSIZE - 3] = 0x14141414;   //R14
	Stacks[i][STACKSIZE - 4] = 0x12121212; //R12
	Stacks[i][STACKSIZE - 5] = 0x03030303; //R3
	Stacks[i][STACKSIZE - 6] = 0x02020202; //R2
	Stacks[i][STACKSIZE - 7] = 0x01010101; //R1
	Stacks[i][STACKSIZE - 8] = 0x00000000; //R0
	Stacks[i][STACKSIZE - 9] = 0x11111111; //R11
	Stacks[i][STACKSIZE - 10] = 0x10101010; //R10
	Stacks[i][STACKSIZE - 11] = 0x09090909; //R9
	Stacks[i][STACKSIZE - 12] = 0x08080808; //R8
	Stacks[i][STACKSIZE - 13] = 0x07070707; //R7
	Stacks[i][STACKSIZE - 14] = 0x06060606; //R6
	Stacks[i][STACKSIZE - 15] = 0x05050505; //R5
	Stacks[i][STACKSIZE - 16] = 0x04040404; //R4
}

//******** OS_Id *************** 
// returns the thread ID for the currently running thread
// Inputs: none
// Outputs: Thread ID, number greater than zero 
unsigned long OS_Id(void) {
	return RunPt->id;
}

//******** OS_AddPeriodicThread *************** 
// add a background periodic task
// typically this function receives the highest priority
// Inputs: pointer to a void/void background function
//         period given in system time units (12.5ns)
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// You are free to select the time resolution for this function
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In lab 2, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, this command will be called 0 1 or 2 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddPeriodicThread(void(*task)(void), 
   unsigned long period, unsigned long priority) {
		 Timer0A_Init(task, period, priority);  // initialize timer0A (16 Hz)
		 return 1;
}


//******** OS_AddSW1Task *************** 
// add a background task to run whenever the SW1 (PF4) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is the highest, 5 is the lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed that the user task will run to completion and return
// This task can not spin, block, loop, sleep, or kill
// This task can call OS_Signal  OS_bSignal	 OS_AddThread
// This task does not have a Thread ID
// In labs 2 and 3, this command will be called 0 or 1 times
// In lab 2, the priority field can be ignored
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW1Task(void(*task)(void), unsigned long priority) {
	PF4Edge_Init(task, priority);
	return 0;
}

//******** OS_AddSW2Task *************** 
// add a background task to run whenever the SW2 (PF0) button is pushed
// Inputs: pointer to a void/void background function
//         priority 0 is highest, 5 is lowest
// Outputs: 1 if successful, 0 if this thread can not be added
// It is assumed user task will run to completion and return
// This task can not spin block loop sleep or kill
// This task can call issue OS_Signal, it can call OS_AddThread
// This task does not have a Thread ID
// In lab 2, this function can be ignored
// In lab 3, this command will be called will be called 0 or 1 times
// In lab 3, there will be up to four background threads, and this priority field 
//           determines the relative priority of these four threads
int OS_AddSW2Task(void(*task)(void), unsigned long priority);

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(unsigned long sleepTime) {
	int status = StartCritical(); 
	RunPt->sleep = sleepTime;
	OS_Suspend();
	EndCritical(status);
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
void OS_Kill(void) {
	int status = StartCritical(); 
	tcbType* prevPt = RunPt->prev; 
	tcbType* nextPt = RunPt->next;
	idQueuePush(RunPt->id);
	prevPt->next = nextPt;
	nextPt->prev = prevPt;
	OS_Suspend();
	EndCritical(status);
}

void Scheduler(void) { //already called without allowing interrupts
	RunPt = RunPt->next;
	while(RunPt->sleep) { //checks sleeping lab 2.2
		RunPt = RunPt->next; 
	}
}

// ******** OS_Suspend ************
// suspend execution of currently running thread
// scheduler will choose another thread to execute
// Can be used to implement cooperative multitasking 
// Same function as OS_Sleep(0)
// input:  none
// output: none
void OS_Suspend(void) {
	NVIC_ST_CURRENT_R = 0;      // resets timer - TODO: dont have it reset the time 
	NVIC_INT_CTRL_R = 0x04000000; //trigger SysTick
	//NVIC_INT_CTRL_R = 0x10000000;  //trigger pendSV
}
 
// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
const long FIFOSIZE = 16;
uint32_t volatile *PutPt;
uint32_t volatile *GetPt;
uint32_t static Fifo[FIFOSIZE];
Sema4Type CurrentSize;
Sema4Type RoomLeft;
Sema4Type FIFOmutex;
uint32_t LostData;
void OS_Fifo_Init(unsigned long size)
{
	PutPt = GetPt = &Fifo[0];
	OS_InitSemaphore(&CurrentSize, 0);
	OS_InitSemaphore(&RoomLeft, FIFOSIZE);
	OS_InitSemaphore(&FIFOmutex, 1);
	LostData = 0;
}

// ******** OS_Fifo_Put ************
// Enter one data sample into the Fifo
// Called from the background, so no waiting 
// Inputs:  data
// Outputs: true if data is properly saved,
//          false if data not saved, because it was full
// Since this is called by interrupt handlers 
//  this function can not disable or enable interrupts
int OS_Fifo_Put(unsigned long data)
{
	if(CurrentSize.Value == FIFOSIZE)
	{
		LostData++;
		return -1;
	}
	*(PutPt) = data;
	PutPt++;
	if(PutPt == &Fifo[FIFOSIZE])
	{
		PutPt = &Fifo[0];
	}
	OS_Signal(&CurrentSize);
	return 0;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void)
{
	uint32_t data;
	OS_Wait(&CurrentSize);
	OS_Wait(&FIFOmutex);
	data = *(GetPt);
	GetPt++;
	if(GetPt == &Fifo[FIFOSIZE])
	{
		GetPt = &Fifo[0];
	}
	OS_Signal(&FIFOmutex);
	return data;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void)
{
	return CurrentSize.Value;
}

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void);

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data);

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void);

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
unsigned long OS_Time(void);

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop);

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void);

// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void);

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick

//******** OS_Launch *************** 
// start the scheduler, enable interrupts
// Inputs: number of 12.5ns clock cycles for each time slice
//         you may select the units of this parameter
// Outputs: none (does not return)
// In Lab 2, you can ignore the theTimeSlice field
// In Lab 3, you should implement the user-defined TimeSlice field
// It is ok to limit the range of theTimeSlice to match the 24-bit SysTick
void OS_Launch(unsigned long theTimeSlice) {
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}

void HardFault_Handler(void){
	LEDS = RED; 
}
