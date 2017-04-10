// OS.c
// Uses time to keep time for our real time OS. 
// TA: Daniel Leach

#include <stdint.h>
#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"
#include "../LiuWareTM4C123Lab3/OS.h"  
#include "../LiuWareTM4C123Lab3/PLL.h"
#include "../LiuWareTM4C123Lab3/Timer4A.h" //used for sleep decrement + OS Time
#include "../LiuWareTM4C123Lab3/Timer0A.h" //used for periodic thread 0
#include "../LiuWareTM4C123Lab3/Timer1.h" //used for periodic thread 1
#include "../LiuWareTM4C123Lab3/PFEdgeTrigger.h"
#include "../LiuWareTM4C123Lab3/ADC.h" //ADC uses timer 2
#include "../LiuWareTM4C123Lab3/IDQueue.h"
#include "../LiuWareTM4C123Lab3/ThreadPriorityPQueue.h" 
#include "../LiuWareTM4C123Lab3/LEDS.h" 


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode 
void StartOS(void);
void doSleepDecrement(void);
void updateOSTime(void); 

#define TIME_1MS   80000  
#define NUMTHREADS 10
#define NUMTHREADSPLUSONE (NUMTHREADS + 1)
#define STACKSIZE 100

extern unsigned long DataLost;     // data sent by Producer, but not received by Consumer
extern unsigned long numCreated;   // number of foreground threads created
extern int numThreads;
extern int debugBlocked;

//---------------- TCB ------------------- 
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];

//---------------- OSTime -------------------
unsigned long OStime; //time in MS
unsigned long OStimeMS;

/* Tasks to be done every 1MS */
void doSleepDecrement() {
	int status = StartCritical(); //critical section because tcb objects are global
	tcbType *oldRunPt = RunPt;
	tcbType *currentRunPt = RunPt;
	do {
		if(1 == currentRunPt->sleep) {
			priQueuePush(currentRunPt);
		}
		if(0 != currentRunPt->sleep) {
			(currentRunPt->sleep)--; 
		}	
		currentRunPt = currentRunPt->next; // move onto next pointer
	} while(oldRunPt != currentRunPt); 
	updateOSTime(); 
	EndCritical(status); 
}

void updateOSTime() { //MS
	OStime++;
	OStimeMS++;
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
  LEDS = GREEN;                        // initalize Led to Green
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
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7 for SysTick - used for pre-emptive scheduler context switch
	NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0xFF00FFFF)|0x00C00000; // priority 6 for PendSV - used for kill context switch
	//initialize idQueue[]
	for(int i = 0; i < NUMTHREADS + 1; i++) {
		idQueuePush(i); 
	}
	//Timer 4A used for Sleep Decrement + Incrementing OS Time
	int tempPriority = 1;
	Timer4A_Init(&doSleepDecrement, TIME_1MS, tempPriority);
	//Sets up LEDs for hardfault debugging and debugging
	initHardFault(); 
	//Initialize Semaphores to be used for Initalizing Drivers
	OS_InitDriverInitSemaphore();
	OStime = 0; //time in MS
	OStimeMS = 0; //time in MS but can be cleared
}

// ******** OS_InitDriverInitSemaphore ************
// initialize semaphore for initalizing drivers 
// input:  none
// output: none
void OS_InitDriverInitSemaphore() {
	OS_InitSemaphore(&mutexPFInit, 1); 
}

// ******** OS_InitSemaphore ************
// initialize semaphore 
// input:  pointer to a semaphore
// output: none
void OS_InitSemaphore(Sema4Type *semaPt, long pValue) {
	long sr = StartCritical(); 
	(semaPt->Value) = pValue;
	EndCritical(sr);
}

// ******** OS_Wait ************
// decrement semaphore 
// Lab2 spinlock
// Lab3 block if less than zero
// input:  pointer to a counting semaphore
// output: none
// Assumption: can only be called by the running thread
// Assumption each thread can only wait one resource
void OS_Wait(Sema4Type *semaPt) {
	DisableInterrupts();
	(semaPt->Value) = (semaPt->Value) - 1; //semaphore value tells you how many resources you have or if negative how many are waiting, all resources are in use but none are waiting																				 // -1 means I want to use it
	semaPt->UserPriority = RunPt->priority; //debug
	//-- add to list
	if(semaPt->Value == -1) { //I am first one waiting
		debugBlocked++;
		RunPt->blockedState = 1;
		semaPt->listHeadPtr = RunPt; 
		RunPt->blockedNext = NULL; //if previously used it may not be null
		semaPt->listEndPtr = RunPt;
		//switch context
		priQueueRemove(RunPt); //remove running thread from active
		OS_Suspend(); //almost equal to an return
	} else if(semaPt->Value < -1) { //there are otherse waiting
		debugBlocked++;
		RunPt->blockedState = 1;
		(semaPt->listEndPtr)->blockedNext = RunPt;
		RunPt->blockedNext = NULL; //if previously used it may not be null
		semaPt->listEndPtr = RunPt;
		priQueueRemove(RunPt); //remove running thread from active
		OS_Suspend(); //almost equal to an return
	}
	//-- end add to list
		EnableInterrupts(); //enable interrupts can occur here - cannot use critical section since end critical section doesn't guarentee interrupts enabled
}

// ******** OS_Signal ************
// increment semaphore 
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate 
// input:  pointer to a counting semaphore
// output: none
void OS_Signal(Sema4Type *semaPt) {
	DisableInterrupts();
	(semaPt->Value) = (semaPt->Value) + 1; //I am freeing the resource that I was using
	semaPt->SignalerPriority = RunPt->priority; //debug - who signaled last on this semaphore
	if(semaPt->Value <= 0) { //if it was previously negative one -> which means atleast one is waiting
		debugBlocked--;
		RunPt->blockedState = 0;
		priQueuePush((semaPt->listHeadPtr)); //add first thread to list
		//update list
		tcbType *newHead = (semaPt->listHeadPtr)->blockedNext; 
		semaPt->UserPriority = newHead->priority; //debug - who is now using the resource?
		semaPt->listHeadPtr = newHead; 
	}
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
int firstThreadEver = 1; //initally true (1) that we are in the state that the next add would be first thread 
int OS_AddThread(void(*task)(void), unsigned long stackSize, unsigned long priority) {
	int32_t status; 
	status = StartCritical(); 
	int nextFreeID = idQueuePop(); 
	if(-1 == nextFreeID) {
		EndCritical(status); //Dan helped
		return 0;
	}
	numThreads++; //debugging for num threads
	if(0 == firstThreadEver) { //not first thread ever
		tcbType* oldPrevPt = (RunPt -> prev); 
		oldPrevPt->next = &tcbs[nextFreeID]; 
		RunPt->prev = &tcbs[nextFreeID]; 
		SetInitialStack(nextFreeID);
		Stacks[nextFreeID][STACKSIZE-2] = (int32_t)(task); //PC
		tcbs[nextFreeID].next = RunPt;///&tcbs[nextFreeID];
		tcbs[nextFreeID].prev = oldPrevPt;
		tcbs[nextFreeID].id = nextFreeID;
		tcbs[nextFreeID].sleep = 0;
		tcbs[nextFreeID].priority = priority; 
		tcbs[nextFreeID].blockedNext = NULL; 
		tcbs[nextFreeID].blockedState = 0;
		//add pointer to active queue
		priQueuePush(&tcbs[nextFreeID]); 
	} else if(1 == firstThreadEver) {
		//set flag to permanently 0
		firstThreadEver = 0;
		//set up tcb
		SetInitialStack(nextFreeID);
		Stacks[nextFreeID][STACKSIZE-2] = (int32_t)(task); //PC
		tcbs[nextFreeID].next = &tcbs[nextFreeID];
		tcbs[nextFreeID].prev = &tcbs[nextFreeID];
		tcbs[nextFreeID].id = nextFreeID;
		tcbs[nextFreeID].sleep = 0;
		tcbs[nextFreeID].priority = priority;
		tcbs[nextFreeID].blockedNext = NULL; 
		tcbs[nextFreeID].blockedState = 0;
		//add pointer to active queue
		priQueuePush(&tcbs[nextFreeID]);
		//init run pointer
		RunPt = priQueuePeek();
		
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
int numPeriodicThreads = 0; 
int OS_AddPeriodicThread(void(*task)(void), unsigned long period, unsigned long priority) {
	if(numPeriodicThreads == 0) {
		Timer0A_Init(task, period, priority);  // initialize timer0A (16 Hz)
		numPeriodicThreads = 1; 
		return 1;
	} else if(numPeriodicThreads == 1) {
		Timer1_Init(task, period, priority); 
		numPeriodicThreads = 2;
		return 1; 
	}
	return 0;
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
	PFEdge_Init(2); //spawns thread
	PF4Edge_Task(task, priority);
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
int OS_AddSW2Task(void(*task)(void), unsigned long priority) {
	PFEdge_Init(2);
	PF0Edge_Task(task, priority);
	return 0; 
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// You are free to select the time resolution for this function
// OS_Sleep(0) implements cooperative multitasking
// Millisecond Resolution -> already put don't need to multiply by TIME_1MS
void OS_Sleep(unsigned long sleepTime) {
	int status = StartCritical(); 
	// priQueue first item is the running thread which we want to pop off
	priQueueRemove(RunPt);
	RunPt->sleep = sleepTime;
	OS_Suspend();
	EndCritical(status);
}

// ******** OS_Kill ************
// kill the currently running thread, release its TCB and stack
// input:  none
// output: none
//	Critical Section -> RunPt
//  OS_Kill takes a working RunPt and either
//		1) Renders it invalid
//		2) Points it 
void OS_Kill(void) {
	int status = StartCritical(); 
	// priQueue first item is the running thread which we want to pop off
	priQueueRemove(RunPt);
	//takes removes it from TCB List */
	tcbType* prevPt = RunPt->prev; 
	tcbType* nextPt = RunPt->next;
	idQueuePush(RunPt->id);
	prevPt->next = nextPt;
	nextPt->prev = prevPt;
	/* Finish removing form TCB List */
	//See whats the next item
	RunPt = priQueuePeek(); 
	numThreads--; 
	OS_KilledSuspend(); //OS_Suspend(); - uses pendSV instead which doesn't load old values
	
	EndCritical(status);
	while(1){ //debug because we know we cannot return here
		LEDS = BLUE;
	}
}

void Scheduler(void) { //already called without allowing interrupts
	RunPt = priQueuePeekAndRotate();
	//RunPt = priQueuePeek();
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
}

void OS_KilledSuspend(void) {
	NVIC_ST_CURRENT_R = 0;      // resets timer - TODO: dont have it reset the time 
	//NVIC_INT_CTRL_R = 0x04000000; //trigger SysTick
	NVIC_INT_CTRL_R = 0x10000000;  //trigger pendSV
}

//---------------- OS_Fifo Vars -------------------
const long FIFOSIZE = 32;
uint32_t volatile *PutPt;
uint32_t volatile *GetPt;
uint32_t static Fifo[FIFOSIZE];
Sema4Type CurrentSize;
Sema4Type RoomLeft;
Sema4Type FIFOmutex;
uint32_t LostData;

// ******** OS_Fifo_Init ************
// Initialize the Fifo to be empty
// Inputs: size
// Outputs: none 
// In Lab 2, you can ignore the size field
// In Lab 3, you should implement the user-defined fifo size
// In Lab 3, you can put whatever restrictions you want on size
//    e.g., 4 to 64 elements
//    e.g., must be a power of 2,4,8,16,32,64,128
// size must be hard coded
void OS_Fifo_Init(unsigned long size) {
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
int OS_Fifo_Put(unsigned long data) {
	OS_Wait(&RoomLeft);
	OS_Wait(&FIFOmutex);
	if(CurrentSize.Value == FIFOSIZE) { //data loss recorder
		LostData++;
		return -1;
	}
	*(PutPt) = data;
	PutPt++;
	if(PutPt == &Fifo[FIFOSIZE]) {
		PutPt = &Fifo[0];
	}
	OS_Signal(&FIFOmutex);
	OS_Signal(&CurrentSize);
	return 0;
}

// ******** OS_Fifo_Get ************
// Remove one data sample from the Fifo
// Called in foreground, will spin/block if empty
// Inputs:  none
// Outputs: data 
unsigned long OS_Fifo_Get(void) {
	uint32_t data;
	OS_Wait(&CurrentSize);
	OS_Wait(&FIFOmutex);
	data = *(GetPt);
	GetPt++;
	if(GetPt == &Fifo[FIFOSIZE]) {
		GetPt = &Fifo[0];
	}
	OS_Signal(&FIFOmutex);
	OS_Signal(&RoomLeft);
	return data;
}

// ******** OS_Fifo_Size ************
// Check the status of the Fifo
// Inputs: none
// Outputs: returns the number of elements in the Fifo
//          greater than zero if a call to OS_Fifo_Get will return right away
//          zero or less than zero if the Fifo is empty 
//          zero or less than zero if a call to OS_Fifo_Get will spin or block
long OS_Fifo_Size(void) {
	return CurrentSize.Value;
}

//---------------- MailBox Vars -------------------
unsigned long mailValue;
Sema4Type mailSend;
Sema4Type mailAck; 

// ******** OS_MailBox_Init ************
// Initialize communication channel
// Inputs:  none
// Outputs: none
void OS_MailBox_Init(void) {
	mailValue = 0;
	mailSend.Value = 0;
  mailAck.Value = 1; 
}

// ******** OS_MailBox_Send ************
// enter mail into the MailBox
// Inputs:  data to be sent
// Outputs: none
// This function will be called from a foreground thread
// It will spin/block if the MailBox contains data not yet received 
void OS_MailBox_Send(unsigned long data) {
	OS_Wait(&mailAck);
	mailValue = data;
	OS_Signal(&mailSend);  
}

// ******** OS_MailBox_Recv ************
// remove mail from the MailBox
// Inputs:  none
// Outputs: data received
// This function will be called from a foreground thread
// It will spin/block if the MailBox is empty 
unsigned long OS_MailBox_Recv(void) {
	unsigned long theData;
	OS_Wait(&mailSend);
	theData = mailValue; 
	OS_Signal(&mailAck);
	return theData; 
}

// ******** OS_Time ************
// return the system time 
// Inputs:  none
// Outputs: time in 12.5ns units, 0 to 4294967295
// The time resolution should be less than or equal to 1us, and the precision 32 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_TimeDifference have the same resolution and precision 
// Resolution: Bus Cycles
unsigned long OS_Time(void) {
	
	long result = (OStime * TIME_1MS) + (80000 - TIMER4_TAV_R); //bus cycles

	return result; 
}

// ******** OS_TimeDifference ************
// Calculates difference between two times
// Inputs:  two times measured with OS_Time
// Outputs: time difference in 12.5ns units 
// The time resolution should be less than or equal to 1us, and the precision at least 12 bits
// It is ok to change the resolution and precision of this function as long as 
//   this function and OS_Time have the same resolution and precision 
// Resolution: Bus Cycles
unsigned long OS_TimeDifference(unsigned long start, unsigned long stop) {
	unsigned long result = (stop - start);
	return result; 
}

// ******** OS_ClearMsTime ************
// sets the system time to zero (from Lab 1)
// Inputs:  none
// Outputs: none
// You are free to change how this works
void OS_ClearMsTime(void) {
	OStimeMS = 0; 
}


// ******** OS_MsTime ************
// reads the current time in msec (from Lab 1)
// Inputs:  none
// Outputs: time in ms units
// You are free to select the time resolution for this function
// It is ok to make the resolution to match the first call to OS_AddPeriodicThread
unsigned long OS_MsTime(void) { 
	return OStimeMS; 
}

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
	RunPt = priQueuePeek();
  StartOS();                   // start on the first task
}

long lockCritical = 0;
unsigned long OS_LockScheduler(void){
  unsigned long old = NVIC_ST_CTRL_R;
  NVIC_ST_CTRL_R = NVIC_ST_CTRL_ENABLE+NVIC_ST_CTRL_CLK_SRC;
	//lockCritical = StartCritical();
  return old;
}

void OS_UnLockScheduler(unsigned long previous){
  NVIC_ST_CTRL_R = previous;
	//EndCritical(lockCritical);
}

void HardFault_Handler(void){
	LEDS = BLUE; 
}
