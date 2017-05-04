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
	//Timer 6: Periodic Thread for Pulse
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
#include "../LiuWareTM4C123Lab3/CAN/Timer4.h" //used for sleep decrement + OS Time

#define PB2  (*((volatile unsigned long *)0x40005010))
#define PB3  (*((volatile unsigned long *)0x40005020))
#define PB4  (*((volatile unsigned long *)0x40005040))
#define PB5  (*((volatile unsigned long *)0x40005080))
#define PB7  (*((volatile unsigned long *)0x40005200))
	
#define PERIOD1MS 80000

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode
void PortB_Init(void); 

#define FS 400              // producer/consumer sampling
#define RUNLENGTH (177 * FS )   // display results and quit when NumSamples==RUNLENGTH

//data variables
unsigned long PIDWork;      // current number of PID calculations finished
unsigned long FilterWork;   // number of digital filter calculations finished
unsigned long NumSamples;   // incremented every ADC sample, in Producer
unsigned long DataLost = 0;     // data sent by Producer, but not received by Consumer

//debug variables
unsigned long numCreated = 0;   // number of foreground threads created
int numThreads = 0; //not used but refered to in interpreter.o

//*********Prototype for FFT in cr4_fft_64_stm32.s, STMicroelectronics
void cr4_fft_64_stm32(void *pssOUT, void *pssIN, unsigned short Nbin);
//*********Prototype for PID in PID_stm32.s, STMicroelectronics
short PID_stm32(short Error, short *Coeff);
// add the following commands, leave other commands, if they make sense
// 1) print performance measures 
//    time-jitter, number of data points lost, number of calculations performed
//    i.e., NumSamples, NumCreated, MaxJitter, DataLost, FilterWork, PIDwork
      
// 2) print debugging parameters 
//    i.e., x[], y[] 

//--------------End of Interpreter-----------------------------

// 20-sec finite time experiment duration 
#define PERIOD TIME_500US   // DAS 2kHz sampling period in system time units
unsigned long xBottomLeft[64],yBottomLeft[64];           // input and output arrays for FFT
unsigned long xTopLeft[64],yTopLeft[64];           // input and output arrays for FFT
unsigned long xTopRight[64],yTopRight[64];           // input and output arrays for FFT
unsigned long xBottomRight[64],yBottomRight[64];           // input and output arrays for FFT

//--------------------End of Display on Screen 2------------------------

//------------------Hardware Triggered ADC--------------------------------
// hardware timer-triggered ADC sampling at 400Hz
// Producer runs as part of ADC ISR
// Producer uses fifo to transmit 400 samples/sec to Consumer
// every 64 samples, Consumer calculates FFT
// every 2.5ms*64 = 160 ms (6.25 Hz), consumer sends data to Display via mailbox
// Display thread updates LCD with measurement

//******** Producer *************** 
// The Producer in this lab will be called from your ADC ISR
// A timer runs at 400Hz, started by your ADC_Collect
// The timer triggers the ADC, creating the 400Hz sampling
// Your ADC ISR runs when ADC data is ready
// Your ADC ISR calls this function with a 12-bit sample 
// sends data to the consumer, runs periodically at 400Hz
// inputs:  none
// outputs: none
int result[3];
void Producer(IR_Data_Type data){  
  if(NumSamples < RUNLENGTH){   // finite time run
    NumSamples++;               // number of samples
    if(OS_Fifo_Put(data) == -1){ // send to consumer
      DataLost++;
    }
  } 
}
void Display(void); 
void StateMachine(void);

int calibrationChartBottomLeft[21][2] = {
	{7,3836},
	{8,3425},
	{9,3032},
	{10,2724},
	{11,2532},
	{13,2060},
	{15,1940},
	{20,1472},
	{25,1203},
	{30,1020},
	{35,879},
	{40,760},
	{45,664},
	{50,568},
	{55,472},
	{60,393},
	{65,321},
	{70,260},
	{75,196},
	{80,155},
	{85, 0}
};

int calibrationChartTopLeft[21][2] = {
	{7,3788},
	{8,3384},
	{9,3053},
	{10,2868},
	{11,2491},
	{13,2294},
	{15,1972},
	{20,1499},
	{25,1188},
	{30,1000},
	{35,861},
	{40,753},
	{45,645},
	{50,544},
	{55,448},
	{60,377},
	{65,303},
	{70,208},
	{75,275},
	{80,151},
	{85, 0}
};

int calibrationChartTopRight[21][2] = {
	{7,3840},
	{8,3508},
	{9,2964},
	{10,2812},
	{11,2554},
	{13,2305},
	{15,1970},
	{20,1589},
	{25,1252},
	{30,1037},
	{35,873},
	{40,769},
	{45,672},
	{50,573},
	{55,472},
	{60,424},
	{65,374},
	{70,300},
	{75,260},
	{80,215},
	{85, 0}
};

int calibrationChartBottomRight[21][2] = {
	{7,3818},
	{8,3452},
	{9,3156},	
	{10,2693},
	{11,2809},
	{13,2389},
	{15,2096},
	{20,1640},
	{25,1337},
	{30,1113},
	{35,932},
	{40,764},
	{45,641},
	{50,492},
	{55,421},
	{60,338},
	{65,300},
	{70,245},
	{75,240},
	{80,165},
	{85, 0}
};

int adcToDistance(int adcValue, int sensorNumber) {
	if(sensorNumber == 0) {
		for(int i = 0; i < 20; i++) {
			if(adcValue <= calibrationChartBottomLeft[i][1])
			{
				if(adcValue > calibrationChartBottomLeft[i+1][1])
				{
						//linear interpolate
						return -5*(adcValue-calibrationChartBottomLeft[i][1])/(calibrationChartBottomLeft[i][1]-calibrationChartBottomLeft[i+1][1])+calibrationChartBottomLeft[i][0];
				}
			}else{
				return 6;
			}
		}
		return 80;
	} else if(sensorNumber == 1) {
		for(int i = 0; i < 20; i++) {
			if(adcValue <= calibrationChartTopLeft[i][1])
			{
				if(adcValue > calibrationChartTopLeft[i+1][1])
				{
						//linear interpolate
						return -5*(adcValue-calibrationChartTopLeft[i][1])/(calibrationChartTopLeft[i][1]-calibrationChartTopLeft[i+1][1])+calibrationChartTopLeft[i][0];
				}
			} else {
				return 6;
			}
		}
	} else if(sensorNumber == 2) {
		for(int i = 0; i < 20; i++) {
			if(adcValue <= calibrationChartTopRight[i][1])
			{
				if(adcValue > calibrationChartTopRight[i+1][1])
				{
						//linear interpolate
						return -5*(adcValue-calibrationChartTopRight[i][1])/(calibrationChartTopRight[i][1]-calibrationChartTopRight[i+1][1])+calibrationChartTopRight[i][0];
				}
			}else{
				return 6;
			}
		}
	} else if(sensorNumber == 3) {
		for(int i = 0; i < 20; i++) {
			if(adcValue <= calibrationChartBottomRight[i][1])
			{
				if(adcValue > calibrationChartBottomRight[i+1][1])
				{
						//linear interpolate
						return -5*(adcValue-calibrationChartBottomRight[i][1])/(calibrationChartBottomRight[i][1]-calibrationChartBottomRight[i+1][1])+calibrationChartBottomRight[i][0];
				}
			}else{
				return 6;
			}
		}
	} 
	return -1;
}
//******** Consumer *************** 
// foreground thread, accepts data from producer
// calculates FFT, sends DC component to Display
// inputs:  none
// outputs: none
void Consumer(void){ 
IR_Data_Type data;
IR_Data_Type DCcomponent;   // 12-bit raw ADC sample, 0 to 4095
unsigned long t;                  // time in 2.5 ms
unsigned long myId = OS_Id(); 
	DataLost = 0; 
  IR_Sensor_Init(FS, &Producer); // start ADC sampling, channel 5, PD2, 400 Hz
  //numCreated += OS_AddThread(&Display,128,1);
	numCreated += OS_AddThread(&StateMachine,128,1);	
  while(NumSamples < RUNLENGTH - 32) { 
    for(t = 0; t < 64; t++){   // collect 64 ADC samples
      data = OS_Fifo_Get();    // get from producer - if we get stuck here then display may have been killed
			xBottomLeft[t] = data.BottomLeft;             // real part is 0 to 4095, imaginary part is 0
			xTopLeft[t] = data.TopLeft;
			xTopRight[t] = data.TopRight;
			xBottomRight[t] = data.BottomRight;
    }
    cr4_fft_64_stm32(yBottomLeft,xBottomLeft,64);  // complex FFT of last 64 ADC values
		cr4_fft_64_stm32(yTopLeft,xTopLeft,64);  // complex FFT of last 64 ADC values
		cr4_fft_64_stm32(yTopRight,xTopRight,64);  // complex FFT of last 64 ADC values
		cr4_fft_64_stm32(yBottomRight,xBottomRight,64);  // complex FFT of last 64 ADC values
    DCcomponent.BottomLeft = yBottomLeft[0]&0xFFFF; // Real part at frequency 0, imaginary part should be zero
		DCcomponent.TopLeft = yTopLeft[0]&0xFFFF;
		DCcomponent.TopRight = yTopRight[0]&0xFFFF;
    DCcomponent.BottomRight = yBottomRight[0]&0xFFFF;
		OS_MailBox_Send(DCcomponent); // called every 2.5ms*64 = 160ms
  }
	LEDS = RED;
  OS_Kill();  // never called
}

void uartIRData(IR_Data_Type pData, int pingData) {
		UART_OutString("ADC VAlue -----------");
		OutCRLF();
		UART_OutUDec(pData.BottomLeft);
		OutCRLF();
		UART_OutUDec(pData.TopLeft);
		OutCRLF();
		UART_OutUDec(pingData);
		OutCRLF();
		UART_OutUDec(pData.TopRight);
		OutCRLF();
		UART_OutUDec(pData.BottomRight);
		OutCRLF(); 
}

//ping
extern unsigned int pulsePeriodPing6_7;
extern int pingPB6_7_ready;
//end ping

//******** Display *************** 
// foreground thread, accepts data from consumer
// displays calculated results on the LCD
// inputs:  none                            
// outputs: none
void Display(void){
IR_Data_Type data;
IR_Data_Type distance;
int pingDistance; 
  while(NumSamples < RUNLENGTH - 32) {
		IR_Data_Type tempData;
    tempData = OS_MailBox_Recv();
    data = tempData;
		distance.BottomLeft = adcToDistance(data.BottomLeft, 0);               // calibrate your device so voltage is in mV
		distance.TopLeft = adcToDistance(data.TopLeft, 1);
		distance.TopRight = adcToDistance(data.TopRight, 2);
		distance.BottomRight = adcToDistance(data.BottomRight, 3);
		
		//ping
		if(pingPB6_7_ready == 1) {
			pingPB6_7_ready = 0;
			pingDistance = (343 * pulsePeriodPing6_7)/1600000;
		}
		
		//display
		uartIRData(distance, pingDistance);
		//uartIRData(data, pingDistance);
  }
	LEDS = RED;
  OS_Kill();  // never called
} 

int inBetween(int value, int low, int high) {
	if(value >= low && value < high) {
		return 1;
	}
	return 0;
}

typedef enum
{
    BOTTOMLEFTMIN,
    BOTTOMLEFTMAX,
		TOPLEFTMIN,
		TOPLEFTMAX,
		PINGMIN,
		PINGMAX,
		TOPRIGHTMIN,
		TOPRIGHTMAX,
		BOTTOMRIGHTMIN,
		BOTTOMRIGHTMAX,
		BOTTOMLEFTDIFFERENCEMIN,
		BOTTOMLEFTDIFFERENCEMAX,
		BOTTOMRIGHTIFFERENCEMIN,
		BOTTOMRIGHTIFFERENCEMAX,
		DIRECTION,
		SERVOMAGNITUDE,
		LEFTMOTORMAGITUDE,
		RIGHTMOTORMAGNITUDE,
		LUTWIDTH
}
LutEnum;

typedef enum
{
    STRAIGHT, //0
		SLIGHTLEFT, //1
		HARDLEFT, //2
		SLIGHTRIGHT, //3
		HARDRIGHT, //4
		REVERSELEFT, //5
		REVERSERIGHT, //6
		STOP
}
DirectionEnum;

#define LUTLENGH 6	

int CONTROL_LUT[LUTLENGH][LUTWIDTH] = {                      
//{BottomLeft Min, BottomLeft Max, TopLeftMin, TopLeftMax, Ping Min, Ping Max, TopRightMin, TopRightMax, BottomRight Min, BottomRight Max, Bottom Difference Min, Bottom Difference Max, Top Difference Min, Top Difference Max, Direction, ServoMagnitude, LeftMotor, RightMotor}
																														 //Difference
//{BLMn, BLMx, TLMn, TLMx, PMn, PMx, TRMn, TRMx, BRMn, BRMx, BLDMn, BLDMx, BRDMn, BRDMx, LDMn,  LDMx,  RDMn,  RDMx, D, Servo, LMotor, RMotor}
//{0,    85,   0,    85,   0,   500, 0,    85,   0,    85,   0,     85,    0,     85,    0,     85,    0,     85,    0, 0,     1,      1    },
	//simple straights - 2
	//19
	{15,   85,   10,   85,   10,  500, 15,   85,  10,   85,   0,     18,    0,     18,    STRAIGHT, 0,     4,      4    }, //works at speed 2 //10
	{0,    85,   0,    85,   0,   500, 0,    85,  0,    85,   0,     28,    0,     28,    STRAIGHT, 0,     3,      3    }, //works at speed 2	//11
	
	{0,    72,   0,    45,   0,   500, 50,    85,  0,    72,   0,     85,    0,     85,    SLIGHTLEFT, 4,     2,      4    }, //works at speed 2	//11
	{0,    72,   50,    85,   0,   500, 0,    45,   0,    50,   0,     85,    0,     85,    SLIGHTRIGHT, 4,     4,      2    }, //works at speed 2	//11

	
	//hard left - 1
	{50,   85,   0,    85,   0,   500, 0,    85,   0,    85,   1,     85,    0,     85,    HARDLEFT, 0,     5,      5    }, //11
	//hard right - 1
	{0,   85,    0,    85,   0,   500, 0,    85,   50,   85,   0,     85,    1,     85,    HARDRIGHT,0,     5,      5    }, //12
	};

	/*
		//near edge straights - leftside first - 10
	//2
	{10,   14,   19,  24,   10,  500, 10,   85,    10,   70,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    }, //1
	{15,   16,   23,  30,   10,  500, 10,   85,    10,   65,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    }, //2
	{17,   18,   27,  34,   10,  500, 10,   85,    10,   55,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    },	//4
	{19,   20,   29,  36,   10,  500, 10,   85,    10,   55,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    },	//5
	{21,   22,   32,  38,   10,  500, 10,   85,    10,   55,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    },	 //6
  //8
	{10,   70,   10,  85,   10,  500, 19,   24,   10,    14,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    }, //7
	{10,   65,   10,  85,   10,  500, 23,   30,   15,    16,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    }, //8
	{10,   55,   10,  85,   10,  500, 27,   34,   17,    18,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    }, //9
	{10,   55,   10,  85,   10,  500, 36,   36,   19,    20,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    },	//10
	{10,   55,   10,  85,   10,  500, 38,   38,   21,    22,   0,     85,    0,     85,    0,     85,    0,     85,    STRAIGHT, 0,     5,      5    },	 //11

	//curve right - 2
  //13
	{6,    12,   0,  85,   10,  500, 10,     85,  10,    85,   0,     85,    0,     85,    0,     85,    0,     85,    SLIGHTRIGHT, 1,     4,      3    },	//7

	{10,   85,   10, 85,   10,  500, 0,      85,  6,     12,   0,     85,    0,     85,    0,     85,    0,     85,    SLIGHTLEFT, 1,     4,      3    },	//7

//{BLMn, BLMx, TLMn, TLMx, PMn, PMx, TRMn, TRMx, BRMn, BRMx, BLDMn, BLDMx, BRDMn, BRDMx, LDMn,  LDMx,  RDMn,  RDMx, D, Servo, LMotor, RMotor}

	//correct left - 4
	//15
	{13,    16,   40,  85,   10,  500, 10,   85,  10,    55,   0,     85,    0,     85,    0,     85,    0,     85,    SLIGHTLEFT, 1,     3,      4    },	//8
	{17,    24,   46,  85,   10,  500, 10,   85,  10,    55,   0,     85,    0,     85,    0,     85,    0,     85,    SLIGHTLEFT, 1,     3,      4    },	//9
	
  {10,    55,   10,  85,   10,  500, 40,   85,  13,    16,   0,     85,    0,     85,    0,     85,    0,     85,    SLIGHTRIGHT, 1,     3,      4    },	//8
	{10,    55,   10,  85,   10,  500, 46,   85,  17,    24,   0,     85,    0,     85,    0,     85,    0,     85,    SLIGHTRIGHT, 1,     3,      4    },	//9

	*/
	
int abs(int num1, int num2) {
	if(num1 >= num2) {
		return num1 - num2;
	} else {
		return num2 - num1;
	}
}

int greaterSubs (int greater, int less) {
	if(greater >= less) {
		return greater - less;
	} else {
		return 0;
	}
}

int findIndexLUT(IR_Data_Type pDistance, int pPingDistance) {
	int index = -1;
	for(int i = 0; i < LUTLENGH; i++) {
		if(pDistance.BottomLeft >= CONTROL_LUT[i][BOTTOMLEFTMIN]
			&& pDistance.BottomLeft <= CONTROL_LUT[i][BOTTOMLEFTMAX]
			&& pDistance.TopLeft >= CONTROL_LUT[i][TOPLEFTMIN]
			&& pDistance.TopLeft <= CONTROL_LUT[i][TOPLEFTMAX]
			&& pPingDistance >= CONTROL_LUT[i][PINGMIN]
			&& pPingDistance <= CONTROL_LUT[i][PINGMAX]
			&& pDistance.TopRight >= CONTROL_LUT[i][TOPRIGHTMIN]
			&& pDistance.TopRight <= CONTROL_LUT[i][TOPRIGHTMAX]
			&& pDistance.BottomRight >= CONTROL_LUT[i][BOTTOMRIGHTMIN]
			&& pDistance.BottomRight <= CONTROL_LUT[i][BOTTOMRIGHTMAX]
			&& greaterSubs(pDistance.BottomLeft, pDistance.BottomRight)  >= CONTROL_LUT[i][BOTTOMLEFTDIFFERENCEMIN]
			&& greaterSubs(pDistance.BottomLeft, pDistance.BottomRight)  <= CONTROL_LUT[i][BOTTOMLEFTDIFFERENCEMAX]
			&& greaterSubs(pDistance.BottomRight, pDistance.BottomLeft)  >= CONTROL_LUT[i][BOTTOMRIGHTIFFERENCEMIN]
			&& greaterSubs(pDistance.BottomRight, pDistance.BottomLeft)  <= CONTROL_LUT[i][BOTTOMRIGHTIFFERENCEMAX]
		) {
			return i;
		}
	}
	return index;
}

uint8_t XmtData[8];
uint8_t RcvData[8];
uint32_t RcvCount=0;
uint32_t lastState = 1;
uint32_t reverseState = 0; //0 means left, 1 means right
int oldLutIndex = -1;
int newLutIndex;
void StateMachine(void){ 
IR_Data_Type data;
IR_Data_Type distance;
int pingDistance; 
  while(NumSamples < RUNLENGTH - 32) {
		//IR Sensors
		IR_Data_Type tempData;
    tempData = OS_MailBox_Recv();
    data = tempData;
		distance.BottomLeft = adcToDistance(data.BottomLeft, 0);               // calibrate your device so voltage is in mV
		distance.TopLeft = adcToDistance(data.TopLeft, 1);
		distance.TopRight = adcToDistance(data.TopRight, 2);
		distance.BottomRight = adcToDistance(data.BottomRight, 3);
		
		//ping
		if(pingPB6_7_ready == 1) {
			pingPB6_7_ready = 0;
			pingDistance = (343 * pulsePeriodPing6_7)/1600000;
		} 
		
		uartIRData(distance, pingDistance);
		
		//CAN
		newLutIndex = findIndexLUT(distance, pingDistance);
		
		
				UART_OutString("State Machine -----------");
		OutCRLF();
		UART_OutUDec(newLutIndex);
		OutCRLF();
	
		if(newLutIndex != oldLutIndex) {
			oldLutIndex = newLutIndex;
			if(newLutIndex != -1) {
				XmtData[0] = CONTROL_LUT[newLutIndex][DIRECTION];
				XmtData[1] = CONTROL_LUT[newLutIndex][SERVOMAGNITUDE];
				XmtData[2] = CONTROL_LUT[newLutIndex][LEFTMOTORMAGITUDE];
				XmtData[3] = CONTROL_LUT[newLutIndex][RIGHTMOTORMAGNITUDE];
				// if(CONTROL_LUT[newLutIndex][DIRECTION] == SLIGHTLEFT || CONTROL_LUT[newLutIndex][DIRECTION]) - impliments reversing
				
				//UART_OutUDec(CONTROL_LUT[newLutIndex][DIRECTION]);
				//OutCRLF();
				
				CAN0_SendData(XmtData);
				OS_Sleep(1);
			}
		}
  }
	XmtData[0] = STOP;
	CAN0_SendData(XmtData);
	LEDS = RED;
  OS_Kill();  // never called
} 


//-----------------End of Hardware Triggered ADC---------------------------

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
      Actuator = PID_stm32(err,Coeff)/256;
    }
    PIDWork++;        // calculation finished
  }
  for(;;){
		PB3 ^= 0x08;
	}          // done
}
//--------------------------End of PID------------------------------

uint8_t sequenceNum=0;  
void CanSendMessage(void) {
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
			UART_OutChar(RcvData[0]);
			UART_OutChar(RcvData[1]);
			UART_OutChar(RcvData[2]);
			UART_OutChar(RcvData[3]);
			UART_OutChar(RcvData[4]);
			UART_OutChar(RcvData[5]);
			UART_OutChar(RcvData[6]);
			UART_OutChar(RcvData[7]);
			OutCRLF();
			OS_Sleep(10);
    }
	}	
}

void postLauntInits(){
	long sr = StartCritical();
	
	//can
	CAN0_Open();
	//OS_AddPeriodicThread(&CanSendMessage,FS*2,2);           // 1 ms, higher priority
	//Timer4_Init(&UserTask, 1600000); // initialize timer3 (10 Hz) // Sleep
	
	EndCritical(sr);
	
	//UART
	UART_Init();
	Timer0A_Init();
	//Debug
	//UART_OutString("Hello Lab 6.0.1");
	//OutCRLF();
	
	OS_Kill();
}


//*******************final user main DEMONTRATE THIS TO TA**********
int main(void){
  OS_Init();           // initialize, disable interrupts
	
 
  DataLost = 0;        // lost data between producer and consumer
  NumSamples = 0;
	FilterWork = 0;
  numCreated = 0;
//********initialize communication channels
  OS_MailBox_Init();		
  OS_Fifo_Init(32);

//*******attach background tasks***********
  //OS_AddSW1Task(&SW1Push,2); //not stuck
  //OS_AddSW2Task(&SW2Push,2);  // add this line in Lab 3
	
	ST7735_InitRDivided(INITR_REDTAB);
	///periodic test
	OS_AddPeriodicThread(StartPingSensorPB6_7, 100* TIME_1MS,2);
	
	//create initial foreground threads
	
	numCreated += OS_AddThread(&postLauntInits,128,1);
	//numCreated += OS_AddThread(&Interpreter,128,5);
	//numCreated += OS_AddThread(&CANTest,128,2);
	numCreated += OS_AddThread(&Consumer,128,2); 
  numCreated += OS_AddThread(&PID,128,6);  // Lab 3, make this lowest priority

  OS_Launch(TIME_2MS); // doesn't return, interrupts enabled in here
  return 0;            // this never executes
}
