// ADC.c
// Driver for ADC
// TA: Daniel Leach

#include <stdint.h>
#include "../LiuWareTM4C123Lab3/tm4c123gh6pm.h"
#include "../LiuWareTM4C123Lab3/UART.h"
#include "../LiuWareTM4C123Lab3/ADC.h"
#include "../LiuWareTM4C123Lab3/OS.h"

#define NVIC_EN0_INT17          0x00020000  // Interrupt 17 enable
#define TIMER_CFG_16_BIT        0x00000004  // 16-bit timer configuration,
                                            // function is controlled by bits
                                            // 1:0 of GPTMTAMR and GPTMTBMR
#define TIMER_TAMR_TACDIR       0x00000010  // GPTM Timer A Count Direction
#define TIMER_TAMR_TAMR_PERIOD  0x00000002  // Periodic Timer mode
#define TIMER_CTL_TAOTE         0x00000020  // GPTM TimerA Output Trigger
                                            // Enable
#define TIMER_CTL_TAEN          0x00000001  // GPTM TimerA Enable
#define TIMER_IMR_TATOIM        0x00000001  // GPTM TimerA Time-Out Interrupt
                                            // Mask
#define TIMER_TAILR_TAILRL_M    0x0000FFFF  // GPTM TimerA Interval Load
                                            // Register Low
#define ADC_ACTSS_ASEN3         0x00000008  // ADC SS3 Enable
#define ADC_RIS_INR3            0x00000008  // SS3 Raw Interrupt Status
#define ADC_IM_MASK3            0x00000008  // SS3 Interrupt Mask
#define ADC_ISC_IN3             0x00000008  // SS3 Interrupt Status and Clear
#define ADC_EMUX_EM3_M          0x0000F000  // SS3 Trigger Select mask
#define ADC_EMUX_EM3_TIMER      0x00005000  // Timer
#define ADC_SSPRI_SS3_4TH       0x00003000  // fourth priority
#define ADC_SSPRI_SS2_3RD       0x00000200  // third priority
#define ADC_SSPRI_SS1_2ND       0x00000010  // second priority
#define ADC_SSPRI_SS0_1ST       0x00000000  // first priority
#define ADC_PSSI_SS3            0x00000008  // SS3 Initiate
#define ADC_SSCTL3_TS0          0x00000008  // 1st Sample Temp Sensor Select
#define ADC_SSCTL3_IE0          0x00000004  // 1st Sample Interrupt Enable
#define ADC_SSCTL3_END0         0x00000002  // 1st Sample is End of Sequence
#define ADC_SSCTL3_D0           0x00000001  // 1st Sample Diff Input Select
#define ADC_SSFIFO3_DATA_M      0x00000FFF  // Conversion Result Data mask
#define ADC_PC_SR_M             0x0000000F  // ADC Sample Rate
#define ADC_PC_SR_125K          0x00000001  // 125 ksps
#define SYSCTL_RCGCGPIO_R4      0x00000010  // GPIO Port E Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R3      0x00000008  // GPIO Port D Run Mode Clock
                                            // Gating Control
#define SYSCTL_RCGCGPIO_R1      0x00000002  // GPIO Port B Run Mode Clock
                                            // Gating Control
																			
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

/* Producer Task
* Function: Serves as callback function for handler to call. 
* Input Parameter: ADC Sequencer Input
*/
void (*ProducerTask)(IR_Data_Type data);   // user function

int IR_Sensor_Init(uint32_t fs, void(*pTask)(IR_Data_Type data)) {
	ProducerTask = pTask;
	volatile uint32_t delay;
  // **** GPIO pin initialization ****
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // activate port GPIO_PORTE
  delay = SYSCTL_RCGCGPIO_R;      // 2) allow time for clock to stabilize
  delay = SYSCTL_RCGCGPIO_R;
	//      Ain0 is on PE3
	GPIO_PORTE_DIR_R &= ~0x08;  // 3.0) make PE3 input
	GPIO_PORTE_AFSEL_R |= 0x08; // 4.0) enable alternate function on PE3
	GPIO_PORTE_DEN_R &= ~0x08;  // 5.0) disable digital I/O on PE3
	GPIO_PORTE_AMSEL_R |= 0x08; // 6.0) enable analog functionality on PE3
	//      Ain1 is on PE2
	GPIO_PORTE_DIR_R &= ~0x04;  // 3.1) make PE2 input
	GPIO_PORTE_AFSEL_R |= 0x04; // 4.1) enable alternate function on PE2
	GPIO_PORTE_DEN_R &= ~0x04;  // 5.1) disable digital I/O on PE2
	GPIO_PORTE_AMSEL_R |= 0x04; // 6.1) enable analog functionality on PE2
	//      Ain2 is on PE1
	GPIO_PORTE_DIR_R &= ~0x02;  // 3.2) make PE1 input
	GPIO_PORTE_AFSEL_R |= 0x02; // 4.2) enable alternate function on PE1
	GPIO_PORTE_DEN_R &= ~0x02;  // 5.2) disable digital I/O on PE1
	GPIO_PORTE_AMSEL_R |= 0x02; // 6.2) enable analog functionality on PE1
	//      Ain3 is on PE0
	GPIO_PORTE_DIR_R &= ~0x01;  // 3.3) make PE0 input
	GPIO_PORTE_AFSEL_R |= 0x01; // 4.3) enable alternate function on PE0
	GPIO_PORTE_DEN_R &= ~0x01;  // 5.3) disable digital I/O on PE0
	GPIO_PORTE_AMSEL_R |= 0x01; // 6.3) enable analog functionality on PE0
  //timer/adc config
	DisableInterrupts();
  SYSCTL_RCGCADC_R |= 0x01;     // activate ADC0 
  SYSCTL_RCGCTIMER_R |= 0x04;   // activate timer0 
  delay = SYSCTL_RCGCTIMER_R;   // allow time to finish activating
  TIMER2_CTL_R = 0x00000000;    // disable timer0A during setup
  TIMER2_CTL_R |= 0x00000020;   // enable timer0A trigger to ADC
  TIMER2_CFG_R = 0;             // configure for 32-bit timer mode
  TIMER2_TAMR_R = 0x00000002;   // configure for periodic mode, default down-count settings
  TIMER2_TAPR_R = 0;            // prescale value for trigger
  TIMER2_TAILR_R = (80000000/fs)-1;        // start value for trigger
  TIMER2_IMR_R = 0x00000000;    // disable all interrupts
  TIMER2_CTL_R |= TIMER_CTL_TAEN + TIMER_CTL_TASTALL + TIMER_CTL_TBSTALL;;   // enable timer0A 32-b, periodic, no interrupts
	//adc config
  ADC0_PC_R = 0x01;         // configure for 125K samples/sec
  ADC0_SSPRI_R = 0x0123;    // sequencer 3 is highest, sequencer 0 is lowest
  ADC0_ACTSS_R &= ~0x04;    // disable sample sequencer 2
  ADC0_EMUX_R = (ADC0_EMUX_R&0xFFFFF0FF)+0x0500; // timer trigger event
  ADC0_SSMUX2_R |= 0x3210;
  ADC0_SSCTL2_R = 0x6000;          // set flag and end    
	ADC0_ACTSS_R |= 0x04;          // enable sample sequencer 2
  ADC0_IM_R |= 0x04;             // enable SS0 interrupts
  NVIC_PRI4_R = (NVIC_PRI4_R&0xFFFFFF00)|0x00000040; //priority 2 -. ADC0 Seq 2
  NVIC_EN0_R = 1<<16;              // enable interrupt 16 in NVIC -> ADC0 Seq 2

	return 1;

}

/* ADC0Seq3_Handler
* Function: Calls callback on sequencer 3 completion to service ADC data
*/
IR_Data_Type PEdata;
void ADC0Seq2_Handler(void){
  ADC0_ISC_R = 0x04;          // acknowledge ADC sequence 3 completion
	PEdata.BottomLeft = ADC0_SSFIFO2_R;
	PEdata.TopLeft = ADC0_SSFIFO2_R;
	PEdata.TopRight = ADC0_SSFIFO2_R;
	PEdata.BottomRight = ADC0_SSFIFO2_R;
	ProducerTask(PEdata);
}

int ADC_Status(void) {
	//TODO?
	return 1;
}

