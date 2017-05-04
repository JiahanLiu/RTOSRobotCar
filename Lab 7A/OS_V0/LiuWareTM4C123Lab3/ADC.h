// ADC.h
// Driver for ADC.
// Last modified 2/10/17 Jiahan Liu, Eric Li
// Jiahan Liu, jl57566
// Eric Li, ecl625
// TA: Daniel Leach

#ifndef __ADC_H
#define __ADC_H  1

/* ADC_Init
* Function: Init for ADC_In, single collect
*/
int ADC_Init(uint32_t channelNum);

/* ADC_In
* Function: single collect
*/
uint16_t ADC_In(void);

/* ADC_Collect
* Function: This is actually the initalization of buffered collect. 
* multiple read, data will be handled by handler 
* which will call callback function ProducerTask. 
* TODO: 1) Semaphore for multiple called
* TODO: 2) Boolean for not multiple init
*/
int ADC_Collect_Init(uint32_t channelNum, uint32_t fs, void(*pTask)(unsigned long data));

int ADC_Status(void);

#endif
