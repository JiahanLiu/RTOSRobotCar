// ADC.h
// Driver for ADC.
// Last modified 2/10/17 Jiahan Liu, Eric Li
// Jiahan Liu, jl57566
// Eric Li, ecl625
// TA: Daniel Leach

#ifndef __ADC_H
#define __ADC_H  1

int ADC_Open(uint32_t channelNum);

uint16_t ADC_In(void);

int ADC_Collect(uint32_t channelNum, uint32_t fs,uint32_t numberOfSamples);

int ADC_Status(void);

#endif
