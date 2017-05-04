// ADC.h
// Driver for ADC.
// Last modified 2/10/17 Jiahan Liu, Eric Li
// Jiahan Liu, jl57566
// Eric Li, ecl625
// TA: Daniel Leach

#ifndef __ADC_H
#define __ADC_H  1

#include <stdint.h>
#include "OS.h"

/* ADC_Collect
* Function: This is actually the initalization of buffered collect. 
* multiple read, data will be handled by handler 
* which will call callback function ProducerTask. 
* TODO: 1) Semaphore for multiple called
* TODO: 2) Boolean for not multiple init
*/
int IR_Sensor_Init(uint32_t fs, void(*pTask)(IR_Data_Type data));

int ADC_Status(void);

#endif
