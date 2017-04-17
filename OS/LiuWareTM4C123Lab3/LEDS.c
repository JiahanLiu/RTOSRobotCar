// LEDS.c
// LEDS for debugging
// TA: Daniel Leach

#include "../LiuWareTM4C123Lab3/LEDS.h"
#include <stdint.h>

// Subroutine to wait 10 msec
// Inputs: None
// Outputs: None
void DelayWait10us(uint32_t n){
	for(int i = 0; i < 128; i++){}
	/*uint32_t volatile time;
  while(n){
    time = 72724*2/91;  // 10msec
    while(time){
      time--;
    }
    n--;
  }*/
}

// Subroutine to wait 10 msec
// Inputs: None
// Outputs: None
void DelayWait10ms(uint32_t n){
	/*uint32_t volatile time;
  while(n){
    time = 72724*2/91;  // 10msec
    while(time){
      time--;
    }
    n--;
  }*/
}