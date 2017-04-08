// LEDS.c
// LEDS for debugging
// TA: Daniel Leach

#ifndef _LEDS_H
#define _LEDS_H  1

//---------------- Debug Lights -------------------
#define LEDS      (*((volatile uint32_t *)0x40025038))
#define RED       0x02
#define BLUE      0x04
#define GREEN     0x08
#define WHEELSIZE 8           // must be an integer multiple of 2	
//const long COLORWHEEL[WHEELSIZE] = {RED, RED+GREEN, GREEN, GREEN+BLUE, BLUE, BLUE+RED, RED+GREEN+BLUE, 0};
static const long COLORWHEEL[WHEELSIZE] = {RED, RED+GREEN, GREEN, GREEN+BLUE, BLUE+RED, RED+GREEN+BLUE, 0};
static int wheelCounter;
#endif
