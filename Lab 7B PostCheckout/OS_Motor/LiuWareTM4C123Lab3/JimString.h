// LEDS.c
// LEDS for debugging
// TA: Daniel Leach

#ifndef _JIMSTRING_H
#define _JIMSTRING_H  1

#define NULLCHAR '\0'

int StringToInt(char * stringNum, int len);

int strLengthByNullCount(char * stringNum);

int strLengthBySpaceCount(char * stringNum);

#endif
