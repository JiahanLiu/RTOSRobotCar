// JimString.c
// String Functions
// TA: Daniel Leach

#include "../LiuWareTM4C123Lab3/JimString.h"

/***************** StringToInt **************
* Input: string and length of string
* Output: int version of string
*/
int StringToInt(char * stringNum, int len) {
	int i, dec = 0; 
	for(i=0; i<len; i++){
		dec = dec * 10 + ( stringNum[i] - '0' );
	}
	return dec; 
}

/***************** strLengthByNullCount **************
* Input: string
* Output: length of string or the logical count of characters before null
*/
int strLengthByNullCount(char * stringNum) {
	int i = 0;
	while (*(stringNum + i) != NULLCHAR) {
		i++;
	}
	return i; 
}

/***************** strLengthBySpaceCount **************
* Input: string
* Output: length of word or the logical count of characters before space
*/
int strLengthBySpaceCount(char * stringNum) {
	int i = 0;
	while (*(stringNum + i) != ' ') {
		i++; 
	}
	return i; 
}
