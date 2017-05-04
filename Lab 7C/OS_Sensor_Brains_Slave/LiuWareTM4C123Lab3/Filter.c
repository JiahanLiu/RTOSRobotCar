// Filter.c
// PriorityQueue for ThreadID's OS
// TA: Daniel Leach

#include "../LiuWareTM4C123Lab3/Filter.h"
#include <stdint.h>

int Median(int u1,int u2,int u3) { 
  if(u1>u2) {
    if(u2>u3) return u2;   // u1>u2,u2>u3       u1>u2>u3
    if(u1>u3) return u3;   // u1>u2,u3>u2,u1>u3 u1>u3>u2
    return u1;             // u1>u2,u3>u2,u3>u1 u3>u1>u2
  } else { 
    if(u3>u2) return u2;   // u2>u1,u3>u2       u3>u2>u1
    if(u1>u3) return u1;   // u2>u1,u2>u3,u1>u3 u2>u1>u3
    return u3;             // u2>u1,u2>u3,u3>u1 u2>u3>u1
	}
}

int Median9(int u[9]) { 
  uint8_t m1 = Median(u[0], u[1], u[2]); 
	uint8_t m2 = Median(u[3], u[4], u[5]);
	uint8_t m3 = Median(u[6], u[7], u[8]);
	return Median(m1, m2, m3);
}


