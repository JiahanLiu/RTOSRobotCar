// IDPriorityQueue.c
// PriorityQueue for ThreadID's OS
// TA: Daniel Leach

#include "../LiuWareTM4C123Lab3/IDQueue.h"
#include "../LiuWareTM4C123Lab3/OS.h"

//---------------- Private Queue Vars-------------------
unsigned long idQueue[NUMTHREADSPLUSONE];
int idQueueHead = 0;
int idQueueTail = 0; //bug resolve -> Abstract that tail must always point to empty
/* bug resolve 
* imagine that queue was only length 'NUMTHREADS' and queue was full, then head == tail even though it is not empty
*/

int idQueuePop() {
	if(idQueueHead == idQueueTail) { //empty
		return -1;
	}
	int oldHead = idQueueHead;
	idQueueHead = (idQueueHead + 1) % (NUMTHREADSPLUSONE); 
	return idQueue[oldHead]; 
}	

int idQueuePush(unsigned long input) {
	if( (idQueueTail + 1) % (NUMTHREADSPLUSONE) == idQueueHead) { //full
		return -1;
	}
	idQueue[idQueueTail] = input;
	idQueueTail = (idQueueTail + 1) % (NUMTHREADSPLUSONE); //add at 0 then increment
	return 1; 
}
