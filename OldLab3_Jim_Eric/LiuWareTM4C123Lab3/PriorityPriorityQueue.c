// PriorityPriorityQueue.c
// PriorityQueue for Priority's in OS
// TA: Daniel Leach

#include "../LiuWareTM4C123Lab3/PriorityPriorityQueue.h" 
#include "../LiuWareTM4C123Lab3/OS.h"  

//---------------- PriorityQueue Helper Functions-------------------
void priQueuePushBack(int index);
void priQueueShiftForward(int index);
//---------------- PriorityQueue Vars-------------------
tcbType *priQueue[NUMTHREADSPLUSONE];
int priQueueHead = 0;
int priQueueTail = 0; //points to empty spot
int numPriItems = 0;

tcbType * priQueuePop() {
	if(priQueueHead == priQueueTail) {
		return NULL;
	}
	int oldHead = priQueueHead;
	priQueueHead = (priQueueHead + 1) % (NUMTHREADSPLUSONE); 
	numPriItems--;
	return priQueue[oldHead]; 
}	

/* priQueueRemove 
* input: pointer to tcbs entry that is wished to be removed
*/
int priQueueRemove(tcbType *input) {
	if(priQueueHead == priQueueTail) { //empty
		return 0;
	}
	int currentIndex = priQueueHead;
	while(priQueue[currentIndex] != input) { //loop to item to be removed
		currentIndex = (currentIndex + 1) % NUMTHREADSPLUSONE;
	}
	while(currentIndex != (priQueueTail + (NUMTHREADSPLUSONE - 1)) % NUMTHREADSPLUSONE ) { //-1 because of corner case that we only have 1 item 
		priQueueShiftForward(currentIndex);
		currentIndex = (currentIndex + 1) % NUMTHREADSPLUSONE; 
	}
	priQueueTail = (priQueueTail + NUMTHREADSPLUSONE - 1) % NUMTHREADSPLUSONE;
	numPriItems--;
	return 1;
}

int priQueuePush(tcbType *input) {
	if( (priQueueTail + 1) % (NUMTHREADSPLUSONE) == priQueueHead) { //checking whether we have space or not
		return -1;
	} 
	int insertLoc = priQueueTail; //tail starts at empty space
	for(int i = 1; i <= numPriItems; i++) {
		int index = (priQueueTail + NUMTHREADSPLUSONE - i) % NUMTHREADSPLUSONE;  
		if(input->priority < priQueue[index]->priority) {
			priQueuePushBack(index);
			insertLoc = index;
		}
	}
	priQueue[insertLoc] = input;
	priQueueTail = (priQueueTail + 1) % (NUMTHREADSPLUSONE); 
	numPriItems++; //if we only have 4 items before hand we only want to check through 4 items and then increment to 5 items
	return 1; 
}

tcbType* priQueuePeek() {
	if(0 == numPriItems) {
		return 
			NULL; //we want to force crash to avoid unexpected behavior
	}
	return priQueue[priQueueHead]; 
}

tcbType* priQueuePeekAndRotate() {
	if(0 == numPriItems) {
		return NULL; //we want to force crash to avoid unexpected behavior
	}
	tcbType *originalFirst = priQueue[priQueueHead]; //extract to make hole 
	int originalPriority = originalFirst->priority;
	int currentIndex = priQueueHead;
	int holeIndex = priQueueHead;
	for(int i = 0; i < numPriItems - 1; i++) {
		tcbType *next = priQueue[(currentIndex+1) % NUMTHREADSPLUSONE]; //look at next
		currentIndex = (currentIndex + 1) % NUMTHREADSPLUSONE; //look at next one
		if(next->priority == originalPriority) { //is next priority on par with origional Priority
			priQueueShiftForward(holeIndex); //
			holeIndex = currentIndex;
		}
	}
	priQueue[holeIndex] = originalFirst; 
	return originalFirst;
}

void priQueuePushBack(int index) {
	priQueue[(index + 1) % NUMTHREADSPLUSONE] = priQueue[(index)];
}

void priQueueShiftForward(int index) {
	priQueue[(index)] = priQueue[(index + 1) % NUMTHREADSPLUSONE];
}
