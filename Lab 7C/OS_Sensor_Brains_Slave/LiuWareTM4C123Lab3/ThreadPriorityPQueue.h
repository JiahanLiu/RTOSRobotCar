// PriorityPriorityQueue.h
// PriorityQueue for Priority's in OS
// TA: Daniel Leach
#ifndef __THREADPRIORITYPQUEUE_H
#define __THREADPRIORITYPQUEUE_H  1

#include "../LiuWareTM4C123Lab3/OS.h"  

tcbType * priQueuePop(void);

/* priQueueRemove 
* input: pointer to tcbs entry that is wished to be removed
*/
int priQueueRemove(tcbType *input);

int priQueuePush(tcbType *input);

tcbType* priQueuePeek(void);

tcbType* priQueuePeekAndRotate(void);

#endif
