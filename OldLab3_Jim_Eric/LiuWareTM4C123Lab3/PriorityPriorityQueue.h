// PriorityPriorityQueue.h
// PriorityQueue for Priority's in OS
// TA: Daniel Leach

#include "../LiuWareTM4C123Lab3/OS.h"  

tcbType * priQueuePop(void);

/* priQueueRemove 
* input: pointer to tcbs entry that is wished to be removed
*/
int priQueueRemove(tcbType *input);

int priQueuePush(tcbType *input);

tcbType* priQueuePeek(void);

tcbType* priQueuePeekAndRotate(void);
