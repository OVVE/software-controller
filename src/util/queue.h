
#ifndef __QUEUE_UTIL_H__
#define __QUEUE_UTIL_H__

#include <stdint.h>

#define QUEUE_OK    0
#define QUEUE_FULL  1
#define QUEUE_EMPTY 2


// Queue Structure
// Note: head and tail go from 0 to 2*maxSize-1 to differential between full and
//       empty queues without additional flags or wasting buffer space
struct queue {
  uint16_t maxSize;
  uint8_t entrySize;
  uint16_t head;
  uint16_t tail;
  void*   data;
};

void queueInit(struct queue* queue, uint16_t maxSize, uint8_t entrySize, void* data);

uint16_t queueSize(struct queue* queue);

int queuePush(struct queue* queue, void* entry);

int queuePop(struct queue* queue, void* entry);

#endif /* __QUEUE_UTIL_H__ */