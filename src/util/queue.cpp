
#include <stdint.h>
#include <string.h>

#include "../util/queue.h"

void queueInit(struct queue* queue, uint16_t maxSize, uint8_t entrySize, void* data)
{
  queue->maxSize = maxSize;
  queue->entrySize = entrySize;
  queue->head = 0;
  queue->tail = 0;
  queue->data = data;
  return;
}

uint16_t queueSize(struct queue* queue)
{
  uint16_t size;
  if (queue->head >= queue->tail) {
    size = queue->head - queue->tail;
  } else {
    size = queue->maxSize * 2 - queue->tail + queue->head;
  }
  return size;
}

int queuePush(struct queue* queue, void* entry)
{
  uint8_t* queueData = (uint8_t*) (queue->data);
  if (queueSize(queue) == queue->maxSize) {
    return QUEUE_FULL;
  }
  
  uint8_t headIdx = ((queue->head) % queue->maxSize) * queue->entrySize;
  memcpy(&(queueData[headIdx]), entry, queue->entrySize);
  queue->head = (queue->head + 1) % (queue->maxSize * 2);
  return QUEUE_OK;
}

int queuePop(struct queue* queue, void* entry)
{
  uint8_t* queueData = (uint8_t*) (queue->data);
  if (queueSize(queue) == 0) {
    return QUEUE_EMPTY;
  }
  
  uint8_t tailIdx = ((queue->tail) % queue->maxSize) * queue->entrySize;
  memcpy(entry, &(queueData[tailIdx]), queue->entrySize);
  queue->tail = (queue->tail + 1) % (queue->maxSize * 2);
  return QUEUE_OK;
}