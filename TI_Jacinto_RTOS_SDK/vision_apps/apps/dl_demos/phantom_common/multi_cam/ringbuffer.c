#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <pthread.h>

#include "app_common.h"

#include "ringbuffer.h"

RingBuffer* RingBuffer_create(size_t capacity, size_t itemSize)
{
  if (capacity == 0 || itemSize == 0)
  {
    return NULL; // Invalid parameters
  }

  RingBuffer* buffer = (RingBuffer*)malloc(sizeof(RingBuffer));
  if (buffer == NULL)
  {
    return NULL; // Memory allocation failed
  }

  buffer->storage = malloc(capacity * itemSize);
  if (buffer->storage == NULL)
  {
    free(buffer); // Clean up previously allocated memory
    return NULL; // Memory allocation failed
  }

  buffer->capacity = capacity;
  buffer->itemSize = itemSize;
  buffer->head = 0;
  buffer->pre_head = 0;
  buffer->tail = 0;
  buffer->size = 0;
  buffer->pre_size = 0;
  buffer->stopped = false;
  if (pthread_mutex_init(&buffer->mutex, NULL) != 0)
  {
    free(buffer->storage);
    free(buffer);
    return NULL; // Error initializing mutex
  }
  if (pthread_cond_init(&buffer->notFull, NULL) != 0)
  {
    pthread_mutex_destroy(&buffer->mutex);
    free(buffer->storage);
    free(buffer);
    return NULL;
  }
  if (pthread_cond_init(&buffer->notEmpty, NULL) != 0)
  {
    pthread_cond_destroy(&buffer->notFull);
    pthread_mutex_destroy(&buffer->mutex);
    free(buffer->storage);
    free(buffer);
    return NULL;
  }
  return buffer;
}

int RingBuffer_enqueue(RingBuffer* buffer)
{
  if (buffer == NULL)
  {
    return RINGBUFFER_FAILURE; // Error: Invalid buffer pointer
  }
  pthread_mutex_lock(&buffer->mutex); // Lock the mutex before modifying shared data
  while ((buffer->size == buffer->capacity) && (!buffer->stopped))
  {
    // Wait until the buffer is not full
    pthread_cond_wait(&buffer->notFull, &buffer->mutex);
  }

  if (buffer->stopped)
  {
    pthread_mutex_unlock(&buffer->mutex);
    return RINGBUFFER_STOPPED;
  }

  if (buffer->size > buffer->capacity)
  {
    printf("Invalid size encountered in RingBuffer_enqueue\n");
    pthread_mutex_unlock(&buffer->mutex); // Unlock the mutex
    return RINGBUFFER_FAILURE;
  }

  buffer->tail = (buffer->tail + 1) % buffer->capacity; // Wrap around if necessary
  buffer->size++;
  pthread_cond_signal(&buffer->notEmpty); // Signal that the buffer is not empty
  pthread_mutex_unlock(&buffer->mutex); // Unlock the mutex after modifying shared data
  return RINGBUFFER_SUCCESS;
}

int RingBuffer_dequeue(RingBuffer* buffer, void** element)
{
  int status = RINGBUFFER_SUCCESS;
  if (buffer == NULL || element == NULL)
  {
    return RINGBUFFER_FAILURE; // Error: Invalid buffer or element pointer
  }
  pthread_mutex_lock(&buffer->mutex); // Lock the mutex
  while ((buffer->size - buffer->pre_size == 0) && (!buffer->stopped))
  {
    // Wait until the buffer is not empty
    pthread_cond_wait(&buffer->notEmpty, &buffer->mutex);
  }

  if (buffer->stopped)
  {
    pthread_mutex_unlock(&buffer->mutex);
    return RINGBUFFER_STOPPED;
  }

  if (buffer->pre_size > buffer->size)
  {
    printf("Invalid size encountered in RingBuffer_dequeue\n");
    pthread_mutex_unlock(&buffer->mutex); // Unlock the mutex
    return RINGBUFFER_FAILURE;
  }

  // Calculate the memory address to read from
  *element = (char*)buffer->storage + buffer->pre_head * buffer->itemSize;

  // Update prefetch pointer
  buffer->pre_head = (buffer->pre_head + 1) % buffer->capacity; // Wrap around if necessary
  buffer->pre_size++;

  pthread_mutex_unlock(&buffer->mutex); // Unlock the mutex
  return status;
}

int RingBuffer_releaseOldest(RingBuffer* buffer)
{
  if (buffer == NULL)
  {
    return RINGBUFFER_FAILURE; // Error: Invalid buffer
  }
  pthread_mutex_lock(&buffer->mutex); // Lock the mutex
  /* 
   * We already know the buffer is not empty from earlier dequeue call,
   * so there is no need to call pthread_cond_wait here.
   */

  if (buffer->size == 0) {
    printf("Invalid size encountered in RingBuffer_releaseOldest\n");
    pthread_mutex_unlock(&buffer->mutex); // Unlock the mutex
    return RINGBUFFER_FAILURE;
  }

  // Update pointers
  if (buffer->pre_head == buffer->head)
  {
    buffer->pre_head = (buffer->pre_head + 1) % buffer->capacity;
    buffer->pre_size = 0;
  }
  else
  {
    buffer->pre_size--;
  }
  buffer->head = (buffer->head + 1) % buffer->capacity; // Wrap around if necessary
  buffer->size--;
  pthread_cond_signal(&buffer->notFull); // Signal that the buffer is not full
  pthread_mutex_unlock(&buffer->mutex); // Unlock the mutex
  return RINGBUFFER_SUCCESS;
}

int RingBuffer_waitForNotFull(RingBuffer* buffer)
{
  int status = RINGBUFFER_SUCCESS;
  if (buffer == NULL)
  {
    return RINGBUFFER_FAILURE; // Error: Invalid buffer pointer
  }
  pthread_mutex_lock(&buffer->mutex);
  while ((buffer->size == buffer->capacity) && (!buffer->stopped))
  {
    // Wait until the buffer is not full
    pthread_cond_wait(&buffer->notFull, &buffer->mutex);
  }

  if (buffer->stopped)
  {
    status = RINGBUFFER_STOPPED;
  }

  pthread_mutex_unlock(&buffer->mutex);
  return status;
}

void* RingBuffer_getStorage(const RingBuffer* buffer)
{
  if (buffer == NULL)
  {
    return NULL;
  }
  return buffer->storage;
}

void RingBuffer_stop(RingBuffer* buffer)
{
  if (buffer != NULL)
  {
    pthread_mutex_lock(&buffer->mutex);
    buffer->stopped = true;
    pthread_cond_broadcast(&buffer->notEmpty);
    pthread_cond_broadcast(&buffer->notFull);
    pthread_mutex_unlock(&buffer->mutex); // Unlock the mutex
  }
}

void RingBuffer_destroy(RingBuffer* buffer)
{
  if (buffer != NULL)
  {
    pthread_cond_destroy(&buffer->notFull);  // Destroy the condition variables
    pthread_cond_destroy(&buffer->notEmpty);
    pthread_mutex_destroy(&buffer->mutex); // Destroy the mutex
    if (buffer->storage != NULL)
    {
      free(buffer->storage); // Free the buffer's data
    }
    free(buffer); // Free the RingBuffer struct itself
  }
}
