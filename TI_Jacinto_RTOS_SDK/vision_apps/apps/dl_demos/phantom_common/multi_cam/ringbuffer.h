// ringbuffer.h

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <pthread.h>

#include "app_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RINGBUFFER_SUCCESS (0)
#define RINGBUFFER_FAILURE (-1)
#define RINGBUFFER_STOPPED (1)

typedef struct {
  void* storage;    // Pointer to the buffer storage
  size_t capacity;  // Maximum number of elements the buffer can hold
  size_t itemSize;  // Size of each element in bytes
  size_t pre_head;  // Index of the next element to be prefetched
  size_t head;      // Index of the next element to be read
  size_t tail;      // Index of the next element to be written
  size_t pre_size;  // Number of prefetched elements  
  size_t size;      // Number of elements currently in the buffer
  bool stopped;     // Stopped and ready for destruction
  pthread_mutex_t mutex;   // Mutex for thread safety
  pthread_cond_t notFull;  // Condition variable for when the buffer is not full
  pthread_cond_t notEmpty; // Condition variable for when the buffer is not empty
} RingBuffer;

/**
 * Creates a new RingBuffer.
 * @param capacity The maximum number of elements the buffer can hold.
 * @param itemSize The size of each element in bytes.
 * @return A pointer to the new RingBuffer, or NULL on error.
 */
RingBuffer* RingBuffer_create(size_t capacity, size_t itemSize);

/**
 * Adds the next element to the buffer.
 * It does not actually copy anything into the buffer and only advances the tail index for bookeeping.
 * To write to the buffer's storage directly, use RingBuffer_getStorage().
 * @param buffer A pointer to the RingBuffer.
 * @return RINGBUFFER_SUCCESS on success, RINGBUFFER_STOPPED on stopped, RINGBUFFER_FAILURE on error.
 */
int RingBuffer_enqueue(RingBuffer* buffer);

/**
 * Retrieves the next element from the buffer by pre-fetching it.
 * It does not update the head index. You must call RingBuffer_releaseOldest subsequently to do so.
 * @param buffer A pointer to the RingBuffer.
 * @param element A pointer to the memory of the dequeued element.
 * @return RINGBUFFER_SUCCESS on success, RINGBUFFER_STOPPED on stopped, RINGBUFFER_FAILURE on error.
 */
int RingBuffer_dequeue(RingBuffer* buffer, void** element);

/**
 * Removes the oldest element from the buffer. Should only be called after ensuring the buffer is not
 * empty using RingBuffer_dequeue.
 * @param buffer A pointer to the RingBuffer.
 * @return RINGBUFFER_SUCCESS on success, RINGBUFFER_FAILURE on error.
 */
int RingBuffer_releaseOldest(RingBuffer* buffer);

/**
 * Wait for the buffer to not be full.
 * @param buffer A pointer to the RingBuffer.
 * @return RINGBUFFER_SUCCESS on success, RINGBUFFER_STOPPED on stopped, RINGBUFFER_FAILURE on error.
 */
int RingBuffer_waitForNotFull(RingBuffer* buffer);

/**
 * Return the storage for access
 * @param buffer A pointer to the RingBuffer.
 * @return pointer to storage on success, NULL on error.
 */
void* RingBuffer_getStorage(const RingBuffer* buffer);

/**
 * Stop a RingBuffer and prepare it to be destroyed.  It unblocks
 * any thread waiting on a RingBuffer_enqueue/dequeue/waitForNotFull call.
 * @param buffer A pointer to the RingBuffer.
 */
void RingBuffer_stop(RingBuffer* buffer);

/**
 * Frees the memory allocated for the RingBuffer.  The caller is responsible
 * for ensuring that no other code is still using the buffer.
 * @param buffer A pointer to the RingBuffer.
 */
void RingBuffer_destroy(RingBuffer* buffer);


#ifdef __cplusplus
}
#endif

#endif // RING_BUFFER_H
