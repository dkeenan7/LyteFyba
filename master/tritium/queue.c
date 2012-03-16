// Queue related functions

#include "queue.h"

/* Enqueue and Dequeue general queueing functions */
// Enqueue a byte. Returns true on success (queue not full)
bool enqueue(
			volatile queue* q,			// Pointer to the queue structure
			unsigned char ch )			// The byte to enqueue
{
	unsigned char wr_copy = q->wr;		// Make a copy of the write index
	q->buf[wr_copy++] = ch;				// Tentatively write the byte to the queue; there is always
										//	one free space, but don't update write index yet
										// Also increments the index copy
	wr_copy &= (q->bufSize-1);			//	modulo the buffer size
	if (wr_copy == q->rd)				// Does the incremented write pointer equal the read pointer?
		return false;					// Yes means queue is full, error return
	q->wr = wr_copy;					// Update write pointer; byte is officially in the queue now
	return true;						// Normal return
}

// Dequeue a byte. Returns true on success (queue was not empty).
bool dequeue(
			volatile queue* q,			// Pointer to the queue structure
			unsigned char* ch )			// Pointer to the char to be read to
{
	unsigned char rd_copy = q->rd;		// Make a copy of the read index
	if (q->wr == rd_copy)				// Indexes equal?
		return false;					// If so, buffer is empty
	*ch = q->buf[rd_copy++];			// Read the byte, increment read index
	rd_copy &= (q->bufSize-1);			//	modulo the buffer size
	q->rd = rd_copy;					// Atomic update
	return true;
}

// Amouunt of space in the queue. This is the capacity of the queue minus the number already in the queue.
// The capacity is actually bufSize-1, so space = (bufSize-1 - (wr - rd)) & bufSize-1, which is the same
// as (rd - wr - 1) & (bufSize-1)
unsigned int queue_space( queue* q )	// Pointer to queue structure
{
	return (q->rd - q->wr - 1) & (q->bufSize-1);
}

