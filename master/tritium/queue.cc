// Queue related functions
#include <stdlib.h>		// Should include alloca() but doesn't
#include "queue.h"

queue::queue(unsigned char sz)
{
	rd = 0;
	wr = 0;
	bufSize = sz;
	buf = (unsigned char*)__builtin_malloc(sizeof(unsigned char[sz]));	// buf = new unsigned char[sz];
}

/* Enqueue and Dequeue general queueing functions */
// Enqueue a byte. Returns true on success (queue not full)
bool queue::enqueue(unsigned char ch )	// ch: the byte to enqueue
{
	unsigned char wr_copy = wr;			// Make a copy of the write index
	buf[wr_copy++] = ch;				// Tentatively write the byte to the queue; there is always
										//	one free space, but don't update write index yet
										// Also increments the index copy
	wr_copy &= (bufSize-1);				//	modulo the buffer size
	if (wr_copy == rd)					// Does the incremented write pointer equal the read pointer?
		return false;					// Yes means queue is full, error return
	wr = wr_copy;						// Update write pointer; byte is officially in the queue now
	return true;						// Normal return
}

// Dequeue a byte. Returns true on success (queue was not empty).
bool queue::dequeue(unsigned char& ch )	// ch: reference to the char to be read to
{
	unsigned char rd_copy = rd;			// Make a copy of the read index
	if (wr == rd_copy)					// Indexes equal?
		return false;					// If so, buffer is empty
	ch = buf[rd_copy++];				// Read the byte, increment read index
	rd_copy &= (bufSize-1);				//	modulo the buffer size
	rd = rd_copy;						// Atomic update
	return true;
}

// Amouunt of space in the queue. This is the capacity of the queue minus the number already in the queue.
// The capacity is actually bufSize-1, so space = (bufSize-1 - (wr - rd)) & bufSize-1, which is the same
// as (rd - wr - 1) & (bufSize-1)
unsigned int queue::queue_space() {
	return (rd - wr - 1) & (bufSize-1);
}


