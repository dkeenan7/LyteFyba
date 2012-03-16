#ifndef __QUEUE_H_
#define __QUEUE_H_

typedef unsigned char bool;				// C does not define type bool
#define true 1							//	or constants true and false
#define false 0

// Define a queue type for serial transmit and receive (charger and BMUs)
typedef struct {
      unsigned char rd;		 // Read index
      unsigned char wr;		 // Write index
      unsigned char bufSize; // Buffer size must be a power of 2
      unsigned char buf[];	 // Circular buffer (size determined when initialised)
} queue;


// Enqueue a byte. Returns true on success (queue not full)
bool enqueue(
			volatile queue* q,			// Pointer to the queue structure
			unsigned char ch );			// The byte to enqueue

// Dequeue a byte. Returns true on success (queue was not empty).
bool dequeue(
			volatile queue* q,			// Pointer to the queue structure
			unsigned char* ch );		// Pointer to the char to be read to


// Amouunt of space in the queue.
unsigned int queue_space( queue* q );	// Pointer to queue structure

#endif		// ifdef __QUEUE_H_