#ifndef __QUEUE_H_
#define __QUEUE_H_

#ifndef __cplusplus
typedef unsigned char bool;				// C does not define type bool
#define true 1							//	or constants true and false
#define false 0
#endif

#define uchar unsigned char

// Define a base queue type for serial transmit and receive (charger and BMUs)
// Always derive a type from this, so that the buffer size gets set automatically
class queue {
	unsigned char rd;					// Read index
    unsigned char wr;					// Write index
	unsigned char bufSize;				// Buffer size must be a power of 2
    unsigned char* buf;					// Ptr to circular buffer (size determined when initialised)
public:
	queue(unsigned char sz);			// Constructor has to supply buffer size

	// Enqueue a byte. Returns true on success (queue not full)
	bool enqueue(unsigned char ch );	// ch is the byte to enqueue

	// Dequeue a byte. Returns true on success (queue was not empty).
	bool dequeue(unsigned char& ch );	// ch is a reference to the char to be read to

	unsigned char queue_space();			// Amount of space in the queue.
	bool empty() {return rd == wr;}		// True if the queue is empty. Defining it here makes it inline.
};

#endif		// ifdef __QUEUE_H_