#ifndef __QUEUE_H_
#define __QUEUE_H_

#ifndef __cplusplus
typedef unsigned char bool;				// C does not define type bool
#define true 1							//	or constants true and false
#define false 0
#endif

// Define a queue type for serial transmit and receive (charger and BMUs)
class queue {
	unsigned char rd;					// Read index
    unsigned char wr;					// Write index
    const unsigned char bufSize;		// Buffer size must be a power of 2
    unsigned char buf[0];				// Circular buffer (size determined when initialised)  
public:
	queue(unsigned char sz);			// Constructor has to supply buffer size

	// Enqueue a byte. Returns true on success (queue not full)
	bool enqueue(unsigned char ch );		// ch is the byte to enqueue

	// Dequeue a byte. Returns true on success (queue was not empty).
	bool dequeue(unsigned char& ch );		// ch is a reference to the char to be read to

	unsigned int queue_space();				// Amouunt of space in the queue.
	bool empty();					// True if the queue is empty

};

#endif		// ifdef __QUEUE_H_