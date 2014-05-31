/*
 * Voice synthesiser interface header file
 *
 * Created 28-May-2014 by Dave Keenan
 * based on bms.h and bms.cc by Mike Van Emmerik and Dave Keenan
 */

#include "queue.h"

// Voice synth constants
#define VOICE_TX_BUFSZ		64
#define VOICE_RX_BUFSZ		64

// Public Function prototypes
void voice_init();
bool voice_sendByte(unsigned char ch);
void voice_readBytes();

// Voice synth buffers
extern queue voice_tx_q;
extern queue voice_rx_q;

