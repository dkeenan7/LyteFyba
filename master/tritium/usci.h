/*
 * Tritium MSP430 2xx USCI SPI interface header file
 * Copyright (c) 2009, Tritium Pty Ltd.  All rights reserved.
 *
 * Last Modified: J.Kennedy, Tritium Pty Ltd, 30 September 2009
 * Then by Mike Van Emmerik for dual serial ports
 *
 */

// Public Function prototypes
extern 	void 			usci_init( unsigned char clock );
extern 	void			usci_transmit( unsigned char data );
extern 	unsigned char 	usci_exchange( unsigned char data );

extern	bool			chgr_sendPacket(const unsigned char* ptr);
extern	bool			chgr_resendLastPacket( void );
extern	bool			bmu_sendPacket(const unsigned char* ptr);
extern	bool			bmu_resendLastPacket( void );

// Byte-level transmit and receive functions
bool bmu_getByte(unsigned char* chp);
bool bmu_sendByte(unsigned char ch);
bool chgr_getByte(unsigned char* chp);
bool chgr_sendByte(unsigned char ch);

// Private Function prototypes

// Public variables
volatile unsigned int chgr_sent_timeout;
volatile unsigned int bmu_sent_timeout;

