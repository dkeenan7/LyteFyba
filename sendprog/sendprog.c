/*
 * SendProg: send a program over the BMS serial system to all CMUs
 * Sends main program only
 * Updated 8/Feb/2017 for trunk password
 */

#define LINUX 0

/* Usage: sengprog path/to/binary COM16 */

#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>		// For usleep()
#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#if !LINUX
#include "windows.h"
#endif

unsigned int address = 0;
unsigned char sum;

unsigned int readHexNibble(FILE* f) {
	char c = fgetc(f);
	int r;
	if (c >= '0') {
		r = c - '0';
		if (r > 9)
			r -= 'A'-1-'9';
		if (r <= 0xF)
			return r;
	}
	fprintf(stderr, "Unexpected char '%c' when reading hex, address = %X\n", c, address);
	exit(1);
	return -1;
}

unsigned int readHexByte(FILE* f) {
	unsigned int r = (readHexNibble(f) << 4) + readHexNibble(f);
	sum += r;
	return r;
}

unsigned int readHexWord(FILE* f) {
	return (readHexByte(f) << 8) + readHexByte(f);
}


void readColon(FILE *f) {
	char c;
	do {
		c = fgetc(f);
	} while (!feof(f) && c != ':');

	if (feof(f)) {
		fprintf(stderr, "End of file waiting for colon; address is %X\n", address);
	}
}


#if LINUX
struct termios  config;				/* UNIX only */
int fd;
void writeByte(const char* p) {
	write(fd, p, 1);
}
#else		// Windows
HANDLE hComm;

void writeByte(const char* p) {
	OVERLAPPED osWrite = {0};

	// Create this write operation's OVERLAPPED structure's hEvent.
	osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
	if (osWrite.hEvent == NULL) {
		fprintf(stderr, "Error creating overlapped event handle\n");
		exit(1);
	}

	// Issue write.
	if (!WriteFile(hComm, p, 1, NULL, &osWrite)) {
 		if (GetLastError() != ERROR_IO_PENDING) {
        	fprintf(stderr, "WriteFile to comm port failed, but isn't delayed.\n");
		 	exit(1);
    	}
	}
	CloseHandle(osWrite.hEvent);
}
#endif

void usage() {
	fprintf(stderr, "Usage: sendprog <binfile> <comm port name/path> [passwordtype]\n");
	fprintf(stderr, "Optional passwordtype is 1 (default, for trunk) or 2 (for Rev 61)\n");
	exit(1);
}

int main(int argc, char* argv[]) {
	FILE* f;
	char c;
	unsigned int len;
	unsigned int add;
	unsigned int typ;
	unsigned int u;
	unsigned char* pBuf;			/* Pointer to program buffer */ 
	unsigned char* p = NULL;
	unsigned int sum, checksum, total_len;
	char lastPassChar = 1;

	if ((argc != 3) && (argc != 4)) {
		usage();}
	if (argc == 4) {
		lastPassChar = (char) atoi(argv[3]);
		if ((lastPassChar != 1) && (lastPassChar != 2))
			usage();
	}

#if LINUX
	fd = open(argv[2], O_RDWR | O_NOCTTY | O_NDELAY );
	if(!isatty(fd)) { printf("Error - not a tty!\n"); exit(1); }
	if(tcgetattr(fd, &config) < 0) {
		printf("Error - getattr failed\n"); exit(1);
	}
	//
	// Input flags - Turn off input processing
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
						INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	config.c_oflag = 0;
	//
	// No line processing:
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	//
	// Turn off character processing
	// clear current char size mask, no parity checking,
	// no output processing, force 8 bit input
	//
	config.c_cflag &= ~(CSIZE | PARENB);
	config.c_cflag |= CS8;
	//
	// One input byte is enough to return from read()
	// Inter-character timer off
	//
	config.c_cc[VMIN]  = 1;
	config.c_cc[VTIME] = 0;
	//
	// Communication speed (simple version, using the predefined
	// constants)
	//
	if(cfsetispeed(&config, B9600) < 0 || cfsetospeed(&config, B9600) < 0) {
		printf("Error - can't set baud rate to 9600\n");
		exit(1);
	}
	//
	// Finally, apply the configuration
	//
	if(tcsetattr(fd, TCSAFLUSH, &config) < 0) {
		printf("Error - could not set configuration\n");
		exit(1);
	}
#else
	{
		char sName[32];
		COMMCONFIG  lpCC;
		sprintf(sName, "\\\\.\\%s", argv[2]);
		hComm = CreateFile(sName,
                    GENERIC_READ | GENERIC_WRITE,
                    0,
                    0,
                    OPEN_EXISTING,
                    FILE_ATTRIBUTE_NORMAL,
                    0);
		if (hComm == INVALID_HANDLE_VALUE) {
  			fprintf(stderr, "Error opening port %s\n", argv[2]);
			exit(1);
		}
		GetCommState( hComm, &lpCC.dcb);

		 /* Initialisation of parameters */
		  lpCC.dcb.BaudRate = CBR_9600;
		//lpCC.dcb.BaudRate = CBR_19200;
		lpCC.dcb.ByteSize = 8;
		lpCC.dcb.StopBits = ONESTOPBIT;
		lpCC.dcb.Parity = NOPARITY;
		lpCC.dcb.fDtrControl = DTR_CONTROL_DISABLE;
		lpCC.dcb.fRtsControl = RTS_CONTROL_DISABLE;
		SetCommState(hComm, &lpCC.dcb );

	}

#endif


	f = fopen(argv[1], "r");
	if (f == NULL) {
		fprintf(stderr, "Could not open %s for reading\n", argv[1]);
		exit(1);
	}

	struct stat st;
	stat(argv[1], &st);
	total_len = st.st_size;
	if ((pBuf = malloc(total_len)) == NULL) {
		fprintf(stderr, "Could not allocate %d bytes for program buffer\n", total_len);
		exit(2);
	}
	fread(pBuf, 1, total_len, f);
	printf("Read %d bytes\n", total_len);
	fclose(f);

	unsigned short uReset;
	unsigned int lenToSend, BSLlen;
	uReset = pBuf[total_len-1]; 
	uReset = (uReset << 8) + pBuf[total_len-2];
	if (uReset == 0xFC00)
		BSLlen = 1024;
	else
		BSLlen = 512;
	lenToSend = total_len - BSLlen;


	/* Now send this image */
	{
	int i, j, k;

	/* Write the prefix */
#define PASSLEN (1+4)
	char* pfx = "\x1B\x05\x04\x03\x01\x00";	/* ESC 05 04 03 01 */
	pfx[4] = lastPassChar;
	for (i=0; i < PASSLEN; ++i) {
		writeByte(pfx+i);					/* Write prefix */
		usleep(2000+100);					/* Time to transmit byte to CMU, and for it to echo to next CMU */
											/* Plus 100 us for safety */
	}

	// Extra 2 second delay in case it's monolith, and it is busy sending data to the PIP inverter
	usleep(2000000);

    /* Allow time for segment erases (approximately 15 ms per segment) */
	/* Note that m_len_to_send is sometimes 2 short of the real length, because of the reset vector */
	/* Be conservative and use 21 ms per segment erase */
	usleep(1000 * ((((lenToSend + 2) / 512) * 21) +1));
	
	/* Send the lenToSend-1 bytes of the binary image */
	sum = 0;
	for (u=0; u < lenToSend-1; ++u) {
		if ((u & 0x7F) == 0x7F)
		{
			printf(".");
			fflush(stdout);
		}
		writeByte(pBuf+u);					/* Write byte */
		sum ^= pBuf[u];
    	usleep(3000);						/* Time to transmit, echo, and flash write */
	}
	}

	// Finally send the checksum byte in place of the very last byte (just before the BSL)
	writeByte((char*)&sum);

#if LINUX
	close(fd);
#else
	CloseHandle(hComm);
#endif
	printf("Done\n");
	return 0;
}
