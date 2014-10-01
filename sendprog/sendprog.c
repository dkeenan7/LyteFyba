/*
 * SendProg: send a program over the BMS serial system to all CMUs
 * Sends main program only
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
unsigned int sum;

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

int main(int argc, char* argv[]) {
	FILE* f;
	char c;
	unsigned int len;
	unsigned int add;
	unsigned int typ;
	unsigned int u;
	char progBuf[8192];
	char* p = progBuf;
	unsigned int sum, checksum, total_len;

	if (argc != 3) {
		fprintf(stderr, "Usage: sendprog <file.hex or file.bin> <comm port name/path>\n");
		fprintf(stderr, "File extension .hex is case sensitive\n");
		exit(1);
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

	memset(progBuf, '\xFF', 8192);

	if (strcmp(argv[1] + strlen(argv[1]) - 4, ".hex") == 0) {

		unsigned int first_addr = (unsigned int) -1;
		total_len = 0;
		do {
			readColon(f);
			sum = 0;
			len = readHexByte(f);
			add = readHexWord(f);
			typ = readHexByte(f);
			if (typ == 1)
				break;
			if (typ > 0) {
				fprintf(stderr, "Unexpected record type %X at address %X\n", typ, address);
				exit(1);
			}
			if (add < 0xE000)
				continue;			/* Repeat the loop, looking for the colon on the next line */
			if (first_addr == (unsigned int) -1)
				/* Assume that the first address past E000 is the start of the image */
				first_addr = add;
			address = add;
			p = progBuf + add - first_addr;
			for (u=0; u < len; ++u) {
				*p++ = readHexByte(f);
			}
			total_len += len;
			checksum = readHexByte(f);
			if (sum & 0xFF) {
				fprintf(stderr, "Bad checksum %X expected %X\n", checksum, 0-(sum-checksum) & 0xFF);
				exit(1);
			}
			address += len;
		} while (!feof(f));
	}

	else /* Assume a binary file */
	{
		struct stat st;
		stat(argv[1], &st);
		total_len = st.st_size;
		fread(progBuf, 1, total_len, f);
	}
	printf("Read %d bytes\n", total_len);
	fclose(f);

	/* Calculate the checksum, and place at 0xFFFD (first unused interrupt vector, starting at highest address,
		after reset */
	sum = 0;
	for (u=0; u < 0xE00-1; ++u)
		sum ^= progBuf[u];
	progBuf[0xE00-1] = sum;		/* Now it will checksum to zero */

	/* Now send this image to the CMUs */
	{
	int i, j, k;

	/* Write the prefix */
#define PASSLEN (1+4)
	char* pfx = "\x1B\x07\x06\x05\x04\x00";	/* ESC ^G ^F ^E ^D */
	for (i=0; i < PASSLEN; ++i) {
		writeByte(pfx+i);					/* Write prefix */
		usleep(2000);						/* Time to transmit byte to CMU, and for it to echo to next CMU */
	}

	/* NOTE: We are putting the delay in two positions now, because we have two versions of the BSL. Soon
		the second delay can go away */
	/* Allow extra time for bulk erase; approximately 3 characters */
	usleep(32000);					// But this is ~ 32 chars worth, could likely cut down
	/* Send the $E00-2 bytes of the binary image */
//	writeByte(progBuf);						/* Write first byte */
	/* Allow time for bulk erase; approximately 3 characters */
	usleep(32000);							/* To be safe, enough for 32 chars */
	for (u=0; u < 0xE00; ++u) {
		if ((u & 0x7F) == 0x7F)
		{
			printf(".");
			fflush(stdout);
		}
		writeByte(progBuf+u);				/* Write byte */
    	usleep(3000);						/* Time to transmit, echo, and flash write */
	}
	}

#if LINUX
	close(fd);
#else
	CloseHandle(hComm);
#endif
	printf("Done\n");
	return 0;
}
