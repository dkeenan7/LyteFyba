/*
 * SendProg: send a program over the BMS serial system to all BMUs
 */

/* Usage: sengprog path/to/binary */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

unsigned int address = 0xF800;
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

int main(int argc, char* argv[]) {

	FILE* f;
	char c;
	unsigned int len;
	unsigned int add;
	unsigned int typ;
	unsigned int u;
	char progBuf[2048];
	char* p = progBuf;
	unsigned int sum, checksum;

	if (argc != 2) {
		fprintf(stderr, "Usage: sendprog <file.hex>\n");
		exit(1);
	}

	f = fopen(argv[1], "r");
	if (f == NULL) {
		fprintf(stderr, "Could not open %s for reading\n", argv[1]);
		exit(1);
	}

	memset(progBuf, '\xFF', 2048);

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
		if (add < address) {
			fprintf(stderr, "Unordered address %X; expected %X\n", add, address);
			exit(1);
		}
		address = add;
		p = progBuf + add - 0xF800;
		for (u=0; u < len; ++u) {
			*p++ = readHexByte(f);
		}
		checksum = readHexByte(f);
		if (sum & 0xFF) {
			fprintf(stderr, "Bad checksum %X expected %X\n", checksum, 0-(sum-checksum) & 0xFF);
			exit(1);
		}
		address += len;
	} while (!feof(f));
	
	printf("Read %d bytes\n", p - progBuf);
	if (p - progBuf != 2048)
		exit(1);

	fclose(f);
	return 0;
}

