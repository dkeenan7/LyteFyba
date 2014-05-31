# makfile configuration
NAME            = tri86
OBJECTS         = ${NAME}.o usci.o can.o gauge.o pedal.o bms.o charger.o voice.o queue.o pid.o
CPU             = msp430f247

ASFLAGS         = -mmcu=${CPU} -x assembler-with-cpp -D_GNU_ASSEMBLER_ -c
CFLAGS          = -mmcu=${CPU} -D__MSP430F147__ -Wall -Wconversion -Wsign-conversion -g -O2
#CFLAGS          = -mmcu=${CPU} -Wall -Wconversion -Wsign-conversion -g -O0

#switch the compiler (for the internal make rules)
CC              = msp430-gcc
AS              = msp430-gcc

.PHONY: all FORCE clean download download-jtag download-bsl dist

#all should be the first target. it's built when make is run without args
all: ${NAME}.elf ${NAME}.hex ${NAME}.lst ${NAME}.hex

#additional rules for files
${NAME}.elf: ${OBJECTS}
	${CC} -mmcu=${CPU} -o $@ ${OBJECTS}

# MVE: Hex file was called .a43 (!?)
${NAME}.hex: ${NAME}.elf
	msp430-objcopy -O ihex $^ $@

${NAME}.lst: ${NAME}.elf
	msp430-objdump -dSt $^ >$@

%.s43: %.c %.h
	${CC} -S -fverbose-asm -mmcu=${CPU} $< -o $@	# -fverbose-asm adds clues to data destination

# Use C++ now: .o files depend on .cc files
%.o: %.cc
	${CC} ${CFLAGS} -c -o $@ $<

# Dependencies: use "make depend"


clean:
	rm -f ${NAME}.elf ${NAME}.hex ${NAME}.lst ${OBJECTS}



#automatic collection of dependencies in the source files.
#it's only updated the first time, after that it must be done maually
#with "make depend"
#the dependecies are included from a separate file:
-include dependencies.in
#target to update the file, it's removed first
# MVE: Don't do as below; the order of the prerequisites is not guaranteed! For now, copy the actions here:
#depend: rmdepend dependencies.in
depend:
	rm -f dependencies.in
	$(CC) -MM ${CFLAGS} $(patsubst %.o,%.cc,$(OBJECTS)) >dependencies.in

#remove the file
rmdepend:
	rm -f dependencies.in

#build the file that contains the dependencies. no deps in this rule.
#if there were deps it would be rebuilt every chnage, which is unneded:
dependencies.in:
	$(CC) -MM ${CFLAGS} $(patsubst %.o,%.c,$(OBJECTS)) >$@



