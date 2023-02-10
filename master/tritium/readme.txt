TRI86 EV driver controls MSP430 firmware README

- This firmware is written to use the GNU GCC toolchain, available from: http://mspgcc.sourceforge.net/
- Refer to the board schematics, available on the Tritium website, for pinouts and device functionality.
- The firmware is licenced using the BSD licence.  There is no obligation to publish modifications.
- If you've written something cool and would like to share, please let us know about it!
- Contact James Kennedy with any questions or comments: james@tritium.com.au


Toolchain walkthrough (Windows MSPGCC instructions):

- Download and install the MSP430 GCC toolchain
	- This will automatically set up command-prompt paths and other items
- Unzip the tri86 firmware files into your working directly
- From a command prompt window (DOS box), and in your working directory, type "make" without the quotes
- The tools will produce a file called tri86.elf, containing the device firmware and debug info
- Connect to the TRI86 driver controls hardware with a TI programmaing adapter
	- Use the supplied 14 to 8 pin adapter for the connection from a standard TI programmer
	- Use either a parallel port or a USB programmer (USB is faster and easier to set up, TI part: USB-FET430UIF)
	- Observe connector polarity (Pin 1 has an arrow pointing to it in the silkscreen overlay)
- Provide 12V DC on the spade terminal connections to the driver controls
- At the command prompt window, type "msp430-jtag -l TIUSB -e tri86.elf" without the quotes
	- The download will begin and re-flash the firmware into the MSP430 microcontroller
	- Omit the "-l TIUSB" if using a parallel port programmer


The above is the original readme that came from Tritium.

You will need a version of "make" for Windows. One way is to install Cygwin in Windows, then install make in Cygwin. Accept the default location for Cygwin C:\cygwin64\bin

And you will need to download and install either:

mspgcc-20120406-p20120911
https://sourceforge.net/projects/mspgcc/files/Windows/mingw32/

Add "C:\mspgcc-20120406-p20120911\bin" to the Windows system variable "Path".

or (not tried successfully):

The TI MSP430-GCC toolchain from:
https://www.ti.com/tool/MSP430-GCC-OPENSOURCE
Accept the default install location C:\ti\msp430-gcc\bin

Add "C:\cygwin64\bin" and "C:\ti\msp430-gcc\bin" to the Windows system variable "Path" and edit tritium/makefile if necessary so that the variables CC and AS have the value msp430-elf-gcc (plus options) and not msp430-gcc.


Then after installing one of the above, and setting the path appropriately, open a Command prompt or Cygwin prompt and cd your way to the folder holding the master/tritium sources. This may look something like:

cd C:\Users\dkeen\OneDrive\OneDriveDocuments\EV\LyteFyba\branches\rev58\master\tritium

Once you're there, type:

make

to build the hex file from the sources.