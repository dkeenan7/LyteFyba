Visit our project page at http://dkeenan7.github.io/LyteFyba/

This project contains the source code, schematics and printed circuit artwork for the 
LyteFyba Battery Monitoring System (BMS) which consists of multiple Cell Management Units (CMUs) 
and one BMS Master Unit (BMU).

It uses Texas Instruments MSP430 microcontrollers.

To build the firmware from the sources you will need to install the free IAR Embedded Workbench Kickstart.
See http://focus.ti.com/docs/toolsw/folders/print/iar-kickstart.html

There are several different CMU programs (IAR projects in celltopCMU.eww), only one of which may
be loaded into the CMU at any given time. There is software that is loaded in normal operation,
and there is the special software that is only loaded to perform testing, ID setting or calibration,
or to load a new version of the bootstrap loader.

The folders within the LyteFyba folder are:

Normal CMU software:
	monitor
		Current. Uses interrupt-driven buffered full-duplex serial comms.
	
Special CMU software:
	TestICal
		Current. Updates the bootstrap loader (BSL), allows calibration and debugging.

Common CMU source files:
	common
		Not a CMU program, but code that is common to more than one of the above.

Host software:
	CMUsend
		Windows software. To load new programs into the CMUs via the serial port and our
		bootstrap loader (BSL), as opposed to using the JTAG port. Requires Microsoft
		Visual Studio.
		Only one of CMUsend or sendprog is needed; they do the same job. sendprog is command line
		based; CMUsend is GUI.
	sendprog
		Linux or Windows/Cygwin software. To load new programs into the CMUs via the serial
		port and our Bootstrap loader (BSL), as opposed to using the JTAG port.
		Only one of CMUsend or sendprog is needed; they do the same job. sendprog is command line
		based; CMUsend is GUI.
Hardware:
	web
		A set of web pages describing the CMUs and printed-circuit artwork.
	PCB
		The printed-circuit-board artwork and schematics in DesignSpark format.
