Visit our project page at http://dkeenan7.github.io/LyteFyba/

This project contains the source code, schematics and printed circuit artwork for the 
LyteFyba Battery Monitoring System (BMS) which consists of multiple Cell Management Units (CMUs) 
and one BMS Master Unit (BMU).

It uses Texas Instruments MSP430G2553 microcontrollers.

To build the firmware from the sources you will need to install the free IAR Embedded Workbench Kickstart.
See http://focus.ti.com/docs/toolsw/folders/print/iar-kickstart.html

There are several different CMU/BMU programs (IAR projects in LyteFyba.eww), only one of which may
be loaded into the CMU or BMU at any given time. There is software that is loaded in normal operation,
and there is the special software that is only loaded to perform testing, ID setting or calibration,
or to load a new version of the bootstrap loader.

The folders within the LyteFyba folder are:

Normal Electric Vehicle software:
	monitor
		Uses interrupt-driven buffered full-duplex serial comms.
	
Normal Solar Power System software for use with PIP-4048MS inverter:
	monolith  (black monolith)
		Uses interrupt-driven buffered full-duplex serial comms.
	
Normal Solar Power System software for use with Schneider inverter:
	wmonolith  (white monolith)
		Uses interrupt-driven buffered full-duplex serial comms.
	
Special CMU software:
	TestICal
		Allows updating the bootstrap loader (BSL). Allows calibration and debugging.

Host software:
	CMUsend
		Windows software. To load new programs into the CMUs via the serial port and our
		bootstrap loader (BSL), as opposed to using the JTAG port. Requires Microsoft
		Visual Studio.
		Only one of CMUsend or sendprog is needed; they do the same job. sendprog is command
		line based; CMUsend is GUI.
	sendprog
		Linux or Windows/Cygwin software. To load new programs into the CMUs via the serial
		port and our Bootstrap loader (BSL), as opposed to using the JTAG port.
		Only one of CMUsend or sendprog is needed; they do the same job. sendprog is command
		line based; CMUsend is GUI.
Hardware:
	web
		A set of web pages describing the CMUs and printed-circuit artwork.
	PCB
		The printed-circuit-board artwork and schematics in DesignSpark format.
