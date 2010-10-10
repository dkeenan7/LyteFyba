This is the source code for the cell-top Battery Monitoring Unit (BMU)
of Mike Van Emmerik and Dave Keenan ("Coulomb" and "Weber" in the Australian Electric Vehicle
Association forum). See http://http://www.aeva.asn.au/forums/forum_posts.asp?TID=980

These BMUs use a Texas Instruments MSP430F2012 microcontroller
(16-bit Ultra-Low-Power, 2kB Flash, 128B RAM, 10-Bit SAR A/D).
http://focus.ti.com/docs/prod/folders/print/msp430f2012.html

To build them you will need to install the free IAR Embedded Workbench Kickstart.
See http://focus.ti.com/docs/toolsw/folders/print/iar-kickstart.html

There are four different BMU programs (IAR projects in celltopbmu.eww), only one of which may
be loaded into the BMU at any given time. They are in two broad categories.
There is the software that is loaded in normal operation, and there is the software that is
only loaded in order to load a new version of the bootstrap loader, or to perform calibration
or debugging.

The folders within the celltopbmu folder are:

Normal BMU software:
	monitor
		Current. Uses interrupt-driven buffered full-duplex serial comms.
	interpreter
		Obsolete, but contains a nifty Wunth (single-character-Forth) interpreter.
	
Special BMU software:
	debugger
		Current. Updates the bootstrap loader (BSL), allows calibration and debugging.
		Adds command interpreting to BSLwriter.
	BSLwriter
		Obsolescent, but may be required (via JTAG) if debugger can't be used serially.
		
Common BMU source files:
	common
		Not a BMU program, but code that is common to more than one of the above.

Host software:
	BMUsend
		Windows software. To load new programs into the BMUs via the serial port and our
		bootstrap loader (BSL), as opposed to using the JTAG port. Requires Microsoft
		Visual Studio.
	sendprog
		Linux or Windows/Cygwin software. To load new programs into the BMUs via the serial
		port and our Bootstrap loader (BSL), as opposed to using the JTAG port.
	Only one of BMUsend or sendprog is needed; they do the same job. sendprog is command line
		based; BMUsend is GUI.
	web
		A set of web pages describing the BMUs and printed-circuit artwork.
