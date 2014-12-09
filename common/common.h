; Common definitions for monitor, TestICal and BSL

; Port 1 bit masks
PIEZO		EQU		1<<0			; Piezo on P1.0
TXDm		EQU		1<<1			; TA0/P1.1. Code requires TXDm bit somewhere to right of TXDp bit
RXD			EQU		1<<2			; Receive data on P1.2 (bit number >3 costs 2 words)
#if !REV61
ACTLED		EQU		1<<3			; Actvity LED (blue) inverted on P1.3
#endif
VREFp		EQU		1<<4			; Analog reference output. Sometimes changed to low digital output
TXDp		EQU		1<<5			; SCLK/P1.5. Can be made inverse of TA0 by hardware.
; Port 2 bit masks
BYPASS		EQU		1<<6			; Bypass transistor on P2.6
RELAYm		EQU		1<<6			; IMU relay for HazV- test on P2.6
ERRLED		EQU		1<<7			; Error LED (red) on P2.7
RELAYp		EQU		1<<7			; IMU relay for HazV+ test on P2.7


#define		WATCHDOG	1			// True if watchdog timer is to be used (only turn off for debugging)
									// Turning it off doesn't work because BSL will still clear and
									// restart the watchdog timer on every call to ReadByte.
#define		PROG_START	$E000		// Start of program image in flash memory. Ends at $FDFF
#define		BSL2_START	$FE00		// Start of BSL2 image in flash memory. Ends at $FFFD

; The address BSL2 downloads to is usually the same as PROG_START,
; but when making a transition between different download sizes, the version of TestICal that does the
; update to the new BSL2 will still need to be the old size, so it can be downloaded by the old BSL2.
#define		PROG_START_FOR_BSL		PROG_START	// Where the BSL should put the images it downloads.
//#define		PROG_START_FOR_BSL		$E000	// Would be used temporarily while changing to a BSL that
										//	loads a different sized image to the one it is contained in.
										//	The password may need changing too, in both BSL2 and Monitor.
			ORG		$10F8
			; Calibration data
DATAVERS		EQU		6			; This is version 6 of the CMU info-flash data structure
infoDataStart						; Used when copying between ram and info-flash
infoVoltCal		ds		2			; Voltage scale calibration word; may be written by BSL writer
infoTempCal		ds		1			; Temperature offset calibration; may be written by BSL writer
infoLinkCal		ds		1			; Link voltage offset calibration data; may be written by BSL writer
info8MHzCalD	ds		1			; 8 MHz DCO frequency calibration byte (same address as in new chip)
info8MHzCalB	ds		1			; 8 MHz DCO range calibration byte (same address as in new chip)
infoID			ds		1			; Cell/CMU identifier byte; first cell is 1; written by 'i' cmd
infoDataVers	ds		1			; Data Version byte (cannot move). Must be set to DATAVERS value above
; Note that xxxDataEnd is one PAST the last calibration byte, i.e. the address of the start of what
;	comes after the calibration data
infoDataEnd							; Used when copying between ram and info-flash