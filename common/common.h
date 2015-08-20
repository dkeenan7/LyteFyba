; Common definitions for monitor, TestICal and BSL

#if	G2553						// If using the newer MSP430G2553 processor

InitSP		EQU		$400			; Initial value of stack pointer

; Port 1 bit masks
TouchV_Byp	EQU		1<<0			; TouchV analog input (BMU) and Bypass output (CMU) on P1.0
RxCmu		EQU		1<<1			; Receive from CMUs (UART input) on P1.1
TxPlCmu		EQU		1<<2			; Transmit+ to CMUs (UART output) on P1.2
TxMiCmu		EQU		1<<3			; Transmit- to CMUs (CAOUT inversion of UART output) on P1.3
VREFp		EQU		1<<4			; Analog reference output on P1.4
ArrayV_BoltVPl EQU	1<<5			; Analog input on P1.5
ShuntV_BoltVMi EQU	1<<6			; Analog input on P1.6
BatV_StrapVPl EQU	1<<7			; Analog input on P1.7
BYPASS		EQU		TouchV_Byp		; Aliases for code that's common to newer and older devices
RXD			EQU		RxCmu
TXDp		EQU		TxPlCmu
TXDm		EQU		TxMiCmu
BypPortDIR	EQU		P1DIR			; Bypass MOSFET output on port 1
BypPortSEL	EQU		P1SEL
BypPortOUT	EQU		P1OUT

; Port 2 bit masks
RELAYm		EQU		1<<0			; IMU relay for HazV- test on P2.0
RELAYp		EQU		1<<1			; IMU relay for HazV+ test on P2.1
Spare0		EQU		1<<2			; Unused P2.2
TxMiScu		EQU		1<<3			; Transmit- to SCU (TA1.0 output) on P2.3
RxScu		EQU		1<<4			; Receive from SCU (TA1.2 input) on P2.4
Spare1		EQU		1<<5			; Unused P2.5
PIEZO		EQU		1<<6			; Piezo (TA0.1 output) on P2.6
ERRLED		EQU		1<<7			; Error LED (red) on P2.7
PiezoPortDIR EQU	P2DIR			; Piezo on port 2
PiezoPortSEL EQU	P2SEL
PiezoPortOUT EQU	P2OUT

; Port 3 bit masks. BMU only.
RxChg		EQU		1<<0			; Receive from charger (TA0.2 input) on P3.0
PreCont		EQU		1<<1			; Precharge contactor output on P3.1
BatCont		EQU		1<<2			; Battery contactor output  P3.2
ChgCont		EQU		1<<3			; Charge sources contactor output  P3.3
TxMiChg		EQU		1<<4			; Transmit- to charger (TA0.0 output) on P3.4
DisCont		EQU		1<<5			; Discharge or discretionary loads contactor output on P3.5
NrmCont		EQU		1<<6			; Normal loads contactor output on P3.6
Spare2		EQU		1<<7			; Unused P3.7
SocMeter	EQU		NrmCont			; PWM output for SoC meter (monolith only)
SocPortDIR	EQU		P3DIR			; Soc meter PWM output on port 3
SocPortSEL	EQU		P3SEL
SocPortOUT	EQU		P3OUT

#define		PROG_START	$C000		// Start of program image in flash memory. Ends at $FDFF

#else							// Else using the older MSP430G2452 processor

InitSP		EQU		$300			; Initial value of stack pointer

; Port 1 bit masks
PIEZO		EQU		1<<0			; Piezo on P1.0
TXDm		EQU		1<<1			; TA0/P1.1. Code requires TXDm bit somewhere to right of TXDp bit
RXD			EQU		1<<2			; Receive data on P1.2 (bit number >3 costs 2 words)
#if REV61
SocMeter	EQU		1<<3			; PWM output for SoC meter (monolith only)
#else
ACTLED		EQU		1<<3			; Actvity LED (blue) inverted on P1.3
#endif
VREFp		EQU		1<<4			; Analog reference output. Sometimes changed to low digital output
TXDp		EQU		1<<5			; SCLK/P1.5. Can be made inverse of TA0 by hardware.
PiezoPortDIR EQU	P1DIR			; Piezo on port 1
PiezoPortSEL EQU	P1SEL
PiezoPortOUT EQU	P1OUT
SocPortDIR	EQU		P1DIR			; Soc meter PWM output on port 1
SocPortSEL	EQU		P1SEL
SocPortOUT	EQU		P1OUT

; Port 2 bit masks
BYPASS		EQU		1<<6			; Bypass transistor on P2.6
RELAYm		EQU		1<<6			; IMU relay for HazV- test on P2.6
ERRLED		EQU		1<<7			; Error LED (red) on P2.7
RELAYp		EQU		1<<7			; IMU relay for HazV+ test on P2.7
BypPortDIR	EQU		P2DIR			; Bypass MOSFET output on port 2
BypPortSEL	EQU		P2SEL
BypPortOUT	EQU		P2OUT

#define		PROG_START	$E000		// Start of program image in flash memory. Ends at $FDFF

#endif							// End else using the older MSP430G2452 processor


#define		WATCHDOG	1			// True if watchdog timer is to be used (only turn off for debugging)
									// Turning it off doesn't work because BSL will still clear and
									// restart the watchdog timer on every call to ReadByte.
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

