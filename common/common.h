; Common definitions for TestICal, monitor, monolith and BSL

HW_REV		EQU		64				; Expected hardware revision number for this code

InitSP		EQU		$400			; Initial value of stack pointer

; Port 1 bit masks
Piezo		EQU		1<<0			; Piezo on P1.0
Rx			EQU		1<<1			; Receive from CMUs (UART input) on P1.1
TxPl 		EQU		1<<2			; Transmit+ to CMUs (UART output) on P1.2
TouchV		EQU		1<<3			; TouchV analog input (BMU) on P1.0,
VrefP		EQU		1<<4			; Analog reference output on P1.4
ArrayV_BoltVPl EQU	1<<5			; Analog input on P1.5
ShuntV_BoltVMi EQU	1<<6			; Analog input on P1.6
BatV_StrapVPl EQU	1<<7			; Analog input on P1.7

; Port 2 bit masks
ErrLed		EQU		1<<0			; Error LED (red) on P2.0
TxMiChg2_RlyP_57k6A EQU 1<<1		; Alt charger Tx- or HazV+ IM relay (BMU), 57.6 kHz IR (CMU) on P2.1
TxMiChg_57k6B EQU		1<<2		; Transmit- to charger (BMU), 57.6 kHz IR carrier (CMU) on P2.2
PreI		EQU		1<<3			; Precharge contactor auxiliary contact input (BMU) on P2.3
RxChg		EQU		1<<4			; Receive from charger (TA1.2 input) on P2.4
RxChg2_RlyM_Byp	EQU	1<<5			; IM relay for HazV- test (BMU), Bypass (CMU) on P2.5
Xin			EQU		1<<6			; Watch crystal in on P2.6
Xout		EQU		1<<7			; Watch crystal out on P2.7
Bypass		EQU		RxChg2_RlyM_Byp	; Aliases for code that's common to newer and older devices
BypPortDIR	EQU		P2DIR			; Bypass MOSFET output on port 2
BypPortSEL	EQU		P2SEL
BypPortOUT	EQU		P2OUT
RelayM		EQU		RxChg2_RlyM_Byp
RelayP		EQU		TxMiChg2_RlyP_57k6A
TxMiChg2	EQU		TxMiChg2_RlyP_57k6A
TxMiChg		EQU		TxMiChg_57k6B
RxChg2		EQU		RxChg2_RlyM_Byp
Ir57k6A		EQU		TxMiChg2_RlyP_57k6A
Ir57k6B		EQU		TxMiChg_57k6B
PreIPortIN	EQU		P2IN			; Precharge auxiliary contact input on port 2
ChgPortDIR	EQU		P2DIR			; Charger comms on port 2
ChgPortSEL	EQU		P2SEL
ChgPortOUT	EQU		P2OUT

; Port 3 bit masks. BMU only.
RxScu		EQU		1<<0			; Receive from SCU (TA0.2 input) on P3.0
PreCtor		EQU		1<<1			; Precharge contactor output on P3.1
BatPosCtor	EQU		1<<2			; Battery positive contactor output  P3.2
BatNegCtor	EQU		1<<3			; Battery negative contactor output  P3.3
AcLfPvCtor	EQU		1<<4			; AC and left PV sources contactor output on P3.4
TxMiScu		EQU		1<<5			; Transmit- to SCU (TA0.1 output) on P3.5
RtPvCtor	EQU		1<<6			; Right PV source contactor output on P3.6
BatI		EQU		1<<7			; Battery contactor auxiliary contact input on P3.7
SocMeter	EQU		BatI			; PWM output for SoC meter (monolith only) Prob not used in future
SocPortDIR	EQU		P3DIR			; Soc meter PWM output is on port 3
SocPortSEL	EQU		P3SEL
SocPortOUT	EQU		P3OUT
BatIPortIN	EQU		P3IN			; BatI input is on port 3
ScuPortDIR	EQU		P3DIR			; SCU comms on port 3
ScuPortSEL	EQU		P3SEL
ScuPortOUT	EQU		P3OUT
ScuPortIN	EQU		P3IN
CtorPortOUT	EQU		P3OUT

; ADC channel numbers
TouchVChan	EQU		$3				; ADC channel number for touch voltage (BMU only)
VRefPChan	EQU		$4				; Vref+ out
ArrayV_BoltVPl_Chan	EQU	$5			; PV Array voltage (BMU) Bolt+ voltage (CMU)
ShuntV_BoltVMi_Chan	EQU	$6			; Current shunt voltage (BMU), Bolt- voltage (CMU)
BatV_StrapVPl_Chan 	EQU	$7			; Battery voltage (BMU), Strap+ (cell) voltage (CMU)
TempChan	EQU		$A				; Temperature
BoltVMiChan	EQU		ShuntV_BoltVMi_Chan	; Aliases for code that's common to newer and older devices
CellVChan	EQU		BatV_StrapVPl_Chan
BoltVPlChan EQU		ArrayV_BoltVPl_Chan
NumSamples	EQU		16				; Number of ADC over-samples (typ. 4 or 16)
; To get n more bits of ADC resolution, add up 4^n samples and shift the result right by n bits.
; i.e. By adding up 4^n samples you get 2n more bits in the result,
; but half of them are noise and should be thrown away,
; leaving us with n bits of additional information.

#define		PROG_START	$C000		// Start of program image in flash memory. Ends at BSL2_START-1
#define		REV61_PROG_ST $E000		// Equivalent start of program image for old rev61 images

#define		WATCHDOG	1			// True if watchdog timer is to be used (only turn off for debugging)
									// Turning it off doesn't work because BSL will still clear and
									// restart the watchdog timer on every call to ReadByte.
#define		BSL2_START	$FC00		// Start BSL2 1 KiB before the end.
#define		REV61_BSL2_ST $FE00		// Equivalent start for old rev61 BSL

; The address BSL2 downloads a program to is usually the same as PROG_START,
; but when making a transition between different download sizes, the version of TestICal that does the
; update to the new BSL2 will still need to be the old size, so it can be downloaded by the old BSL2.
#define		PROG_START_FOR_BSL		PROG_START	// Where the BSL should put the images it downloads.
//#define		PROG_START_FOR_BSL		$E000	// Would be used temporarily while changing to a BSL that
										//	loads a different sized image to the one it is contained in.
										//	The password may need changing too, in both BSL2 and Monitor.

// Convert unfriendly TI provided indices to the addresses of the relevant constants
// Their index names start with CAL_ADC, our pointer names start with CALADC (one less underscore)
CALADC_15VREF_FACTOR EQU TLV_ADC10_1_TAG_ + 2 + (CAL_ADC_15VREF_FACTOR * 2) ; Index vref cal off tag adr
CALADC_OFFSET		 EQU TLV_ADC10_1_TAG_ + 2 + (CAL_ADC_OFFSET		   * 2) ; Index off. cal off tag adr
CALADC_GAIN_FACTOR	 EQU TLV_ADC10_1_TAG_ + 2 + (CAL_ADC_GAIN_FACTOR   * 2) ; Index gain cal off tag adr
CALADC_15T30		 EQU TLV_ADC10_1_TAG_ + 2 + (CAL_ADC_15T30 * 2) 		; Index temp cal off tag adr
CALADC_15T85		 EQU TLV_ADC10_1_TAG_ + 2 + (CAL_ADC_15T85 * 2)

				ORG		$1004		; For compatibility with existing calibration values
			; Calibration data
DATAVERS		EQU		6			; This is version 6 of the CMU info-flash data structure
infoDataStart						; Used when copying between ram and info-flash
infoBoltMiCal	ds		2			; Bolt- voltage / current scale calibration word
infoTempSlope	ds		2			; Precomputed slope of temperature vs ADC-value curve
infoBoltPlOff	ds		1			; Bolt/array voltage offset calibration signed byte
infoCellOff		ds		1			; Cell/battery voltage offset calibration signed byte
infoCapacity	ds		2			; Battery capacity in tenths of an amp-hour
infoCellRes		ds		2			; High temp cell internal resistance in micro-ohms
infoBoltPlCal	ds		2			; Bolt+/array voltage scale calibration word
infoCellCal		ds		2			; Cell/battery voltage scale calibration word
infoTempOff		ds		1			; Temperature offset calibration for internal sensor
infoBoltMiOff	ds		1			; Bolt- voltage / current offset calibration signed byte
info8MHzCalD	ds		1			; 8 MHz DCO frequency calibration byte
info8MHzCalB	ds		1			; 8 MHz DCO range calibration byte
infoID			ds		1			; Cell/CMU identifier byte; first cell is 1; written by 'i' cmd
infoDataVers	ds		1			; Data Version byte (cannot move). Must be set to DATAVERS value above
; Note that xxxDataEnd is one PAST the last calibration byte, i.e. the address of the start of what
;	comes after the calibration data
infoDataEnd							; Used when copying between ram and info-flash

; To allow moving old calibration data from the end of the A segment to the new D segment location
oldInfoDataStart	EQU	$10F8
oldInfoDataVers		EQU $10FF

; Clock constants. They were in InterruptComms.h, but even TestICal needs these now
DCOfreq		EQU		3686400				; DCO clock in Hz. Allows for 57.6 kHz IR carrier & 9600 b/s
DCOckPerMck	EQU		1					; DCO clocks per MCK (CPU, ADC) (allowed values 1,2,4,8)
DCOckPerSMck EQU	1					; DCO clocks per SMCK (Timer) (allowed values 1,2,4,8)
SMckPerTAck	EQU		1					; Number of SMCK clocks per timer clock (allowed 1,2,4,8)
MckPerTAck	EQU		1
MClock		EQU		DCOfreq/DCOckPerMck	; MCLK (CPU clock) frequency in hertz
										;	(DCO software-locked to watch xtal)
BaudRate	EQU		9600				; Serial comms rate in bits per second

			; LOG2 -- The following preprocessor-macro gem is due to Dave Keenan.
			; It is based on a Taylor series expansion. It is valid for the domain 1-64, and range 0-6.
			; NOTE: It depends on the assembler rounding toward zero (truncated division).
			; There is a similar simple formula for rounding towards neg infinity (floored division).
			#define LOG2(x) (4 * (x-8) / (x+8) + 3)

