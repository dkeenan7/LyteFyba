; Common definitions for monitor, TestICal and BSL

#define		WATCHDOG	1			// True if watchdog timer is to be used (only turn off for debugging)
									// Turning it off doesn't work because BSL will still clear and
									// restart the watchdog timer on every call to ReadByte.
#define		PROG_START	$E000		// Start of program image in flash memory. Ends at $FDFF
#define		BSL2_START	$FE00		// Start of BSL2 image in flash memory. Ends at $FFFD

; The address BSL2 downloads to is usually the same as PROG_START,
; but when making a transition between different download sizes, the version of TestICal that does the
; update to the new BSL2 will still need to be the old size, so it can be downloaded by the old BSL2.
#define		PROG_START_FOR_BSL PROG_START	// Where the BSL should put the images it downloads
//#define		PROG_START_FOR_BSL		$E000	// Would be used temporarily while changing to a BSL that
										//	loads a different sized image to the one it is contained in.
										//	Don't forget, the password may change too.