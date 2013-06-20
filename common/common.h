; Common definitions for monitor, TestICal and BSL

#define		WATCHDOG	1			// True if watchdog timer is to be used (only turn off for debugging)
									// Turning it off doesn't work because BSL will still clear and
									// restart the watchdog timer on every call to ReadByte.
#define		IMAGE_START	$F000		// Start of program image in flash memory. Ends at $FEFF
									// Low flash may be used for data logging in future.

; The address the BSL downloads to is usually the same as IMAGE_START,
; but when making a transition between different download sizes, the version of TestICal that does the
; update to the new BSL will still need to be the old size, so it can be downloaded by the old BSL.
#define		BSL_IMAGE_START IMAGE_START	// Where the BSL should put the images it downloads
//#define		BSL_IMAGE_START		$E000	// Would be used temporarily while changing to a BSL that
										//	loads an 8K image