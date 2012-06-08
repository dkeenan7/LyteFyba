; Common definitions for monitor, debugger, BSLWriter, and BSL

#define 	PCBVERSION  57			// Before ver 57 there was no Activity LED or Piezo. Different pinouts
#define		WATCHDOG	1			// True if watchdog timer is to be used (only turn off for debugging)
									// Turning it off doesn't work because BSL will still clear and restart
									// the watchdog timer on every call to ReadByte.
#define		IMAGE_START	$F000		// Start of program image in flash memory. Always ends at $FFFF
//#define		BSL_IMAGE_START IMAGE_START	// Start of program image for the BSL
			; The above is usually the same as IMAGE_START, but can be different if a transition
			  ; image is required
#define		BSL_IMAGE_START		$F000	// For 4K debugger