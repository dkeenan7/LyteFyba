; Common definitions for monitor, debugger, BSLWriter, and BSL

#define 	PCBVERSION  57			// Before ver 57 there was no Activity LED or Piezo. Different pinouts
#define		WATCHDOG	1			// True if watchdog timer is to be used (only turn off for debugging)
									// Turning it off doesn't work because BSL will still clear and restart
									// the watchdog timer on every call to ReadByte.
#define		FLASH_START	$E000		// Start of flash memory. Always ends at $FFFF