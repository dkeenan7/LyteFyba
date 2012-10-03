#include <stdlib.h>						// For exit()

inline void assert2(bool pred, const char* str) {
	if (!pred)
		exit(1);						// This is a great place for a breakpoint
}