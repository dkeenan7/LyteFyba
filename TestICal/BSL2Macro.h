; The start of a macro wrapper for the inactive copy of the bootstrap-loader and interrupt vectors
; as carried by TestICal for use by the 'b' (bootstrap-loader update) command.
; The whole Macro is assembled by the following prebuild Windows command
; type BSL2Macro.h BSL2.s43 BSL2Endm.h > BSL2Macro.s43
; This is needed because IAR doesn't allow #includes inside macros

BSL2MACRO	MACRO	jBSL, jBSLErase, jReadByte, jWriteByte, jBslRevision
			LOCAL	BSL, BSLErase, ReadByte, WriteByte
			LOCAL	MClock, BaudRate, BitCycles, BitCyc1_5, MckPerFTGck, initBSL2, mainLoop2
			LOCAL	resetFSM2, FSMloop2, processByte2, eraseLoop, progByte2, TestNoInterp2
			LOCAL	testNoInterp, WriteBrk, WriteBreak, BslRevision
			LOCAL	freeSpaceBSL2, BSL2presence, BSL2checksum
			LOCAL	TA0FllIsr

