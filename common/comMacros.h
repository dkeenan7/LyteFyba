ActLedOff	MACRO
#if	REV61
			tst.b	&ID
			_IF		_NZ
				bit.b	#ERRLED,&P2OUT		; Activity and error LEDs share an output.
				_IF		_Z					; If the error LED is off
					bic.b	#ERRLED,&P2DIR		; make output high-Z so activity LED goes off
				_ENDIF							; without turning error LED on.
			_ENDIF
#else
			bis.b	#ACTLED,&P1OUT
#endif
			ENDM

ActLedOn	MACRO
#if	REV61
			; No provision for oscillating to light both LEDs yet.
			; Error LED has priority.
			tst.b	&ID
			_IF		_NZ
				bis.b	#ERRLED,&P2DIR		; Make it a proper output again
			_ENDIF
#else
			bic.b	#ACTLED,&P1OUT
#endif
			ENDM

ErrLedOff	MACRO
			tst.b	&ID
			_IF		_NZ
#if	REV61
				bic.b	#ERRLED,&P2DIR		; Make output high-Z so activity LED will not come on
				bic.b	#ERRLED,&P2OUT		; Turn off the error LED (so ActLedOff/On can tell)
#else
				bic.b	#ERRLED,&P2OUT		; Turn off the error LED
#endif
			_ENDIF
			ENDM

ErrLedOn	MACRO
			tst.b	&ID
			_IF		_NZ
#if	REV61
				; No provision for oscillating to light both LEDs yet.
				; Error LED has priority.
				bis.b	#ERRLED,&P2DIR		; Make it a proper output
				bis.b	#ERRLED,&P2OUT		; Turn on the error LED
#else
				bis.b	#ERRLED,&P2OUT		; Turn on the error LED
#endif
			_ENDIF
			ENDM

;
; Macros giving meaningful names to some obscure instruction sequences we have used repeatedly.
;
NCtoAllBits	MACRO	dest
			subc	dest,dest	; Sets all bits to not carry
			ENDM

CtoAllBits	MACRO	dest
			subc	dest,dest	; Sets all bits to not carry
			inv		dest		; Sets all bits to carry
			ENDM

allBitsIfZ	MACRO	src,dest
			and		src,src		; Sets carry if src is not zero
			subc	dest,dest	; Sets all bits to not carry = zero
			ENDM

allBitsIfNZ	MACRO	src,dest
			and		src,src		; Sets carry if src is not zero
			subc	dest,dest	; Sets all bits to not carry = zero
			inv		dest		; Sets all bits to carry = not zero
			ENDM

movBits		MACRO	src,mask,dest	; Trashes src
			xor		dest,src	; Get one bits in src for bits that need to toggle
			and		mask,src	; Mask so we only change those we want to change
			xor		src,dest	; Toggle bits as required
			ENDM

movBits_B	MACRO	src,mask,dest	; Trashes src
			xor.b	dest,src	; Get one bits in src for bits that need to toggle
			and.b	mask,src	; Mask so we only change those we want to change
			xor.b	src,dest	; Toggle bits as required
			ENDM

rra2		MACRO  dest
			rra	dest
			rra	dest
			ENDM

rra3		MACRO  dest
			rra	dest
			rra	dest
			rra	dest
			ENDM

rra4		MACRO  dest
			rra	dest
			rra	dest
			rra	dest
			rra	dest
			ENDM

rra5		MACRO  dest
			rra	dest
			rra	dest
			rra	dest
			rra	dest
			rra	dest
			ENDM

rra6		MACRO  dest
			rra	dest
			rra	dest
			rra	dest
			rra	dest
			rra	dest
			rra	dest
			ENDM

rla2		MACRO  dest
			rla	dest
			rla	dest
			ENDM

rla3		MACRO  dest
			rla	dest
			rla	dest
			rla	dest
			ENDM

rla4		MACRO  dest
			rla	dest
			rla	dest
			rla	dest
			rla	dest
			ENDM

rla5		MACRO  dest
			rla	dest
			rla	dest
			rla	dest
			rla	dest
			rla	dest
			ENDM

rla6		MACRO  dest
			rla	dest
			rla	dest
			rla	dest
			rla	dest
			rla	dest
			rla	dest
			ENDM

ClearWatchdog MACRO
#if WATCHDOG
			mov.w	#WDTPW+WDTCNTCL,&WDTCTL	; Clear and restart the watchdog timer
#endif
			ENDM

abs			MACRO	dest
			cmp		#0,dest
			_IF		_L
				inv		dest
				inc		dest
			_ENDIF
			ENDM