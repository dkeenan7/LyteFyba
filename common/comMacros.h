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

