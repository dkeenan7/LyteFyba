;-------------------------------------------------------------------------------
; Assembler Structured Control-flow Macros
; Dave Keenan, 5-Feb-2010
; <d.keenan@bigpond.net.au>

; Make a Control-flow Stack (CS) in the assembler so we can implement
; Forth-like structured control-flow words for assembly.

_CS_TOP	SET 0
_CS2	SET 0
_CS3	SET 0
_CS4	SET 0
_CS5	SET 0
_CS6	SET 0
_CS7	SET 0
_CS8	SET 0

_CS_PUSH MACRO arg
_CS8	SET _CS7
_CS7	SET _CS6
_CS6	SET _CS5
_CS5	SET _CS4
_CS4	SET _CS3
_CS3	SET _CS2
_CS2	SET _CS_TOP
_CS_TOP SET arg
		ENDM

_CS_DROP MACRO
_CS_TOP	SET _CS2
_CS2	SET _CS3
_CS3	SET _CS4
_CS4	SET _CS5
_CS5	SET _CS6
_CS6	SET _CS7
_CS7	SET _CS8
_CS8	SET 0
		ENDM

_CS_SWAP MACRO
_CS_TOP SET _CS_TOP^_CS2
_CS2	SET _CS_TOP^_CS2
_CS_TOP SET _CS_TOP^_CS2
		ENDM

; Define condition codes for structured assembly. Used with _IF _WHILE _UNTIL.
; For convenience they are mostly defined as bits 12..10 of the machine-code for the jump instruction
; with the inverse condition. But codes for _NN and above are increased by one to allow for
; an _N code, despite the MSP430 having no JNN instruction.
; This is to allow the use of _N with _OR_ELSE macros, where it will generate a JN instruction.
; For other uses of _N, and for uses of _NN with _OR_ELSE macros,
; a JGE instruction will be generated, with no warning issued.
; One should instead use _L (or _GE with _OR_ELSE) and ensure the V flag is cleared
; e.g. by a preceding TST instruction, or invert the logic and use _NN (except with _OR_ELSE).

; MSP430 condition codes
_Z  EQU 0 ; (jnz) Zero
_EQ EQU 0 ; (jne) Equal, zame as Zero
_NZ EQU 1 ; (jz ) Not Zero
_NE EQU 1 ; (jeq) Not Equal, same as Not Zero
_C  EQU 2 ; (jnc) Carry
_HS EQU 2 ; (jlo) High or Same (unsigned), same as Carry
_NC EQU 3 ; (jc ) No Carry
_LO EQU 3 ; (jhs) Low (unsigned), same as Not Carry
_N	EQU 4 ; (jge substituted with no warning) Negative. There is no jnn instruction.
_NN EQU 5 ; (jn ) Not Negative
_L  EQU 6 ; (jge) Less (signed), (N xor V)
_GE EQU 7 ; (jl ) Greater or Equal (signed), not(N xor V)
_NEVER EQU 8 ; (jmp) Never, i.e. unconditional, Used by ELSE, AGAIN, REPEAT

#define InverseCond(cond) ((cond&1)==0)*(cond+1) + ((cond&1)!=0)*(cond-1)


; Assemble a Jxx instruction given a condition code and an offset
_ASM_Jxx MACRO cond,offset
	; ! Need to give error message for offset odd or out of range, or bad condition code,
	; and give warning when JGE is substituted for non-existent JNN,
	; but can't figure out how to get assembler to give conditional message
	; dependent on macro parameters.
	; Assembly hangs when I try to do the following. May be an assembler bug.

;		IF cond == _N
;			#message "Warning: jge substituted for non-existent jnn."
;			#message "Ensure V flag is cleared, e.g. by preceding tst."
;		ENDIF

cond_field	SET (cond<_N)*cond + (cond==_N)*(_L-1) + (cond>_N)*(cond-1)

		; Assemble the jump instruction with offset
Jxx	 SET 1<<13 | (cond_field&7)<<10 | (offset>>1)&$03FF
						LSTOUT+
		DW Jxx
						LSTOUT-
						ENDM


; Define macros for
; _IF _CC ... _ENDIF
; _IF _CC ... _ELSE ... _ENDIF

; Mark the origin of a forward unconditional branch.
; Called by _ELSE.
_AHEAD  MACRO
						LSTOUT-
		_CS_PUSH (_NEVER << 28) | ($ & $0FFFFFFF) ; Push the condition code (unconditional) and
				   ; the address where the jump instruction will be filled-in later
		ORG $+2	; Skip over that location
						LSTOUT+
						ENDM

; Mark the origin of a forward conditional branch.
; Called by _WHILE.
_IF 	MACRO cond
						LSTOUT-
		_CS_PUSH (cond << 28) | ($ & $0FFFFFFF) ; Push the condition code and
				   ; the address where the jump instruction will be filled-in later
		ORG $+2	; Skip over that location
						LSTOUT+
						ENDM

; Resolve a forward branch due to most recent _AHEAD, _IF, _ELSE or _WHILE.
; Called by _ELSE and _REPEAT.
_ENDIF	MACRO
						LSTOUT-
_destin SET $		   ; Remember where we were up to in assembling
_origin SET _CS_TOP & $0FFFFFFF ; Mask off the condition code to leave the origin address
		ORG _origin	 ; Go back to the address on the top of the control-flow stack
_offset SET _destin-_origin-2 ; Calculate the offset in bytes
_cond   SET _CS_TOP>>28 ; Extract the condition code
		_ASM_Jxx  _cond,_offset ; Assemble the jump instruction with offset
		_CS_DROP		; Drop the address and cond code off the control-flow stack
		ORG _destin	 ; Go forward again to continue assembling
						LSTOUT+
						ENDM

; Mark the origin of a forward unconditional branch and
; resolve a forward branch due to an _IF.
_ELSE	MACRO
						LSTOUT-
		_AHEAD		  ; Leave space for an unconditional jump and push its address
						LSTOUT-
		_CS_SWAP		; Get the original _IF address back on top
		_ENDIF		  ; Back-fill the jump and offset for previous _IF.
						ENDM

; Define macros for
; _BEGIN ... _AGAIN				  (infinite)
; _BEGIN ... _UNTIL _CC			  (post-tested
; _BEGIN ... _WHILE _CC ... _REPEAT  (pre or mid tested)

; Mark a backward destination (i.e. the start of a loop)
_BEGIN  MACRO
						LSTOUT-
		_CS_PUSH $	  ; Push the address to jump back to
						LSTOUT+
						ENDM


; Resolve most recent _BEGIN with a backward unconditional branch
; The end of an infinite loop
_AGAIN  MACRO
						LSTOUT-
_offset SET _CS_TOP-$-2
		_ASM_Jxx  _NEVER,_offset  ; Assemble an unconditional jump back to the address on the top of the stack
		_CS_DROP		; Drop the address off the control-flow stack
						LSTOUT+
						ENDM

; Resolve most recent _BEGIN with a backward conditional branch
; The end of a post-tested loop
_UNTIL	MACRO cond
						LSTOUT-
_offset SET _CS_TOP-$-2
		_ASM_Jxx  cond,_offset ; Assemble a conditional jump back to the address on the top of the stack
		_CS_DROP		; Drop the address off the control-flow stack
						LSTOUT+
						ENDM


; Mark the origin of a forward conditional branch out of a loop
; The test of a pre-tested or mid-tested loop
_WHILE	MACRO cond
						LSTOUT-
		_IF cond		; Assemble conditional jump and push the address of its offset
						LSTOUT-
		_CS_SWAP		; Get the _BEGIN address back on top
						LSTOUT+
						ENDM


; Resolve most recent _BEGIN with a backward unconditional branch and
; resolve a forward branch due to most recent _WHILE.
; The end of a pre-tested or mid-tested loop
_REPEAT MACRO
						LSTOUT-
		_AGAIN		  ; Jump back to the corresponding _BEGIN
						LSTOUT-
		_ENDIF		  ; Fill in the offset for the last _WHILE
						ENDM

; Any loop may have additional _WHILEs to exit it, but each additional one must be
; balanced by an _ENDIF (or _ELSE ... _ENDIF) after the end of the loop. Examples:
; _BEGIN ... _WHILE _CC ... _WHILE _CC  ... _REPEAT ... _ENDIF
; _BEGIN ... _WHILE _CC ... _UNTIL _CC  ... _ELSE ...  _ENDIF
; _BEGIN ... _WHILE _CC ... _WHILE _CC  ... _AGAIN ... _ENDIF ... _ENDIF
;
; See http://www.taygeta.com/forth/dpansa3.htm#A.3.2.3.2


; Short-circuit conditionals (using overlapping structures)

; _IF _CC1 && _CC2 && _CC3 ... _ELSE ... _ENDIF
; is written as
;	   ...
;	   _IF _CC1
;		   ...
;		   _IF _CC2
;			   ...
;			   _IF _CC3
;				   ...
;				_ELSE
;		   _END_PRIOR_IF
;	   _END_PRIOR_IF
;				   ...
;			   _ENDIF

; _IF _CC1 || _CC2 || _CC3 ... _ELSE ... _ENDIF
; is written as
;	   ...
;	   _IF _CC1_inverse
;		   ...
;		   _IF _CC2_inverse
;			   ...
;			   _IF _CC3
;		   _END_PRIOR_IF
;	   _END_PRIOR_IF
;				   ...
;				_ELSE
;				   ...
;			   _ENDIF

_END_PRIOR_IF	MACRO
						LSTOUT-
		_CS_SWAP
		_ENDIF
						ENDM


; Short-circuit conditionals (without overlapping structures)

; _IF _CC1 && _CC2 && _CC3 ... _ELSE ... _ENDIF
; is written as
;		_COND
;			...
;		_AND_IF		_CC1
;			...
;		_AND_IF		_CC2
;			...
;		_AND_IF		_CC3
;			...
;		_ELSES
;			...
;		_ENDIF			; Use _ENDIFS if there is no ELSES clause

; _IF _CC1 || _CC2 || _CC3 ... _ELSE ... _ENDIF
; is written as
;		_COND
;			...
;		_OR_ELSE	_CC1
;			...
;		_OR_ELSE	_CC2
;			...
;		_OR_IFS 	_CC3
;			...
;		_ELSE
;			...
;		_ENDIF


_COND	MACRO			; Begin a short-circuit conditional of either type
						LSTOUT-
		_CS_PUSH 0		; Push an _IF-count of zero onto the control flow stack
						LSTOUT+
						ENDM


_AND_IF	MACRO cond		; Short circuit AND condition
						LSTOUT-
_count  SET _CS_TOP+1   ; Copy and increment the _IF-count
		_CS_DROP		; and take it off the stack for now

		_IF cond
						LSTOUT-
		_CS_PUSH _count ; Put the _IF-count back on the stack
						LSTOUT+
						ENDM


_ELSES	MACRO			; Used in place of ELSE for short circuit AND
						LSTOUT-
_count  SET _CS_TOP-1	; Copy and decrement the _IF-count
		_CS_DROP		; and take it off the stack permanently

		_ELSE
						LSTOUT-
		REPT _count		; Repeat _IF-count - 1 times
			_END_PRIOR_IF	; Resolve a forward jump
						LSTOUT-
		ENDR
						LSTOUT+
						ENDM


_ENDIFS	MACRO			; Used in place of ENDIF for short-circuit AND, but only
						; when there is no ELSES clause
						LSTOUT-
_count  SET _CS_TOP		; Copy the _IF-count
		_CS_DROP		; and take it off the stack permanently
		REPT _count		; Repeat _IF-count times
			_ENDIF		; Resolve a forward jump
						LSTOUT-
		ENDR
						LSTOUT+
						ENDM


_OR_ELSE MACRO cond		; Short circuit OR condition, except last
						LSTOUT-
_count  SET _CS_TOP+1   ; Copy and increment the _IF-count
		_CS_DROP		; and take it off the stack for now

		_IF InverseCond(cond)
						LSTOUT-
		_CS_PUSH _count ; Put the _IF-count back on the stack
						LSTOUT+
						ENDM


_OR_IFS	MACRO cond		; Last short-circuit OR condition
						LSTOUT-
_count  SET _CS_TOP		; Copy the _IF-count
		_CS_DROP		; and take it off the stack for good
		_IF	cond
						LSTOUT-
		REPT _count		; Repeat _OF-count times
			_END_PRIOR_IF	; Resolve an unconditional forward jump
						LSTOUT-
		ENDR
						LSTOUT+
						ENDM


; CASE statement macros

; Typical use:

;	   _CASE
;		   _OF src,dest		; OF uses CMP (word comparison)
;				...
;		   _ENDOF
;		   _OFb src,dest	; OFb uses CMP.B (byte comparison)
;			   ...
;		   _ENDOF
;		   ... ( default case )
;	   _ENDCASE


_CASE   MACRO
						LSTOUT-
		_CS_PUSH 0	 	; Push an _OF-count of zero onto the control flow stack
						LSTOUT+
						ENDM


_OF	 MACRO src,dest  ; src is usually #N, dest can be Rn, X(Rn), &ADDR, ADDR
						LSTOUT-
_count  SET _CS_TOP+1   ; Increment the _OF-count
		_CS_DROP		; and take it off the stack for now
		; Assemble the word comparison
						LSTOUT+
		CMP src,dest
						LSTOUT-
		_IF _EQ			; Mark where a conditional jump will be backfilled later
						LSTOUT-
		_CS_PUSH _count ; Put the _OF-count back on the stack
						LSTOUT+
						ENDM

_OFb	MACRO src,dest  ; src is usually #N, dest can be Rn, X(Rn), &ADDR, ADDR
						LSTOUT-
_count  SET _CS_TOP+1   ; Increment the _OF-count
		_CS_DROP		; and take it off the stack for now
		; Assemble the byte comparison
						LSTOUT+
		CMP.B src,dest
						LSTOUT-
		_IF _EQ			; Mark where a conditional jump will be backfilled later
						LSTOUT-
		_CS_PUSH _count ; Put the _OF-count back on the stack
						LSTOUT+
						ENDM


_ENDOF  MACRO
						LSTOUT-
_count  SET _CS_TOP		; Take the _OF-count off the stack for now

		_CS_DROP
		_ELSE			; Resolve the previous unconditional jump
						LSTOUT-
						; and mark where an unconditional jump will be backfilled later
		_CS_PUSH _count ; Put the _OF-count back on the stack
						LSTOUT+
						ENDM


_ENDCASE MACRO
						LSTOUT-
_count  SET _CS_TOP		; Copy the _OF-count
		_CS_DROP		; And take it  off the stack permanently
		REPT _count		; Repeat _OF-count times
			_ENDIF	  	; Resolve a forward jump
						LSTOUT-
		ENDR
						LSTOUT+
						ENDM


; Implement DO or ?DO ... LOOP or -LOOP


_DO	 MACRO src,dest
		MOV src,dest
						LSTOUT-
		_BEGIN
						ENDM

_LOOP   MACRO dest
		DEC   dest
						LSTOUT-
		_UNTIL _Z
						ENDM

_mLOOP  MACRO src,dest
		SUB   src,dest
						LSTOUT-
		_UNTIL _Z
						ENDM

_qDO	MACRO src,dest
		MOV src,dest
						LSTOUT-
		_BEGIN
						LSTOUT+
		TST dest
						LSTOUT-
		_WHILE _NZ
						ENDM

_qLOOP  MACRO dest
		DEC   dest
						LSTOUT-
		_REPEAT
						ENDM

_qmLOOP MACRO src,dest
		SUB   src,dest
						LSTOUT-
		_REPEAT
						ENDM

; Byte versions of the above
_DOb	MACRO src,dest
		MOV.B src,dest
						LSTOUT-
		_BEGIN
						ENDM

_LOOPb  MACRO dest
		DEC.B dest
						LSTOUT-
		_UNTIL _Z
						ENDM

_mLOOPb MACRO src,dest
		SUB.B src,dest
						LSTOUT-
		_UNTIL _Z
						ENDM

_qDOb   MACRO src,dest
		MOV.B src,dest
						LSTOUT-
		_BEGIN
						LSTOUT+
		TST.B dest
						LSTOUT-
		_WHILE _NZ
						ENDM

_qLOOPb MACRO dest
		DEC.B dest
						LSTOUT-
		_REPEAT
						ENDM

_qmLOOPb MACRO src,dest
		SUB.B src,dest
						LSTOUT-
		_REPEAT
						ENDM

