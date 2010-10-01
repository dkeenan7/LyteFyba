;-------------------------------------------------------------------------------
; Assembler Structured Control-flow Macros
; Dave Keenan, 5-Feb-2010
; <d.keenan@bigpond.net.au>

; Make a Control-flow Stack (CS) in the assembler so we can implement
; Forth-like structured control-flow words for assembly.

_CS_TOP	SET 0
_CS2    SET 0
_CS3    SET 0
_CS4    SET 0
_CS5    SET 0
_CS6    SET 0
_CS7    SET 0
_CS8    SET 0

_CS_PUSH MACRO arg
_CS8	SET _CS7
_CS7	SET _CS6
_CS6	SET _CS5
_CS5    SET _CS4
_CS4    SET _CS3
_CS3    SET _CS2
_CS2    SET _CS_TOP
_CS_TOP SET arg
        ENDM

_CS_DROP MACRO
_CS_TOP	SET _CS2
_CS2    SET _CS3
_CS3    SET _CS4
_CS4    SET _CS5
_CS5    SET _CS6
_CS6    SET _CS7
_CS7    SET _CS8
_CS8    SET 0
        ENDM

_CS_SWAP MACRO
_CS_TOP SET _CS_TOP^_CS2
_CS2    SET _CS_TOP^_CS2
_CS_TOP SET _CS_TOP^_CS2
        ENDM

; Define condition codes for structured assembly. Used with _IF _WHILE _UNTIL.
; For convenience they are defined as bits 12..10 of the machine-code for the jump instruction
; with the opposite condition.

; MSP430 condition codes
_Z  EQU 0 ; (jnz) Zero
_EQ EQU 0 ; (jne) Equal, zame as Zero
_NZ EQU 1 ; (jz ) Not Zero
_NE EQU 1 ; (jeq) Not Equal, same as Not Zero
_C  EQU 2 ; (jnc) Carry
_HS EQU 2 ; (jlo) High or Same (unsigned), same as Carry
_NC EQU 3 ; (jc ) No Carry
_LO EQU 3 ; (jhs) Low (unsigned), same as Not Carry
;_N !!! Negative condition not available.
          ;       No JNN in MPS430 instruction set
          ;       Use _L while ensuring V flag cleared
          ;       e.g. by TST, or invert logic and use _NN.
_NN EQU 4 ; (jn ) Not Negative
_L  EQU 5 ; (jge) Less (signed), (N xor V)
_GE EQU 6 ; (jl ) Greater or Equal (signed), not(N xor V)
_NV EQU 7 ; (jmp) Never, i.e. unconditional, Used by ELSE, AGAIN, REPEAT


; Assemble a Jxx instruction given a condition code and an offset
_ASM_Jxx MACRO cond,offset
   ; ! Need to check for offset out of range, or odd, or bad condition code
   ; but can't figure out how to get assembler to give conditional message
   ; Tried "LIMIT" and "#message" to no avail.
        ; Assemble the jump instruction with offset
Jxx     SET 1<<13 | (cond&7)<<10 | (offset>>1)&$03FF
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
        _CS_PUSH (_NV << 29) | ($ & $1FFFFFFF) ; Push the condition code (unconditional) and
                   ; the address where the jump instruction will be filled-in later
        ORG $+2    ; Skip over that location
                        LSTOUT+
                        ENDM

; Mark the origin of a forward conditional branch.
; Called by _WHILE.
_IF 	MACRO cond
                        LSTOUT-
        _CS_PUSH (cond << 29) | ($ & $1FFFFFFF) ; Push the condition code and
                   ; the address where the jump instruction will be filled-in later
        ORG $+2    ; Skip over that location
                        LSTOUT+
                        ENDM

; Resolve a forward branch due to most recent _AHEAD, _IF, _ELSE or _WHILE.
; Called by _ELSE and _REPEAT.
_ENDIF	MACRO
                        LSTOUT-
_destin SET $           ; Remember where we were up to in assembling
_origin SET _CS_TOP & $1FFFFFFF ; Mask off the condition code to leave the origin address
		ORG _origin     ; Go back to the address on the top of the control-flow stack
_offset SET _destin-_origin-2 ; Calculate the offset in bytes
_cond   SET _CS_TOP>>29 ; Extract the condition code
        _ASM_Jxx  _cond,_offset ; Assemble the jump instruction with offset
		_CS_DROP        ; Drop the address and cond code off the control-flow stack
        ORG _destin     ; Go forward again to continue assembling
                        LSTOUT+
                        ENDM

; Mark the origin of a forward unconditional branch and
; resolve a forward branch due to an _IF.
_ELSE	MACRO
                        LSTOUT-
        _AHEAD          ; Leave space for an unconditional jump and push its address
                        LSTOUT-
        _CS_SWAP        ; Get the original _IF address back on top
		_ENDIF          ; Back-fill the jump and offset for previous _IF.
                        ENDM

; Define macros for
; _BEGIN ... _AGAIN                  (infinite)
; _BEGIN ... _UNTIL _CC              (post-tested
; _BEGIN ... _WHILE _CC ... _REPEAT  (pre or mid tested)

; Mark a backward destination (i.e. the start of a loop)
_BEGIN  MACRO
                        LSTOUT-
        _CS_PUSH $      ; Push the address to jump back to
                        LSTOUT+
                        ENDM


; Resolve most recent _BEGIN with a backward unconditional branch
; The end of an infinite loop
_AGAIN  MACRO
                        LSTOUT-
_offset SET _CS_TOP-$-2
        _ASM_Jxx  _NV,_offset  ; Assemble an unconditional jump back to the address on the top of the stack
        _CS_DROP        ; Drop the address off the control-flow stack
                        LSTOUT+
                        ENDM

; Resolve most recent _BEGIN with a backward conditional branch
; The end of a post-tested loop
_UNTIL	MACRO cond
                        LSTOUT-
_offset SET _CS_TOP-$-2
		_ASM_Jxx  cond,_offset ; Assemble a conditional jump back to the address on the top of the stack
		_CS_DROP        ; Drop the address off the control-flow stack
                        LSTOUT+
                        ENDM


; Mark the origin of a forward conditional branch out of a loop
; The test of a pre-tested or mid-tested loop
_WHILE	MACRO cond
                        LSTOUT-
		_IF cond        ; Assemble conditional jump and push the address of its offset
                        LSTOUT-
		_CS_SWAP        ; Get the _BEGIN address back on top
                        LSTOUT+
                        ENDM


; Resolve most recent _BEGIN with a backward unconditional branch and
; resolve a forward branch due to most recent _WHILE.
; The end of a pre-tested or mid-tested loop
_REPEAT MACRO
                        LSTOUT-
		_AGAIN          ; Jump back to the corresponding _BEGIN
                        LSTOUT-
		_ENDIF          ; Fill in the offset for the last _WHILE
                        ENDM

; Any loop may have additional _WHILEs to exit it, but each additional one must be
; balanced by an _ENDIF (or _ELSE ... _ENDIF) after the end of the loop. Examples:
; _BEGIN ... _WHILE _CC ... _WHILE _CC  ... _REPEAT ... _ENDIF
; _BEGIN ... _WHILE _CC ... _UNTIL _CC  ... _ELSE ...  _ENDIF
; _BEGIN ... _WHILE _CC ... _WHILE _CC  ... _AGAIN ... _ENDIF ... _ENDIF
;
; See http://www.taygeta.com/forth/dpansa3.htm#A.3.2.3.2


; CASE statement macros

; Typical use:

;       _CASE
;         _OF src,dest        ; OF uses CMP (word comparison)
;            ...
;         _ENDOF
;         _OFb src,dest       ; OFb uses CMP.B (byte comparison)
;           ...
;         _ENDOF
;         ... ( default case )
;       _ENDCASE


_CASE   MACRO
                        LSTOUT-
        _CS_PUSH 0      ; Push an _OF-count of zero onto the control flow stack
                        LSTOUT+
                        ENDM


_OF     MACRO src,dest  ; src is usually #N, dest can be Rn, X(Rn), &ADDR, ADDR
                        LSTOUT-
_count  SET _CS_TOP+1   ; Increment the _OF-count
        _CS_DROP        ; and take it off the stack for now
        ; Assemble the word comparison
                        LSTOUT+
        CMP src,dest
                        LSTOUT-
        _IF _EQ         ; Mark where a conditional jump will be backfilled later
                        LSTOUT-
        _CS_PUSH _count ; Put the _OF-count back on the stack
                        LSTOUT+
                        ENDM

_OFb    MACRO src,dest  ; src is usually #N, dest can be Rn, X(Rn), &ADDR, ADDR
                        LSTOUT-
_count  SET _CS_TOP+1   ; Increment the _OF-count
        _CS_DROP        ; and take it off the stack for now
        ; Assemble the byte comparison
                        LSTOUT+
        CMP.B src,dest
                        LSTOUT-
        _IF _EQ         ; Mark where a conditional jump will be backfilled later
                        LSTOUT-
        _CS_PUSH _count ; Put the _OF-count back on the stack
                        LSTOUT+
                        ENDM


_ENDOF  MACRO
                        LSTOUT-
_count  SET _CS_TOP     ; Take the _OF-count off the stack for now
                        LSTOUT-
        _CS_DROP
        _ELSE           ; Resolve the previous unconditional jump
                        LSTOUT-
                        ; and mark where an unconditional jump will be backfilled later
        _CS_PUSH _count ; Put the _OF-count back on the stack
                        LSTOUT+
                        ENDM


_ENDCASE MACRO
                        LSTOUT-
_count  SET _CS_TOP     ; Take the _OF-count off the stack for good
                        LSTOUT-
        _CS_DROP
        REPT _count     ; Repeat _OF-count times
          _ENDIF        ; Resolve an unconditional forward jump
                        LSTOUT-
        ENDR
                        LSTOUT+
                        ENDM


; Implement DO or ?DO ... LOOP or -LOOP


_DO     MACRO src,dest
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

_qDO    MACRO src,dest
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

