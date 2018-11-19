;-------------------------------------------------------------------------------
; Assembler Structured Control-flow Macros
; Dave Keenan, 5-Feb-2010, updated 10-Jan-2018
; Ported to CCS assembler for TMS320C28x by Dave Keenan 11-Nov-2018
; Documentation here:
; http://dkeenan.com/AddingStructuredControlFlowToAnyAssembler.htm

; In memory of Wil Baden (1928-2016)
; http://www.boston-baden.com/hazel/dad/
; Among Wil's many gifts to the world, is the elegant control-flow implementation scheme
; in ANS Forth (1994), on which this work is based.

; Make a Control-flow Stack (CS) in the assembler so we can implement structured control-flow words
; for assembly, similar to those in higher-level languages.
; It needs to have more elements than the maximum number of cases in any _CASE statement.
		.asg	0, _CS_TOP
		.asg	0, _CS2
		.asg	0, _CS3
		.asg	0, _CS4
		.asg	0, _CS5
		.asg	0, _CS6
		.asg	0, _CS7
		.asg	0, _CS8
		.asg	0, _CS9
		.asg	0, _CS10
		.asg	0, _CS11
		.asg	0, _CS12
		.asg	0, _CS_COUNT

; Initialise the variable used to generate unique labels beginning with "_L"
		.asg	100, _LABEL_NUM			; Can't be zero. Easier to read the listing if they are all 3 digits.


; Assign an assembler variable the value of an expression. Improves readability of macros below.
_SET	.macro var, expr
		.eval expr, var
		.endm

; Increment an assembler variable. Improves readability of macros below.
_INC	.macro var
		.eval var + 1, var
		.endm

; Decrement an assembler variable. Improves readability of macros below.
_DEC	.macro var
		.eval var - 1, var
		.endm

; Control flow stack operations
_CS_PUSH .macro arg
		_INC	_CS_COUNT
		.if	_CS_COUNT > 12
			.emsg "Control-flow stack overflow"
		.endif
		_SET	_CS12, _CS11
		_SET	_CS11, _CS10
		_SET	_CS10, _CS9
		_SET	_CS9, _CS8
		_SET	_CS8, _CS7
		_SET	_CS7, _CS6
		_SET	_CS6, _CS5
		_SET	_CS5, _CS4
		_SET	_CS4, _CS3
		_SET	_CS3, _CS2
		_SET	_CS2, _CS_TOP
		_SET	_CS_TOP, arg
		.endm

_CS_DROP .macro
		_DEC	_CS_COUNT
		.if	_CS_COUNT < 0
			.emsg "Control-flow stack underflow"
		.endif
		_SET	_CS_TOP, _CS2
		_SET	_CS2, _CS3
		_SET	_CS3, _CS4
		_SET	_CS4, _CS5
		_SET	_CS5, _CS6
		_SET	_CS6, _CS7
		_SET	_CS7, _CS8
		_SET	_CS8, _CS9
		_SET	_CS9, _CS10
		_SET	_CS10, _CS11
		_SET	_CS11, _CS12
		_SET	_CS12, 0
		.endm

_CS_SWAP .macro
		_SET	_CS_TOP, _CS_TOP ^ _CS2
		_SET	_CS2,    _CS_TOP ^ _CS2
		_SET	_CS_TOP, _CS_TOP ^ _CS2
		.endm

; Check that the control flow stack is empty and has not underflowed.
; Use at end of program, or anywhere that control flow structures should all be complete.
_CS_CHECK .macro
									.nolist
		.if	_CS_COUNT != 0
			.emsg "Control-flow stack is unbalanced"
		.endif
									.list
									.endm

; Generate a label from an integer variable. The label consists of "_L" followed by the
; decimal representation of the integer. Uses a recursive macro.
; The idea of using generated labels came from https://github.com/WestfW/structured_gas
; which was in turn inspired by the first version of my control-flow macros article,
; which did not use computed labels but relied on being able to ORG backwards
; to fill in a jump instruction using DW, once the offset was known.
; The GNU assembler can't ORG backwards.
; But it sure was difficult to come up with a way to generate labels in the IAR assembler.
; In the GNU assembler one can simply use % to evaluate an integer expression as a string
; at macro-expansion time.
_LABEL	.macro num, str		; ":str:" below is equivalent to "str" (2nd argument) but can be concatenated
		.if	:num: <= 0
									.list
_L:str:
									.nolist
		.else
			.if :num: % 8 == 0				; Forced to use octal, because assembler treats anything with
				_LABEL :num: / 8, 0:str:	; a leading zero as an octal constant, and hence it treats
			.elseif :num: % 8 == 1			; intermediate results of "08" and "09" as bad octal constants
				_LABEL :num: / 8, 1:str:	; and throws an error.
			.elseif :num: % 8 == 2
				_LABEL :num: / 8, 2:str:
			.elseif :num: % 8 == 3
				_LABEL :num: / 8, 3:str:
			.elseif :num: % 8 == 4
				_LABEL :num: / 8, 4:str:
			.elseif :num: % 8 == 5
				_LABEL :num: / 8, 5:str:
			.elseif :num: % 8 == 6
				_LABEL :num: / 8, 6:str:
			.elseif :num: % 8 == 7
				_LABEL :num: / 8, 7:str:
			.endif
		.endif
		.endm

; Assemble a (possibly conditional) jump instruction, generating the label from an integer variable.
_JUMP	.macro cond, num, str	; ":str:" below is equivalent to "str" (3rd argument) but can be concatenated
		.if	:num: <= 0
									.nolist
		_J:cond:	 _L:str:
									.nolist
		.else
			.if :num: % 8 == 0
				_JUMP :cond:, :num: / 8, 0:str:
			.elseif :num: % 8 == 1
				_JUMP :cond:, :num: / 8, 1:str:
			.elseif :num: % 8 == 2
				_JUMP :cond:, :num: / 8, 2:str:
			.elseif :num: % 8 == 3
				_JUMP :cond:, :num: / 8, 3:str:
			.elseif :num: % 8 == 4
				_JUMP :cond:, :num: / 8, 4:str:
			.elseif :num: % 8 == 5
				_JUMP :cond:, :num: / 8, 5:str:
			.elseif :num: % 8 == 6
				_JUMP :cond:, :num: / 8, 6:str:
			.elseif :num: % 8 == 7
				_JUMP :cond:, :num: / 8, 7:str:
			.endif
		.endif
		.endm

; Translate the jump instructions generated by _JUMP above,
; when "N" (for "not") is placed before the condition code.
; Used by _IF and _UNTIL. Specific to the TMS320C28x processor.

_JNZ		.macro	label
									.list
		sbf		label, NEQ
									.nolist
									.endm
_JNNZ	.macro	label
									.list
		sbf		label, EQ
									.nolist
									.endm
_JNEQ	.macro	label
									.list
		sbf		label, NEQ
									.nolist
									.endm
_JNNEQ	.macro	label
									.list
		sbf		label, EQ
									.nolist
									.endm
_JNC		.macro	label
									.list
		sb		label, NC
									.nolist
									.endm
_JNNC	.macro	label
									.list
		sb		label, C
									.nolist
									.endm
_JNHIS	.macro	label
									.list
		sb		label, LO
									.nolist
									.endm
_JNLO	.macro	label
									.list
		sb		label, HIS
									.nolist
									.endm
_JNLOS	.macro	label
									.list
		sb		label, HI
									.nolist
									.endm
_JNHI	.macro	label
									.list
		sb		label, LOS
									.nolist
									.endm
_JNLT	.macro	label
									.list
		sb		label, GEQ
									.nolist
									.endm
_JNGEQ	.macro	label
									.list
		sb		label, LT
									.nolist
									.endm
_JNGT	.macro	label
									.list
		sb		label, LEQ
									.nolist
									.endm
_JNLEQ	.macro	label
									.list
		sb		label, GT
									.nolist
									.endm
_JNOV	.macro	label
									.list
		sb		label, NOV
									.nolist
									.endm
_JNNOV	.macro	label
									.list
		sb		label, OV
									.nolist
									.endm
_JNTC	.macro	label
									.list
		sbf		label, NTC
									.nolist
									.endm
_JNNTC	.macro	label
									.list
		sbf		label, TC
									.nolist
									.endm
_JNBIO	.macro	label
									.list
		sb		label, NBIO
									.nolist
									.endm
_JNNBIO .macro	label
									.list
		sb		$+2, NBIO	; Workaround for the non-existent BIO condition code
		sb		label, UNC
									.nolist
									.endm
_JNNEVER .macro label				; An unconditional jump
									.list
		sb		label, UNC
									.nolist
									.endm

;------------------------------------------------------------
; Define macros for simple conditionals

;		<test>
;		_IF cc
;			<stuff to do if cc>
;		_ENDIF

;		<test>
;		_IF cc
;			<stuff to do if cc>
;		_ELSE
;			<stuff to do if NOT cc>
;		_ENDIF
;
; Where cc is one of Z, NZ, EQ, NEQ, C, NC, HIS, LO, LOS, HI, LT, GEQ, GT, LEQ, OV, NOV, TC, NTC, BIO, NBIO or NEVER.


; Mark the origin of a forward conditional branch.
; Called by _ELSE, _WHILE and OR_IFS. Aliased as _AND_IF and _OF.


_IF 	.macro cond		; ":cond:" below is equivalent to "cond" (1st argument) but can be concatenated
									.nolist
		_JUMP		N:cond:, _LABEL_NUM ; Assemble a conditional jump with the opposite condition
		_CS_PUSH	_LABEL_NUM		; Push its label number
		_INC		_LABEL_NUM		; Increment the label number
									.list
									.endm

; Resolve a forward branch due to most recent _IF, _ELSE or _WHILE.
; Called by _ELSE and _ENDW.

_ENDIF	.macro
									.nolist
		_LABEL		_CS_TOP			; Assemble the label for the previous _IF.
		_CS_DROP					; Drop its label number off the control-flow stack
									.list
									.endm

; Mark the origin of a forward unconditional branch and
; resolve a forward branch due to an _IF. Aliased as _ENDOF.

_ELSE	.macro
									.nolist
		_IF			NEVER			; Assemble an unconditional jump and push its label number
									.nolist
		_CS_SWAP					; Get the original _IF label number back on top
		_ENDIF						; Assemble the label for the previous _IF, and drop its number
									.endm


;------------------------------------------------------------
; Define macros for uncounted loops

;		_REPEAT
;			<do_stuff>
;		_FOREVER				(infinite)

;		_REPEAT
;			<do_stuff>
;			<test>
;		_UNTIL cc			(post-tested

;		_DO
;			<test>
;		_WHILE cc			(pre-tested)
;			<do_stuff>
;		_ENDW

;		_DO
;			<do_stuff>
;			<test>
;		_WHILE cc			(mid tested)
;			<do_other_stuff>
;		_ENDW


; Mark a backward destination (i.e. the start of a loop)

_REPEAT  .macro
									.nolist
		_LABEL		_LABEL_NUM		; Assemble the label
		_CS_PUSH	_LABEL_NUM		; Push the number of the label to jump back to
		_INC		_LABEL_NUM		; Increment the label number
									.list
									.endm

_DO  .macro
									.nolist
		_REPEAT
									.endm

; Resolve most recent _REPEAT or _DO with a backward conditional branch
; The end of a post-tested loop

_UNTIL	.macro cond
									.nolist
		_JUMP		N:cond:, _CS_TOP	; Assemble a conditional jump back to corresponding _REPEAT or _DO
		_CS_DROP					; Drop its label number off the control-flow stack
									.list
									.endm

; Resolve most recent _REPEAT with a backward unconditional branch
; The end of an infinite loop

_FOREVER  .macro
									.nolist
		_UNTIL		NEVER			; Assemble an unconditional jump back to the corresponding _REPEAT
									.endm

; Mark the origin of a forward conditional branch out of a loop
; The test of a pre-tested or mid-tested loop

_WHILE	.macro cond
									.nolist
		_IF			:cond:			; Assemble a conditional jump and push its label number
									.nolist
		_CS_SWAP					; Get the _DO label number back on top
									.list
									.endm

; Resolve most recent _DO with a backward unconditional branch and
; resolve a forward branch due to most recent _WHILE.
; The end of a pre-tested or mid-tested loop

_ENDW	.macro
									.nolist
		_UNTIL		NEVER			; Assemble an unconditional jump back to the corresponding _DO
									.nolist
		_ENDIF		  				; Assemble the label for the last _WHILE
									.endm


; Any loop may have additional _WHILEs to exit it, but each additional one must be
; balanced by an _ENDIF (or _ELSE ... _ENDIF) after the end of the loop. Examples:
; _DO     ... _WHILE cc1 ... _WHILE cc2  ... _ENDW  ... _ENDIF
; _REPEAT ... _WHILE cc1 ... _UNTIL cc2  ... _ELSE  ... _ENDIF
; _REPEAT ... _WHILE cc1 ... _WHILE cc2  ... _FOREVER ... _ENDIF ... _ENDIF
;
; See http://www.taygeta.com/forth/dpansa3.htm#A.3.2.3.2


;------------------------------------------------------------
; Short-circuit conditionals

; IF cond1 && cond2 && cond3 ... ENDIF
; is written as
;
;		_COND
;			<test1>
;		_AND_IF		cc1
;			<test2>
;		_AND_IF		cc2
;			<test3>
;		_AND_IF		cc3
;			<stuff to do when cond1 && cond2 && cond3>
;		_ENDIFS				; Note plural _ENDIFS when there is no _ELSES clause
;
; Note: COND ... ENDIFS is equivalent to COND ... THENS which, although it is not standard ANS Forth,
; is also due to Wil Baden.

; IF cond1 && cond2 && cond3 ... ELSE ... ENDIF
; is written as
;
;		_COND
;			<test1>
;		_AND_IF		cc1
;			<test2>
;		_AND_IF		cc2
;			<test3>
;		_AND_IF		cc3
;			<stuff to do when cond1 && cond2 && cond3>
;		_ELSES				; Note plural _ELSES
;			<stuff to do when NOT (cond1 && cond2 && cond3)>
;		_ENDIF				; Note singular _ENDIF when there is an _ELSES clause

; IF cond1 || cond2 || cond3 ... ENDIF
; is written as
;
;		_COND
;			<test1>
;		_OR_ELSE	cc1
;			<test2>
;		_OR_ELSE	cc2
;			<test3>
;		_OR_IFS 	cc3		; Note plural _OR_IFS for last condition
;			<stuff to do when cond1 || cond2 || cond3>
;		_ENDIF

; IF cond1 || cond2 || cond3 ... ELSE ... ENDIF
; is written as
;
;		_COND
;			<test1>
;		_OR_ELSE	cc1
;			<test2>
;		_OR_ELSE	cc2
;			<test3>
;		_OR_IFS 	cc3		; Note plural _OR_IFS for last condition
;			<stuff to do when cond1 || cond2 || cond3>
;		_ELSE
;			<stuff to do when NOT (cond1 || cond2 || cond3)>
;		_ENDIF

; IF cond1 && cond2 && cond3 && (cond4 || cond5 || cond6) ... ELSE ... ENDIF
; is written as
;
;		_COND
;			<test1>
;		_AND_IF		cc1
;			<test2>
;		_AND_IF		cc2
;			<test3>
;		_AND_IF		cc3
;		_COND				; Note the second COND to bracket the ORs
;				<test4>
;		_OR_ELSE	cc4
;				<test5>
;		_OR_ELSE	cc5
;				<test6>
;		_OR_IFS 	cc6		; Note plural _OR_IFS for last condition
;			<stuff to do when (cond1 && cond2 && cond3 && (cond4 || cond5 || cond6))>
;		_ELSES				; Note plural _ELSES
;			<stuff to do when NOT (cond1 && cond2 && cond3 && (cond4 || cond5 || cond6))>
;		_ENDIF				; Note singular _ENDIF when there is an _ELSES clause


; Mark the origin of a forward conditional branch with the opposite condition.
; Aliased to _OR_ELSE.

_IF_NOT .macro cond
									.nolist
		_JUMP		:cond:, _LABEL_NUM ; Assemble a conditional jump with the same condition
		_CS_PUSH	_LABEL_NUM		; Push its label number
		_INC		_LABEL_NUM		; Increment the label number
									.list
									.endm

; Translate the jump instructions generated by _JUMP above,
; when "N" (for "not") is not placed before the condition code.
; Used by _OR_ELSE. Specific to the TMS320C28x processor.

_JZ		.macro	label
									.list
		sbf		label, EQ
									.nolist
									.endm
_JEQ		.macro	label
									.list
		sbf		label, EQ
									.nolist
									.endm
_JC		.macro	label
									.list
		sb		label, C
									.nolist
									.endm
_JLO		.macro	label
									.list
		sb		label, LO
									.nolist
									.endm
_JHIS	.macro	label
									.list
		sb		label, HIS
									.nolist
									.endm
_JHI		.macro	label
									.list
		sb		label, HI
									.nolist
									.endm
_JLOS	.macro	label
									.list
		sb		label, LOS
									.nolist
									.endm
_JGEQ	.macro	label
									.list
		sb		label, GEQ
									.nolist
									.endm
_JLT		.macro	label
									.list
		sb		label, LT
									.nolist
									.endm
_JLEQ	.macro	label
									.list
		sb		label, LEQ
									.nolist
									.endm
_JGT		.macro	label
									.list
		sb		label, GT
									.nolist
									.endm
_JOV		.macro	label
									.list
		sb		label, OV
									.nolist
									.endm
_JTC		.macro	label
									.list
		sbf		label, TC
									.nolist
									.endm
_JBIO 	.macro	label
									.list
		sb		$+2, NBIO	; Workaround for the non-existent BIO condition code
		sb		label, UNC
									.nolist
									.endm


; Used by _ELSES and _OR_IFS below

_END_PRIOR_IF	.macro
									.nolist
		_CS_SWAP
		_ENDIF
									.endm

; Begin a short-circuit conditional of either type.
; Aliased as _CASE.

_COND	.macro
									.nolist
		_CS_PUSH	0				; Push a zero marker onto the control flow stack
									.list
									.endm

; Short circuit AND condition.

_AND_IF	.macro cond
									.nolist
		_IF :cond:
									.endm

; Resolve all _IFs or _ELSEs except the most recent one, back to the most recent _COND.
; Called by _ELSES and OR_IFS.

_END_PRIOR_IFS .macro
									.nolist
		_CS_SWAP
		.if	_CS_TOP == 0
			_CS_DROP
		.else
			_ENDIF
									.nolist
	 		_END_PRIOR_IFS			; Recursive
									.nolist
		.endif
									.list
									.endm

; Used in place of _ELSE for short-circuit AND.
; Does an _ELSE, then ends all _IFs or _ELSES except the most recent one, back to the most recent _COND.

_ELSES	.macro
									.nolist
		_ELSE						; Assemble an _ELSE
									.nolist
		_END_PRIOR_IFS				; Resolve all prior _IFs back to _COND
									.endm

; Used in place of _ENDIF for short-circuit AND, but only when there is no _ELSES clause.
; Resolves multiple _IFs or _ELSEs. Aliased as _ENDCASE.

_ENDIFS	.macro
									.nolist
		.if	_CS_TOP == 0
			_CS_DROP
		.else
			_ENDIF
									.nolist
			_ENDIFS					; Recursive
									.nolist
		.endif
									.list
									.endm

; Short circuit OR condition, except last

_OR_ELSE .macro cond
									.nolist
		_IF_NOT :cond:
									.endm

; Last short-circuit OR condition
; Does an _IF, then ends all _IFs except the most recent one, back to the most recent _COND.


_OR_IFS	.macro cond
									.nolist
		_IF	:cond:					; Assemble an _IF
									.nolist
		_END_PRIOR_IFS				; Resolve all prior _IFs back to _COND
									.endm


;------------------------------------------------------------
; CASE statement macros

; Typical use:

;	   _CASE
;          <test>
;		   _OF cc
;			   ...
;		   _ENDOF
;
;		   _OF_EQ src, dest			; OF_EQ uses CMP (word comparison)
;				...
;		   _ENDOF
;
;		   _OF_EQ_B src, dest		; OF_EQ_B uses CMP.B (byte comparison)
;			   ...
;		   _ENDOF
;		   ... ( default case )
;	   _ENDCASE

; IF cond1 ... ELSEIF cond2 ... ELSEIF cond3 ... ELSE ... ENDIF
; is written as
;
;	   _CASE
;          <test1>
;		   _OF cc1
;			   ...
;		   _ENDOF
;          <test2>
;		   _OF cc2
;			   ...
;		   _ENDOF
;          <test3>
;		   _OF cc3
;			   ...
;		   _ENDOF
;		   ...
;	   _ENDCASE


_CASE   .macro
									.nolist
		_COND			 			; Push a zero marker onto the control flow stack
									.endm

_OF		.macro cond
									.nolist
		_IF :cond:					; Assemble an _IF
									.endm

_OF_EQ	.macro dest, src			; dest is loc16, src is #16bitSigned
									.nolist
		cmp		dest, src
		_OF EQ						; Assemble an _IF EQ
									.endm

_OF_EQ_B .macro dest, src			; dest is AH or AL, src is #8bit
									.nolist
		cmpb	dest, src
		_OF EQ						; Assemble an _IF EQ
									.endm

_ENDOF  .macro
									.nolist
		_ELSE						; Resolve the previous cond'l jump and assemble an uncond'l jump
									.endm

_ENDCASE .macro
									.nolist
		_ENDIFS	  					; Resolve all the uncoditional jumps from the _ENDOFs
									.endm


;------------------------------------------------------------
; Counted loops

;	_FOR dest, src
;		...					; dest = src down to 1   (src 0 = 65536)
;	_NEXT_DEC dest

;	_FOR dest, src			; src must be even
;		...					; dest = src down to 2 in steps of 2   (src 0 = 65536)
;	_NEXT_DECD dest


_FOR	.macro dest, src			; dest is loc16, src is #16bit
		mov dest, src
									.nolist
		_REPEAT
									.endm

_NEXT_DEC .macro dest
		dec   dest
									.nolist
		_UNTIL Z
									.endm

_NEXT_DECD .macro dest
		dec   dest
		dec   dest
									.nolist
		_UNTIL Z
									.endm

; Byte versions of the above

;	_FOR_B dest, src
;		...					; dest = src down to 1   (src 0 = 265536)
;	_NEXT_DEC   dest

;	_FOR_B dest, src		; src must be even
;		...					; dest = src down to 2 in steps of 2   (src 0 = 65536)
;	_NEXT_DECD  dest


_FOR_B	.macro dest, src			; dest is loc16, src is #8bit
		movb dest, src
									.nolist
		_REPEAT
									.endm


