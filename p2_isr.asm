; Interrupt Routine for Port2 Pins Interrupt Request
; Program jumps here, when switch S1 is pressed
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file


;-------------------------------------------------------------------------------

			.ref	S1pressed				; reference the global variable

;-------------------------------------------------------------------------------
			.text
; Interrupt handler for P2ISR
P2ISR:		mov.b #1, &S1pressed			; set indicator that button is pressed
			bic.b #BIT1, &P2IFG				; clear interrupt flag of port2 pin1
			bic.b #BIT1, &P2IE				; disable interrupt from P2.1
			reti


;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".int42"                ; MSP430 PORT2_VECTOR Vector
            .short  P2ISR
