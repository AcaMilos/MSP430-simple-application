; Interrupt Routine for UART interrupt
; Program jumps here, when the first digit is passed using UCA1TXBUF
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file


;-------------------------------------------------------------------------------

			.ref	SendDigitToUART			; reference the extern function
			.ref	S1pressed				; reference the global variable
			.ref	debounceTimeS1			; reference the global variable
			.ref	digitSec				; reference the global variable

;-------------------------------------------------------------------------------
			.text
; Interrupt handler for UARTISR
UARTISR:	mov.w	&UCA1IV, R8				;
			cmp.w	#USCI_UCTXIFG, R8		; check if UCA1IV == USCI_UCTXIFG
			jz 		checkUART_REQ			; if it is => check UARTrequest indicator
			jmp		EXIT_ISR				; if it's not => exit from ISR

checkUART_REQ:
			mov.b	&S1pressed, R8			;
			cmp.w	#1, R8					; check if UARTrequest == 1
			jnz		EXIT_ISR				; if it's not => exit from ISR

			mov.w	&digitSec, R12			; prepare R12 to send argument digitSec to function sendSecondDigit
			call	#SendDigitToUART		; call extern function sendSecondDigit

			mov.b	#0, &S1pressed			; clear UART request for sending result to USB
			mov.w	#0, &debounceTimeS1		; clear debounce time used for debouncing SW1
			bic.b	#BIT1, &P2IFG			; clear P2.1 (SW1) Interrupt Flag
			bis.b	#BIT1, &P2IE			; enable again interrupts on P2.1 (SW1)

EXIT_ISR:	reti							; exit from UART1ISR


;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".int46"                ; MSP430 USCI_A1_VECTOR Vector
            .short  UARTISR
