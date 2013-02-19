;----------------------------------------------------------------------------
;
; siehe auch http://www.mikrocontroller.net/articles/AVR-Tutorial:_UART
;
;----------------------------------------------------------------------------

	; cpu_frequency  			; system clock in Hz
	.equ	default_baud_rate = 38400	; Baudrate

	; Berechnungen
	.equ	ubrr_val   = ((cpu_frequency+default_baud_rate*8)/(default_baud_rate*16)-1)	; smart rounding
	.equ	baud_real  = (cpu_frequency/(16*(ubrr_val+1)))					; real baudrate
	.equ	baud_error = ((baud_real*1000)/default_baud_rate-1000)				; error in promille

.if ((baud_error>10) || (baud_error<-10))							; max. +/-10 promille error
  .error "Baudrate error greater than 1 percent, too high!"
.endif

;----------------------------------------------------------------------------
; initialize uart
;----------------------------------------------------------------------------

debug_uart_init_default_baudrate:
	ldi	r17, high( ubrr_val) & ~(1<<URSEL)
	ldi	r16, low( ubrr_val)

debug_uart_init_baudrate:
	; Set baud rate
	out	UBRRH, r17
	out	UBRRL, r16

debug_uart_init:
	; Set frame format: 8 data, 2 stop bit
	ldi	r16, (1<<URSEL)|(1<<USBS)|(3<<UCSZ0)
	out	UCSRC, r16

.if !USE_SERIAL_OUT
; Enable only transmitter, stopp receiver
	ldi	r16, (1<<TXEN)
	out	UCSRB, r16
.else
; enable transmitter, receiver and receiver interrupt
	ldi	r16, (1<<TXEN)|(1<<RXEN)|(1<<RXCIE)
	out	UCSRB, r16
.endif
	ret

;----------------------------------------------------------------------------
; check for uart ready
;----------------------------------------------------------------------------

debug_uart_ready:
.if USE_SERIAL_OUT
tx_buf_not_empty:
	; wait for transmitter has send all data
	call	tx_buf_empty
	cpi	r16, 0				; is not empty?
	breq	tx_buf_not_empty		; repeat until empty
.endif

debug_entry_uart_busy:
	; wait for uart is in idle state (no data in uart)
	in	r16, UCSRA
	andi	r16, (1<<RXC)|(1<<TXC)|(1<<UDRE)
	cpi	r16, (0<<RXC)|(1<<TXC)|(1<<UDRE)
	brne	debug_entry_uart_busy
	ret

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------

debug_entry:
	push	r17
	push	r16

	call	debug_uart_ready

	call	debug_uart_init_default_baudrate ; set baudrate to 38400
	pop	r16
	pop	r17
	ret

;----------------------------------------------------------------------------
; reinitialize UART for midi
;----------------------------------------------------------------------------

debug_exit:
	push	r16

	; before change the baudrate wait for transmitter ready
	call	debug_uart_ready

	call	uart_init
	pop	r16
	ret

;----------------------------------------------------------------------------
; transmitter
;----------------------------------------------------------------------------

; transmit value in r16

debug_uart_transmit:
.if !USE_SERIAL_OUT
	; Wait for empty transmit buffer
	sbis	UCSRA, UDRE
	rjmp	debug_uart_transmit

	; Put data (r16) into buffer, sends the data
	out	UDR, r16
	ret
.else
	rjmp	uart_putc	; interrupt driven buffered serial output
.endif

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------

debug_byte:
	push	r16

	call	debug_uart_write_byte
	call	debug_uart_write_crlf

	pop	r16
	ret

;----------------------------------------------------------------------------
; send startup message
;----------------------------------------------------------------------------

debug_uart_startup_message:
	ldi	zl, low( startup_message*2);	    ; load z pointer
	ldi	zh, high( startup_message*2);

; output string from flash, start address in zh:zl
debug_uart_string:
	lpm	r16, Z+			; get next character from flash
	tst	r16			; = zero?
	breq	debug_uart_string_end	; if so -> end
.if !USE_SERIAL_OUT
debug_uart_string_wait:
	sbis	UCSRA, UDRE		; wait until UDRE is ready
					; for next character
	rjmp	debug_uart_string_wait
	out	UDR, r16
.else
	call	uart_putc
.endif
	rjmp	debug_uart_string	; process next character

debug_uart_string_end:
	ret

;----------------------------------------------------------------------------
; print nibble, byte, crlf
;----------------------------------------------------------------------------

;  value to write in r16

debug_uart_write_nibble:
	andi	r16, 0x0F
	cpi	r16, 10
	brlo	debug_uart_write_nibble_1
	subi	r16, -7
debug_uart_write_nibble_1:
	subi	r16, -'0'
	call	debug_uart_transmit
	ret

debug_uart_write_byte:
	push	r16
	swap	r16
	call	debug_uart_write_nibble
	pop	r16
	call	debug_uart_write_nibble
	ret

debug_uart_write_crlf:
	ldi	r16, 0x0d
	call	debug_uart_transmit
	ldi	r16, 0x0a
	call	debug_uart_transmit

	ret

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------
; from Atmel application note AVR204
;
; ***************************************************************************
; *
; * "bin2BCD16" - 16-bit Binary to BCD conversion
; *
; * This subroutine converts a 16-bit number (fbinH:fbinL) to a 5-digit
; * packed BCD number represented by 3 bytes (tBCD2:tBCD1:tBCD0).
; * MSD of the 5-digit number is placed in the lowermost nibble of tBCD2.
; *
; * Number of words	: 25
; * Number of cycles	: 751/768 (Min/Max)
; * Low registers used	: 3 (tBCD0, tBCD1, tBCD2)
; * High registers used	: 4(fbinL, fbinH, cnt16a, tmp16a)
; * Pointers used	: Z
; *
; ***************************************************************************

; ***** Subroutine Register Variables

	.equ	AtBCD0	= 12		; address of tBCD0
	.equ	AtBCD1	= 13		; address of tBCD2
	.equ	AtBCD2	= 14		; address of tBCD4

	.def	tBCD0	= r12		; BCD value digits 1 and 0
	.def	tBCD1	= r13		; BCD value digits 3 and 2
	.def	tBCD2	= r14		; BCD value digit 4

	.def	fbinL	= r16		; binary value Low byte
	.def	fbinH	= r17		; binary value High byte
	.def	cnt16a	= r18		; loop counter
	.def	tmp16a	= r19		; temporary value

; ***** Code

bin2BCD16:
	push	r12
	push	r13
	push	r14

	ldi	cnt16a, 16	; Init loop counter
	clr	tBCD2		; clear result (3 bytes)
	clr	tBCD1
	clr	tBCD0
	clr	ZH		; clear ZH
bBCDx_1:
	lsl	fbinL		; shift input value
	rol	fbinH		; through all bytes
	rol	tBCD0		;
	rol	tBCD1
	rol	tBCD2
	dec	cnt16a		; decrement loop counter
	brne	bBCDx_2		; if counter not zero

	movw	r16, tBCD0
	mov	r18, tBCD2
	pop	r14
	pop	r13
	pop	r12
	ret			;   return

bBCDx_2:
	ldi	ZL, AtBCD2+1	; Z points to result MSB + 1
bBCDx_3:
	ld	tmp16a, -Z	; get (Z) with pre-decrement
	subi	tmp16a, -$03	; add 0x03
	sbrc	tmp16a, 3	; if bit 3 not clear
	st	Z, tmp16a	; 	store back
	ld	tmp16a, Z	; get (Z)
	subi	tmp16a, -$30	; add 0x30
	sbrc	tmp16a, 7	; if bit 7 not clear
	st	Z, tmp16a	; 	store back
	cpi	ZL, AtBCD0	; done all three?
	brne	bBCDx_3		; loop again if not
	rjmp	bBCDx_1

	.undef	tBCD0
	.undef	tBCD1
	.undef	tBCD2
	.undef	fbinL
	.undef	fbinH
	.undef	cnt16a
	.undef	tmp16a

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------

; print a 16 bit number in r17:r16, here we have only 7bit numbers
print_number:
	call	bin2BCD16		; bcd number in r18:r17:r16
	push	r16
	mov	r16, r17
	ldi	r17, 2			; we print a least two digits
	andi	r16, 0x0f
	breq	print_number_1
	call	debug_uart_write_nibble
	inc	r17
print_number_1:
	pop	r16
	call	debug_uart_write_byte
	mov	r16, r17
	ret

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------

midi_var_print:
	push	r17			; the midi value
	push	r16			; the midi address
	call	debug_uart_string		; print the midi parameter name
	mov	r16,r17
	call	debug_uart_write_byte	; print the midi value as hexadecimal byte

; print the midi address and value
	ldi	zl, low( STR_MIDI_CC_1*2)
	ldi	zh, high( STR_MIDI_CC_1*2)
	call	debug_uart_string

	pop	r16			; get the midi address
	clr	r17
	call	print_number		; print 16 bit number in r17:r16
	cpi	r17, 2
	breq	midi_var_print_1

	ldi	r16, ' '		; print trailing blank
	call	debug_uart_transmit

midi_var_print_1:
	ldi	zl, low( STR_MIDI_CC_2*2)
	ldi	zh, high( STR_MIDI_CC_2*2)
	call	debug_uart_string

	pop	r16			; get the midi value
	lsr	r16
	clr	r17
	call	print_number		; print 16 bit number in r17:r16
	ret

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------
