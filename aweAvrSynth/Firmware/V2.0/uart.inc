;-------------------------------------------------------------------------------------------------------------------

	; cpu_frequency  			; system clock in Hz
	.equ	default_baud_rate = 38400	; Baudrate

	; Berechnungen
	.equ	ubrr_val   = ((cpu_frequency+default_baud_rate*8)/(default_baud_rate*16)-1)	; smart rounding
	.equ	baud_real  = (cpu_frequency/(16*(ubrr_val+1)))					; real baudrate
	.equ	baud_error = ((baud_real*1000)/default_baud_rate-1000)				; error in promille

.if ((baud_error>10) || (baud_error<-10))							; max. +/-10 promille error
  .error "Baudrate error greater than 1 percent, too high!"
.endif

;-------------------------------------------------------------------------------------------------------------------
;
;-------------------------------------------------------------------------------------------------------------------

usart_init_default_baudrate:
	ldi	R17, high( ubrr_val) & ~(1<<URSEL)
	ldi	R16, low( ubrr_val)

usart_init:
	; Set baud rate
	out	UBRRH, r17
	out	UBRRL, r16

	; Enable receiver, receiver interrupt and transmitter
;	ldi	r16, (1<<RXEN)|(1<<RXCIE)|(1<<TXEN)
	ldi	r16, (1<<TXEN)
	out	UCSRB, r16

	; Set frame format: 8data, 2stop bit
	ldi	r16, (1<<URSEL)|(1<<USBS)|(3<<UCSZ0)
	out	UCSRC, r16

	ret

;-------------------------------------------------------------------------------------------------------------------
;
;-------------------------------------------------------------------------------------------------------------------

; transmit value in r16

usart_transmit:
	; Wait for empty transmit buffer
	sbis	UCSRA, UDRE
	rjmp	usart_transmit

	; Put data (r16) into buffer, sends the data
	out	UDR, r16
	ret
;-------------------------------------------------------------------------------------------------------------------
;
;-------------------------------------------------------------------------------------------------------------------

; return value received in r16

usart_receive:
	; Wait for data to be received
	sbis	UCSRA, RXC
	rjmp	usart_receive

	; Get and return received data from buffer
	in	r16, UDR
	ret

;-------------------------------------------------------------------------------------------------------------------
;
;-------------------------------------------------------------------------------------------------------------------

usart_startup_message:
	ldi     zl, low( startup_message*2);	    ; load z pointer
	ldi     zh, high( startup_message*2);

; output string from flash, start address in zh:zl
usart_string:
	lpm				; get next character from flash
	and     r0, r0			; = zero?
	breq    usart_string_end	; if so -> end
usart_string_wait:
	sbis    UCSRA, UDRE		; wait until UDRE is ready
					; for next character
	rjmp    usart_string_wait
	out     UDR, r0
	adiw    zl:zh, 1		; increment pointer
	rjmp    usart_string		; process next character

usart_string_end:
	ret

;-------------------------------------------------------------------------------------------------------------------
;
;-------------------------------------------------------------------------------------------------------------------

;  value to write  in r16

usart_write_nibble:
	andi	r16, 0x0F
	cpi	r16, 10
	brlo	usart_write_nibble_1
	subi	r16, -7
usart_write_nibble_1:
	subi	r16, -'0'
	call	usart_transmit
	ret

usart_write_byte:
	push	r16
	swap	r16
	call	usart_write_nibble
	pop	r16
	call	usart_write_nibble

usart_write_crlf:
	ldi	r16, 0x0d
	call	usart_transmit
	ldi	r16, 0x0a
	call	usart_transmit

	ret

;-------------------------------------------------------------------------------------------------------------------
;
;-------------------------------------------------------------------------------------------------------------------
