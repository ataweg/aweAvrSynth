
.ifndef MEEBLIP_VERSION
	.set	MEEBLIP_VERSION		= -1
.endif


;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------

.MACRO print_string			;  <variable>
	ldi	zl, low( @0*2)		; load z pointer
	ldi	zh, high( @0*2)
	call	debug_uart_string
.ENDMACRO

.MACRO print_midi_var			;  <variable>
	ldi	zl, low( STR_@0*2)	; load z pointer
	ldi	zh, high( STR_@0*2)
	ldi	r16, @0 - MIDICC	; the midi address
	lds	r17, @0			; the midi value
	call	midi_var_print
.ENDMACRO

.MACRO print_variable			;  <variable>
	ldi	zl, low( STR_@0*2)	; load z pointer
	ldi	zh, high( STR_@0*2)
	call	debug_uart_string
	lds	r16, @0
	call	debug_uart_write_byte
.ENDMACRO

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------

print_sound_parameter:
; lower knobs
	print_string	STR_LINE
	print_string	STR_LOWER

	print_midi_var	RESONANCE
	print_midi_var	CUTOFF
	print_midi_var	LFOFREQ
	print_midi_var	PANEL_LFOLEVEL
	print_midi_var	VCFENVMOD
	print_midi_var	PORTAMENTO
	print_midi_var	PULSE_KNOB
	print_midi_var	OSC_DETUNE

; upper knobs
	print_string	STR_NEXT_LINE
	print_string	STR_UPPER

	print_midi_var	KNOB_DCF_DECAY
	print_midi_var	KNOB_DCF_ATTACK
	print_midi_var	KNOB_AMP_DECAY
	print_midi_var	KNOB_AMP_ATTACK
.if MEEBLIP_VERSION == 303
	print_midi_var	ARP_THRESHOLD
	print_midi_var	ARP_RATE
.endif

; midi cc parameter
	print_string	STR_NEXT_LINE
	print_string	STR_MIDI_LINE

	print_midi_var	LFO2FREQ
	print_midi_var	PWMDEPTH
	print_midi_var	FMDEPTH
	print_midi_var	MIDI_KNOB_4
	print_midi_var	MIDI_KNOB_5
	print_midi_var	MIDI_KNOB_6
	print_midi_var	MIXER_BALANCE
	print_midi_var	MASTER_VOLUME
	call	debug_uart_write_crlf

	print_midi_var	ATTACKTIME2
	print_midi_var	DECAYTIME2
	print_midi_var	SUSTAINLEVEL2
	print_midi_var	RELEASETIME2

	call	debug_uart_write_crlf
	print_midi_var	ATTACKTIME
	print_midi_var	DECAYTIME
	print_midi_var	SUSTAINLEVEL
	print_midi_var	RELEASETIME

; switches
	print_string	STR_NEXT_LINE
	print_string	STR_SWITCHES_LINE

	print_variable	SW1
	print_variable	SW2
	print_variable	SW3
	print_variable	SW4
	call	debug_uart_write_crlf

; patch switches
	print_variable	PATCH_SWITCH1
	print_variable	PATCH_SWITCH2
	print_variable	PATCH_SWITCH3
	print_variable	PATCH_SWITCH4

; varibales
	print_string	STR_NEXT_LINE
	print_string	STR_VAR_LINE

	print_variable	DCOA_LEVEL
	print_variable	DCOB_LEVEL
	print_variable	LPF_I
	print_variable	SCALED_RESONANCE
.if USE_MASTER_VOLUME
	print_variable	VOLUME
	print_variable	VOLUME_X4
.endif	; USE_MASTER_VOLUME
	print_string	STR_NEXT_LINE
	call	debug_uart_write_crlf
	ret

;----------------------------------------------------------------------------
; the strings

STR_RESONANCE:		.db	0x0D, 0x0A, "RESONANCE        ", 0
STR_CUTOFF:		.db	0x0D, 0x0A, "CUTOFF           ", 0
STR_LFOFREQ:		.db	0x0D, 0x0A, "LFOFREQ          ", 0
STR_PANEL_LFOLEVEL:	.db	0x0D, 0x0A, "LFOLEVEL         ", 0
STR_VCFENVMOD:		.db	0x0D, 0x0A, "VCFENVMOD        ", 0
STR_PORTAMENTO:		.db	0x0D, 0x0A, "PORTAMENTO       ", 0
STR_PULSE_KNOB:		.db	0x0D, 0x0A, "PULSE_KNOB       ", 0
STR_OSC_DETUNE:		.db	0x0D, 0x0A, "OSC_DETUNE       ", 0

STR_KNOB_DCF_DECAY:	.db	0x0D, 0x0A, "FLT_DECAY        ", 0
STR_KNOB_DCF_ATTACK:	.db	0x0D, 0x0A, "FLT_ATTACK       ", 0
STR_KNOB_AMP_DECAY:	.db	0x0D, 0x0A, "AMP_DECAY        ", 0
STR_KNOB_AMP_ATTACK:	.db	0x0D, 0x0A, "AMP_ATTACK       ", 0
.if MEEBLIP_VERSION == 303
STR_ARP_THRESHOLD:	.db	0x0D, 0x0A, "ARP_THRESHOLD    ", 0
STR_ARP_RATE:		.db	0x0D, 0x0A, "ARP_RATE         ", 0
.endif

STR_LFO2FREQ:		.db	0x0D, 0x0A, "LFO2FREQ         ", 0
STR_PWMDEPTH:		.db	0x0D, 0x0A, "PWMDEPTH         ", 0
STR_FMDEPTH:		.db	0x0D, 0x0A, "FMDEPTH          ", 0
STR_MIDI_KNOB_4:	.db	0x0D, 0x0A, "MIDI_KNOB_4      ", 0
STR_MIDI_KNOB_5:	.db	0x0D, 0x0A, "MIDI_KNOB_5      ", 0
STR_MIDI_KNOB_6:	.db	0x0D, 0x0A, "MIDI_KNOB_6      ", 0
STR_MIXER_BALANCE:	.db	0x0D, 0x0A, "MIXER_BALANCE    ", 0
STR_MASTER_VOLUME:	.db	0x0D, 0x0A, "MASTER_VOLUME    ", 0

STR_ATTACKTIME2:	.db	0x0D, 0x0A, "DCF_ATTACKTIME   ", 0
STR_DECAYTIME2:		.db	0x0D, 0x0A, "DCF_DECAYTIME    ", 0
STR_SUSTAINLEVEL2:	.db	0x0D, 0x0A, "DCF_SUSTAINLEVEL ", 0
STR_RELEASETIME2:	.db	0x0D, 0x0A, "DCF_RELEASETIME  ", 0
STR_ATTACKTIME:		.db	0x0D, 0x0A, "DCA_ATTACKTIME   ", 0
STR_DECAYTIME:		.db	0x0D, 0x0A, "DCA_DECAYTIME    ", 0
STR_SUSTAINLEVEL:	.db	0x0D, 0x0A, "DCA_SUSTAINLEVEL ", 0
STR_RELEASETIME:	.db	0x0D, 0x0A, "DCA_RELEASETIME  ", 0

STR_SW1:		.db	0x0D, 0x0A, "SW1              ", 0
STR_SW2:		.db	0x0D, 0x0A, "SW2              ", 0
STR_SW3:		.db	0x0D, 0x0A, "SW3              ", 0
STR_SW4:		.db	0x0D, 0x0A, "SW4              ", 0
STR_PATCH_SWITCH1:	.db	0x0D, 0x0A, "PATCH_SWITCH1    ", 0
STR_PATCH_SWITCH2:	.db	0x0D, 0x0A, "PATCH_SWITCH2    ", 0
STR_PATCH_SWITCH3:	.db	0x0D, 0x0A, "PATCH_SWITCH3    ", 0
STR_PATCH_SWITCH4:	.db	0x0D, 0x0A, "PATCH_SWITCH4    ", 0

STR_DCOA_LEVEL:		.db	0x0D, 0x0A, "DCOA_LEVEL       ", 0
STR_DCOB_LEVEL:		.db	0x0D, 0x0A, "DCOB_LEVEL       ", 0
STR_SCALED_RESONANCE:	.db	0x0D, 0x0A, "SCLD_RESONANCE   ", 0
STR_LPF_I:		.db	0x0D, 0x0A, "LPF_I            ", 0
STR_VOLUME:		.db	0x0D, 0x0A, "VOLUME           ", 0
STR_VOLUME_X4:		.db	0x0D, 0x0A, "VOLUME_X4        ", 0

STR_NEXT_LINE:		.db	0x0D, 0x0A, 0, 0
STR_LINE:		.db	0x0D, 0x0A, "-----------------------", 0
STR_UPPER:		.db	0x0D, 0x0A, "----- upper knobs -----", 0
STR_LOWER:		.db	0x0D, 0x0A, "----- lower knobs -----", 0
STR_MIDI_LINE:		.db	0x0D, 0x0A, "----- midi control ----", 0
STR_SWITCHES_LINE:	.db	0x0D, 0x0A, "----- switches --------", 0
STR_VAR_LINE:		.db	0x0D, 0x0A, "----- variables -------", 0

STR_MIDI_CC_1:		.db	"   CC", 0
STR_MIDI_CC_2:		.db	" = ", 0

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------

startup_message:
	.db	0x0d, 0x0a, 0x0d, 0x0a
	.db	STARTUP_MSG, 0x0d, 0x0a
	.db	"   build ", __DATE__, " ", __TIME__, " ", 0x0d, 0x0a

.if !USE_ORIGINAL_RAW_OSC		; 0: use switch for Ringmodulator enable, set SW_ANTI_ALIAS to 1
	.db	"* remove raw waveform generators", 0x0d, 0x0a
.endif	; !USE_ORIGINAL_RAW_OSC

.if USE_RINGMODULATOR
	.db	"* using ring modulator", 0x0d, 0x0a
.endif	; USE_RINGMODULATOR

	.db	0, 0			; end of string

;----------------------------------------------------------------------------
;
;----------------------------------------------------------------------------

