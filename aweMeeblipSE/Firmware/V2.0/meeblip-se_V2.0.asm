;-----------------------------------------------------------------------------
;
; meeblip se V2.02
; Axel Werner
;
; requires asm2 assembler, asm leads to some undefined variable errors
;
; Changelog
;
; 2012.01.09	AWe	integrate changes from meeblip-se-v2.asm (V2.04 2012.01.05) from James Grahame
;			(create a new version aweMeeblipSE_V2.04.asm)
;			remove code parts previously controlled by the switches
;			 * USE_ORIGINAL_MUL32X16, USE_ORIGINAL_MUL8X8S, USE_ORIGINAL_DCA,
;			 * USE_OSCBOFF_LEVEL_CORRECTION, USE_ORIGINAL_WAVETABLES, USE_ORIGINAL_BANDLIMITED_PWM,
;			 * USE_BANDLIMITED_DCO_A, USE_BANDLIMITED_DCO_B, USE_ORIGINAL_TAB_VCA
;			set Q_OFFSET to 8, previous value was 24
;			remove patch switch for SW_OSCA_WAVE_SIN and SW_OSCB_WAVE_SIN
;			correct LED flash frequency for extended button functions
;			correct some typos
; 2012.01.04	AWe	use better rounding for calculated table TAB_VCF
;			move calculation of scaled_resonance from isr to mainloop
;			change q offset from 32 to 24, lower values will cause clipping
;			change index of some midi cc
;			change bit SW_MIX_RING position of in variable PATCH_SWITCH3
;			for extended button functions let LED blink faster
; 2011.12.22	AWe	bugfix: using of option USE_INIT_ENV_GENERATORS can cause hearable clicks, so don't use it
;			bugfix: correct routine mul32x16 with hw multiplier and in  volume control function
;			bugfix: use mulsu for LFO modulation and keybard tracking
;			for LFO2 change Amin to 0 and Amx to 255, because wee have a PWM_DEPTH control via MIDI
;			add more sample patches to the eeprom
; 2011.12.15	AWe	integrate bugfixes on meeblip-se-v2.asm (V2.02 2011.12.13) from James Grahame
;			I found and reported these bugfixes ;-)
;			save/load more midi cc parameter to eeprom (enhanced patches) (USE_EXTENDED_PATCH)
;			remove limits of RESONANCE knob, now no limits 0..254 (USE_ORIGINAL_RESONANCE_LIMIT)
;			correct limits of PULSE_KNOB, PULSE_KNOB_LIMITED, now 4..127 (USE_ORIGINAL_PWM_LIMIT)
;			optimize usage of "lpm" opcode; hint: "lpm" has not the same opcode like "lpm r0,z"
;			use patch 0 as a default patch, it is save for over writing (USE_READ_ONLY_PATCH_ZERO)
;			in DCA function use a HW-multipier, not the shift-and-add algorithm for the
;			   16s*8u multiplication (USE_ORIGINAL_DCA)
;			update env parameter from midi-cc or switches and knobes (USE_ORIGINAL_ENVELOPE_CONTROL)
;			map ATTACKTIME, ATTACKTIME2 to KNOB_AMP_ATTACK, KNOB_DCF_ATTACK and vice versa (USE_ORIGINAL_ENVELOPE_CONTROL)
;			level adjustment, when osc b is off (USE_OSCBOFF_LEVEL_CORRECTION)
;			master volume control - VOLUME 0..254 or -36db..+12db (USE_MASTER_VOLUME)
;			redesign PWM signal generation without using falling sawtooth waveform (USE_ORIGINAL_BANDLIMITED_PWM)
;			mixer balance also for ringmodulator input (USE_RINGMODULATOR_BALANCE)
; 2011.12.11	AWe	integrate changes from meeblip-se-v2.asm (V2.01 2011.12.01) from James Grahame
;			(create a new version aweMeeblipSE_V2.01.asm)
;			add eeprom content for patch 0
;			correct some bugs
;			* when osc b is disabled, set its output value to zero, not to $80
;			* save mixer output value (r16) to OSC_OUT_L after scaling down
;			* initialize some sound parameter with zero instead of 255
;			* after power on force processing the key changes by set the old state to the
;			  inverted value of the current switch state
; 			adjust ringmodulator output level
; 2011.12.07	AWe	improve output level of mixer
;			recalculate bandlimited waveform so that they have the same average amplitude
;			 now we have 12 subtables for each waveform; one subtable is used for 8 notes
;			osc A and osc B use their own WAVETABLE variable
;			remove dco_fm
;			replace switch "osc fm" and "bandlimited osc" with "mixer ringmodulator" and "transpose"
; 2011.12.03	AWe	use DCOs from avrSynth, but without sine waveform
;			fix bug: adjust level of pwm waveform by adding a duty cycle depend offset
;			change slope of wavetable sawtooth from falling to rising
; 2011.11.29	AWe	add Ringmodulator
;			change mixer
;			use improved noise generator
;			add more switches, controlled thru midi cc $50..$5F
;			add midi cc for contolling some sound parameter
;			correct bug: msb of midi cc data is bit 6, not bit 7
;			use FMDEPTH to control the dco A fm modulation
;			initialize varibales RESONANCE, PORTAMENTO, ENV_FRAC_L2, ENV_FRAC_H2, ENV_INTEGR2, ENVPHASE2,
;			read some sound paramters for eeprom during startup: LFO2FREQ, PWMDEPTH, DCOA_LEVEL, DCOB_LEVEL
;			correct bug: SW_SUSTAIN did only work when SW_KNOB_SHIFT is set to upper
;			remove dead code
;			set Amax for LFO2 to 255, previos value was 100
;			add features from LFO also to LFO2
;			correct sustain level after decay is done
;			clear env variables when envelope starts
;			add "transpose" and keyboard tracking switch
;			use modified dcf amount control with range -128..+127
;			add dca mode switch 0 (gate), 1 (env)
; 2011.11.24	AWe	code beautification, replace blanks with tabs (tab width is 8)
;			add more comments
;			comment/define out useless opcodes, search ";(1)"
;			optimize function TAB_BYTE, value returned now in r16
;			optimize access to program memory in functions LOAD_32BIT, TAB_WROD
;
; Adaptions to my avrSynth Hardware
; 2011.11.06	AWe	move LED from Atmega pin PD1 to PB6
; 2011.09.06	AWe	invert state of switches
; 2011.08.19	AWe	swap ATmega32 pins: DAC_RW/PD2 with DAC_AB/PD3
;
;-----------------------------------------------------------------------------
;                     _     _  _
;                    | |   | |(_)
;   ____  _____ _____| |__ | | _ ____      ___ _____
;  |    \| ___ | ___ |  _ \| || |  _ \    /___) ___ |
;  | | | | ____| ____| |_) ) || | |_| |  |___ | ____|
;  |_|_|_|_____)_____)____/ \_)_|  __/   (___/|_____)
;                             |_|
;
;   meeblip se V2 - the hackable digital synthesiser
;
;	For Version 2.0 Meeblip hardware (with save/load switches and patch memory)
;
;-----------------------------------------------------------------------------
;
;					Changelog
;
;V2.04 2012.01.05	- Added maximum resonance table
;			- Envelopes no longer restart on note off release if they're stopped
;			- Replace MUL8X8S routine with hardware multiply
;			- Updated LPM calls throughout code
;			- Implement 2 byte offsets for all table lookups
;			- New bandlimited waveforms with matched average amplitude (Axel Werner)
;			- New 32x16 bit multiply, 32 bit load (AW)
;			- Use signed multiply in DCA (AW)
;			- New oscillator mix routine (AW)
;			- New TAB_VCF, TAB_VCA and TIMETORATE tables (AW)
;			- Fix envelope decay bug (AW)
;V2.03 2011.12.20	- Added MIDI Velocity to VCF Envelope Modulation
;			- Changed maximum resonance and resonance scaling limits
;			- Updated oscillator mix levels because of mix clipping
;V2.02 2011.12.12 	- Corrected code to save pre-filtered waveform
;V2.01 2011.12.01	- Added inverse sawtooth waveform
;			- Fixed wavetable overflow in variable pulse calculation
;			- Increase Portamento rate
;			- Enable patch load/save and MIDI save functions (careful with V1 hardware - you have to use the DIP switches)
;			- Increase Portamento rate
;			- Debugged MIDI CC switch loading routine
;			- Removed unnecessary opcodes throughout code
;V2.00 2011.11.23	- Bandlimited pulse and PWM wave built from out-of-phase ramps
;			- Bandlimited square, saw and triangle oscillators
;			- Patch state engine (independent of front panel controls)
;			- New knob scan routine (scans only the most recently converted analog channel)
;			- Dual ADSD envelopes
;			- PWM sweep switch and PWM rate knob (limited between 0-50% pulse duty cycle)
;			- Filter envelope amount knob added
;			- MIDI control of all parameters
;			- Reorganized front panel
;			- MIDI knob and switch CC control
;			- DCA gate, LFO sync, modulation wheel and filter key track now always on
;			- LFO enable switch added
;			- anti-alias switch added - off uses bandlimited wave 0, on uses bandlimited wavetable 0..15, based on note
;			- Added variable names for switches instead of using bit ops - allows easier reorganization of panel
;			- FM switch - off or 100%
;			- New sample rate - 36363.63636 Hz, approximately 440 instructions per sample
;V1.05 2011.02.04	- Save dual parameter knob values to eeprom and reload on power up.
;V1.04 2011.01.19	- MIDI CC RAM table added.
;			- PWM waveform with dedicated fixed sweep LFO
;			- 8-bit multiplies optimized in main loop
;			- LFO Sync switch now retriggers LFO on each keypress
;			- Initialize FM Depth to zero on power up
;V1.03			- VCA and VCF level tables extended to reduce stairstepping
;V1.02			- Flip DAC write line high immediately after outputting sample
;V1.01			- Optimized DCOA+DCOB summer, outputs signed value
;V1.00			- Power/MIDI status LED remains on unless receiving MIDI
;			- Sustain level of ADSR envelope is exponentially scaled
;			- Non-resonant highpass filter implemented
;			- Filter Q level compensation moved outside audio sample calc interrupt
;			- Filter calculations increased to 16x8-bit to reduce noise floor
;			- DCA output level calculations are rounded
;			- Mod wheel no longer overrides LFO level knob when less than knob value
;V0.90			- Initial release
;
;-----------------------------------------------------------------------------
;
;	MeeBlip Contributors
;
;	Jarek Ziembicki - Created the original AVRsynth, upon which this project is based.
;	Laurie Biddulph - Worked with Jarek to translate his comments into English, ported to Atmega16
;	Daniel Kruszyna - Extended AVRsynth (several of his ideas are incorporated in MeeBlip)
;	Julian Schmidt  - Original filter algorithm
; 	Axel Werner	- Code optimization, bug fixes and new bandlimited waveforms
;	James Grahame   - Ported and extended the AVRsynth code to MeeBlip hardware.
;
;-----------------------------------------------------------------------------
;
;	Port Mapping
;
;	PA0..7		8 potentiometers
;	PB0-PB4		Control Panel Switches - ROWS
;	PB5-PB7		ISP programming header
;	PB7		DAC LDAC signal (load both DAC ports synchronously)
;	PC0-PC7		DATA port for DAC
;	PD0		RxD (MIDI IN)
;	PD1		Power ON/MIDI LED
;	PD2		Select DAC port A or B
;	PD3		DAC Write line
;	PD4-PD7		Control Panel Switches - COLUMNS
;
;	Timers
;
;	Timer0		not used
;	Timer1		Time counter: CK/400      --> TCNT1
;	Timer2		Sample timer: (CK/8) / 32 --> 36363.63636 Hz
;
;-----------------------------------------------------------------------------

	.NOLIST
	.INCLUDE "m32def.inc"
	.LIST
	.LISTMAC

;-----------------------------------------------------------------------------

	.equ	MEEBLIP_SE_ORIGINAL_CODE	= 0
	.equ	MEEBLIP_SE_OPIMIZED_CODE	= 0

	.equ	SKIP_FILTER			= 0
	.equ	SKIP_AMP			= 0

.if MEEBLIP_SE_ORIGINAL_CODE
	.equ	MEEBLIP_V1_27_HW		= 1	; 0: use my cloned&modified pcb version 1.27
	.equ	USE_UART			= 0

.if MEEBLIP_SE_OPIMIZED_CODE
; change these values carefully
	.equ	KEEP_BUGS			= 0

	.equ	USE_ORIGINAL_MIXER		= 1	; 0: use alternative mixer
	.equ	USE_ORIGINAL_NOISEGEN		= 0	; 0: new algorithm
	.equ	USE_ORIGINAL_VCFENVMOD		= 0	; 0: range -128..+127
	.equ	USE_ORIGINAL_SWITCH_MAP		= 1
	.equ	USE_ORIGINAL_TAB_VCF		= 0	; 0: improved, calculated tables
	.equ	USE_ORIGINAL_DCO_FM		= 1
	.equ	USE_ORIGINAL_RAW_OSC		= 1
	.equ	USE_ORIGINAL_Q_CALC		= 1
	.equ	Q_OFFSET			= 8
	.equ	USE_ORIGINAL_RESONANCE_LIMIT	= 0	; 0: remove ranger limitation from v2.02
	.equ	USE_ORIGINAL_PWM_LIMIT		= 0	; 0: allow duty cycle near 50%
	.equ	USE_ORIGINAL_MIXER_LEVEL	= 1	; 0: will cause signal overflow
	.equ	USE_ORIGINAL_ENVELOPE_CONTROL	= 1

	.equ	USE_MORE_SWITCHES		= 0
	.equ	USE_DCA_MOD_SWITCH		= 0 & USE_MORE_SWITCHES
	.equ	USE_EXTENDED_BUTTON_FUNCTIONS	= 0
	.equ	USE_EXTENDED_LFO2		= 0 & USE_MORE_SWITCHES
	.equ	USE_EXTENDED_MIDI_CONTROL	= 0 & USE_MORE_SWITCHES
	.equ	USE_INIT_ENV_GENERATORS		= 0
	.equ	USE_INIT_LFO2			= 0
	.equ	USE_KBD_SYNC			= 0 & USE_MORE_SWITCHES
	.equ	USE_KBD_TRACK_SWICTH		= 0 & USE_MORE_SWITCHES
	.equ	USE_MIDI_FMDEPTH		= 0
	.equ	USE_MIDI_PWMDEPTH		= 0
	.equ	USE_SUSTAIN_UPDATE_ON_NOTE	= 0
	.equ	USE_SYNC_LFO2			= 0
	.equ	USE_TRANSPOSE_SWITCH		= 0 & USE_MORE_SWITCHES

	.equ	USE_EXTENDED_PATCH		= 0 & USE_MORE_SWITCHES
	.equ	USE_MASTER_VOLUME		= 0
	.equ	USE_READ_ONLY_PATCH_ZERO	= 0
	.equ	USE_RINGMODULATOR		= 1 & !USE_ORIGINAL_MIXER & !USE_ORIGINAL_RAW_OSC & USE_MORE_SWITCHES
	.equ	USE_RINGMODULATOR_BALANCE	= 0

	.equ	PARAM_DEAD_ZONE			= 5

;-----------------------------------------------------------------------------
.else	; MEEBLIP_SE_OPIMIZED_CODE
; don't change these values
	.equ	KEEP_BUGS			= 1

	.equ	USE_ORIGINAL_MIXER		= 1	; 0: use alternative mixer
	.equ	USE_ORIGINAL_NOISEGEN		= 1	; 0: new algorithm
	.equ	USE_ORIGINAL_VCFENVMOD		= 1	; 0: range -128..+127
	.equ	USE_ORIGINAL_SWITCH_MAP		= 1
	.equ	USE_ORIGINAL_TAB_VCF		= 1	; 0: improved, calculated tables
	.equ	USE_ORIGINAL_DCO_FM		= 1
	.equ	USE_ORIGINAL_RAW_OSC		= 1
	.equ	USE_ORIGINAL_Q_CALC		= 1
	.equ	Q_OFFSET			= 8
	.equ	USE_ORIGINAL_RESONANCE_LIMIT	= 1	; 0: remove ranger limitation from v2.02
	.equ	USE_ORIGINAL_PWM_LIMIT		= 1	; 0: allow duty cycle near 50%
	.equ	USE_ORIGINAL_MIXER_LEVEL	= 1	; 0: will cause signal overflow
	.equ	USE_ORIGINAL_ENVELOPE_CONTROL	= 1

	.equ	USE_MORE_SWITCHES		= 0
	.equ	USE_DCA_MOD_SWITCH		= 0 & USE_MORE_SWITCHES
	.equ	USE_EXTENDED_MIDI_CONTROL	= 0
	.equ	USE_EXTENDED_BUTTON_FUNCTIONS	= 0 & USE_MORE_SWITCHES
	.equ	USE_EXTENDED_LFO2		= 0 & USE_MORE_SWITCHES
	.equ	USE_INIT_ENV_GENERATORS		= 0
	.equ	USE_INIT_LFO2			= 0
	.equ	USE_KBD_SYNC			= 0 & USE_MORE_SWITCHES
	.equ	USE_KBD_TRACK_SWICTH		= 0 & USE_MORE_SWITCHES
	.equ	USE_MIDI_FMDEPTH		= 0
	.equ	USE_MIDI_PWMDEPTH		= 0
	.equ	USE_SUSTAIN_UPDATE_ON_NOTE	= 0
	.equ	USE_SYNC_LFO2			= 0
	.equ	USE_TRANSPOSE_SWITCH		= 0 & USE_MORE_SWITCHES

	.equ	USE_EXTENDED_PATCH		= 0 & USE_MORE_SWITCHES
	.equ	USE_MASTER_VOLUME		= 0
	.equ	USE_READ_ONLY_PATCH_ZERO	= 0
	.equ	USE_RINGMODULATOR		= 0 & !USE_ORIGINAL_MIXER & !USE_ORIGINAL_RAW_OSC & USE_MORE_SWITCHES
	.equ	USE_RINGMODULATOR_BALANCE	= 0

	.equ	PARAM_DEAD_ZONE			= 5
.endif ; MEEBLIP_SE_OPIMIZED_CODE

;-----------------------------------------------------------------------------
.else	; MEEBLIP_SE_ORIGINAL_CODE
	.equ	MEEBLIP_V1_27_HW		= 0	; 0: use my cloned&modified pcb version 1.27
	.equ	USE_UART			= 1
	.equ	KEEP_BUGS			= 0

	.equ	USE_ORIGINAL_MIXER		= 0	; 0: use alternative mixer
	.equ	USE_ORIGINAL_NOISEGEN		= 0	; 0: new algorithm
	.equ	USE_ORIGINAL_VCFENVMOD		= 0	; 0: range -128..+127
	.equ	USE_ORIGINAL_SWITCH_MAP		= 0
	.equ	USE_ORIGINAL_TAB_VCF		= 0	; 0: improved, calculated tables
	.equ	USE_ORIGINAL_DCO_FM		= 1 	; 0: use switch for transpose
	.equ	USE_ORIGINAL_RAW_OSC		= 0	; 0: use switch for Ringmodulator enable, set SW_ANTI_ALIAS to 1
	.equ	USE_ORIGINAL_Q_CALC		= 0
	.equ	Q_OFFSET			= 16
	.equ	USE_ORIGINAL_RESONANCE_LIMIT	= 0	; 0: knob value is limited to 94% ($F6)
	.equ	USE_ORIGINAL_PWM_LIMIT		= 0	; 0. duty cycle 1.5%..42,7% (4..109), 1: 1.5%..50% (4..127)
	.equ	USE_ORIGINAL_MIXER_LEVEL	= 1	; 0: 6 db more output level, 1: (recommended)
	.equ	USE_ORIGINAL_ENVELOPE_CONTROL	= 0

	.equ	USE_MORE_SWITCHES		= 1
	.equ	USE_DCA_MOD_SWITCH		= 1 & USE_MORE_SWITCHES
	.equ	USE_EXTENDED_BUTTON_FUNCTIONS	= 1
	.equ	USE_EXTENDED_LFO2		= 0 & USE_MORE_SWITCHES	; 1: add waveform selection to lfo2 tri, sqr, random
	.equ	USE_EXTENDED_MIDI_CONTROL	= 1 & USE_MORE_SWITCHES
	.equ	USE_INIT_ENV_GENERATORS		= 0 			; 1: can cause clicks
	.equ	USE_INIT_LFO2			= 1
	.equ	USE_KBD_SYNC			= 1 & USE_MORE_SWITCHES
	.equ	USE_KBD_TRACK_SWICTH		= 1 & USE_MORE_SWITCHES
	.equ	USE_MIDI_FMDEPTH		= 1
	.equ	USE_MIDI_PWMDEPTH		= 1
	.equ	USE_SUSTAIN_UPDATE_ON_NOTE	= 1
	.equ	USE_SYNC_LFO2			= 1
	.equ	USE_TRANSPOSE_SWITCH		= 1 & USE_MORE_SWITCHES

	.equ	USE_EXTENDED_PATCH		= 1 & USE_MORE_SWITCHES	; 0: 16 byte patch, 1: 48 byte patch
	.equ	USE_MASTER_VOLUME		= 1                    	; 0: master volume control; 1: control it via MIDI cc
	.equ	USE_READ_ONLY_PATCH_ZERO	= 1			; 0: can overwrite of patch 0, 1: protect patch 0 for overwriting
	.equ	USE_RINGMODULATOR		= 1 & !USE_ORIGINAL_MIXER & !USE_ORIGINAL_RAW_OSC & USE_MORE_SWITCHES
	.equ	USE_RINGMODULATOR_BALANCE	= 1

	.equ	PARAM_DEAD_ZONE			= 3
.endif	; MEEBLIP_SE_ORIGINAL_CODE

;-----------------------------------------------------------------------------

	.SET cpu_frequency	= 16000000
	.SET baud_rate		= 31250
	.SET KBDSCAN		= 6250

.if MEEBLIP_V1_27_HW
	.set PORT_MIDI_LED	= PORTD
	.set MIDI_LED		= 1	; PD1

	.set DAC_AB		= 2	; PD2
	.set DAC_WR		= 3	; PD3

	.set LOAD_BUTTON	= 2
	.set SAVE_BUTTON	= 1
	.set MIDI_BUTTON	= 0
.else	; MEEBLIP_V1_27_HW
	.set PORT_MIDI_LED	= PORTB
	.set MIDI_LED		= 6	; PB6

	.set DAC_WR		= 2	; PD2
	.set DAC_AB		= 3	; PD3

	.set LOAD_BUTTON	= 1
	.set SAVE_BUTTON	= 2
	.set MIDI_BUTTON	= 3
.endif	; MEEBLIP_V1_27_HW

;-----------------------------------------------------------------------------
;		V A R I A B L E S   &  D E F I N I T I O N S
;-----------------------------------------------------------------------------
;registers:

			; r0		; multiplierer result regsiter
			; r1		; dto.

;current phase of DCO A:
	.DEF PHASEA_0	= r2		; used in timer0 ISR, not saved on stack
	.DEF PHASEA_1	= r3		; dto.
	.DEF PHASEA_2	= r4		; dto.

;current phase of DCO B:
	.DEF PHASEB_0	= r5		; dto.
	.DEF PHASEB_1	= r6		; dto.
	.DEF PHASEB_2	= r7		; dto.
.if MEEBLIP_SE_ORIGINAL_CODE
	.DEF ZERO	= r8		; dto.

;DCF:
	.def a_L	= r9		; used in timer0 ISR, not saved on stack
	.def a_H	= r10		; also used in sate variable filter, keeps lowpass.
	.def temp_SREG	= r11
.else	; MEEBLIP_SE_ORIGINAL_CODE

;DCF:
	.def a_L	= r8		; used in timer0 ISR, not saved on stack
	.def a_H	= r9		; also used in sate variable filter, keeps lowpass.
	.DEF ZERO	= r10		; dto.
	.def temp_SREG	= r11

.endif	; MEEBLIP_SE_ORIGINAL_CODE
			; r12		; used in state variable filter, keeps bandpass
			; r13		; so used in timer0 ISR, not saved on stack

	.DEF OSC_OUT_L	= r14		; pre-filter audio
	.DEF OSC_OUT_H	= r15		; temporary used in ISR, not saved on stack

	.def LDAC	= r16		; general purpose, used in ISR, saved on stack
	.def HDAC	= r17		; dto

	.def z_L	= r18		; temporary used in ISR, saved on stack
	.def z_H	= r19		; dto.

			; r20		; general purpose, used in ISR, saved on stack
			; r21		; dto.
			; r22		; dto.
			; r23		; dto.
			; r24		; dto.
			; r25		; general purpose
			; r26		; general purpose
			; r27		; general purpose
			; r28		; general purpose
			; r29		; general purpose

	.def temp	= r30		; general purpose, used in ISR, saved on stack
	.def temp2	= r31		; dto


;RAM (0060h...025Fh):

	.DSEG

;MIDI:
MIDIPHASE:		.BYTE 1
MIDICHANNEL:		.BYTE 1
MIDIDATA0:		.BYTE 1
MIDIVELOCITY:		.BYTE 1
MIDINOTE:		.BYTE 1
MIDINOTEPREV:		.BYTE 1		; buffer for MIDI note
MIDIPBEND_L:		.BYTE 1		;\
MIDIPBEND_H:		.BYTE 1		;/ -32768..+32766

;current sound parameters:
LFOLEVEL:		.BYTE 1		; 0..255
KNOB_SHIFT:		.BYTE 1		; 0= unchanged 255= changed state (not used)
POWER_UP:		.BYTE 1		; 255 = Synth just turned on, 0 = normal operation
KNOB0_STATUS:		.BYTE 1		; Each byte corresponds to a panel knob.
KNOB1_STATUS:		.BYTE 1		; 0 = pot not updated since Knob Shift switch change
KNOB2_STATUS:		.BYTE 1		; 1 = pot has been updated.
KNOB3_STATUS:		.BYTE 1
KNOB4_STATUS:		.BYTE 1
KNOB5_STATUS:		.BYTE 1
KNOB6_STATUS:		.BYTE 1
KNOB7_STATUS:		.BYTE 1
SWITCH1:		.BYTE 1
SWITCH2:		.BYTE 1
OLD_SWITCH1:		.BYTE 1		; Previous switch values (used to flag switch changes)
OLD_SWITCH2:		.BYTE 1

; Switch value currently used (from front panel, MIDI or last loaded patch)
PATCH_SWITCH1:		.BYTE 1
	.equ SW_KNOB_SHIFT		= 0	; 0=lower, 1=upper
	.equ SW_OSC_FM			= 1	;
	.equ SW_LFO_RANDOM		= 2	; 0=norm, 1=rand
	.equ SW_LFO_WAVE		= 3	; 0=tri, 1=squ
	.equ SW_FILTER_MODE		= 4	; 0=lp, 1=hp
	.equ SW_DISTORTION		= 5	; 0=off, 1=on
	.equ SW_LFO_ENABLE		= 6	;
	.equ SW_LFO_DEST		= 7	; 0=DCF, 1=DCO

PATCH_SWITCH2:		.BYTE 1
	.equ SW_ANTI_ALIAS		= 0	;
	.equ SW_OSCB_OCT		= 1	; 0=down, 1=up
	.equ SW_OSCB_ENABLE		= 2	; 0=off, 1=on
	.equ SW_OSCB_WAVE		= 3	; 0=saw, 1=squ
	.equ SW_SUSTAIN			= 4	;
	.equ SW_OSCA_NOISE		= 5	; 0=off, 1=on
	.equ SW_PWM_SWEEP		= 6	; 0=off, 1=LFO2
	.equ SW_OSCA_WAVE		= 7	; 0=saw, 1=squ

.if USE_MORE_SWITCHES
PATCH_SWITCH3:		.BYTE 1
	.equ SW_USER_3_0		= 0	;
	.equ SW_USER_3_1		= 1	;
	.equ SW_MIX_RING		= 2	; 0=Mixer(default), 1=Ringmodulator
	.equ SW_TRANSPOSE		= 3	; 0=down, 1=up (default)
	.equ SW_DCA_MODE		= 4	; 0=gate, 1=env (default)
	.equ SW_MODWHEEL_ENABLE		= 5	; 0=diable, 1=enable (default)
	.equ SW_LFO_KBD_SYNC		= 6	; 0=off, 1=on (default)
	.equ SW_DCF_KBD_TRACK		= 7	; 0=off, 1=on (default)

PATCH_SWITCH4:		.BYTE 1
	.equ SW_OSCA_WAVE_EXT		= 0	; 0=normal squ or SAW, 1=extended pwm or tri
	.equ SW_USER_4_1		= 1	;
	.equ SW_OSCB_WAVE_EXT		= 2	; 0=normal SQU or SAW, 1=extended pwm or tri
	.equ SW_USER_4_3		= 3	;
	.equ SW_USER_4_4		= 4	;
	.equ SW_USER_4_5		= 5	;
	.equ SW_LFO2_WAVE		= 6	; 0=tri, 1=squ
	.equ SW_LFO2_RANDOM		= 7	; 0=norm, 1=rand
.endif	; USE_MORE_SWITCHES

SWITCH3:		.BYTE 1		; b0: MIDI SWITCH 1
					; b1: MIDI SWITCH 2
					; b2: MIDI SWITCH 3
					; b3: MIDI SWITCH 4

SETMIDICHANNEL:		.BYTE 1		; selected MIDI channel: 0 for OMNI or 1..15
DETUNEB_FRAC:		.BYTE 1		;\
DETUNEB_INTG:		.BYTE 1		;/ -128,000..+127,996

.if !USE_EXTENDED_MIDI_CONTROL	; full env control
; DCA enevelop
ATTACKTIME:		.BYTE 1		; 0..255
DECAYTIME:		.BYTE 1		; 0..255
SUSTAINLEVEL:		.BYTE 1		; 0/255
RELEASETIME:		.BYTE 1		; 0..255

; DCF enevelop
ATTACKTIME2:		.BYTE 1		; 0..255
DECAYTIME2:		.BYTE 1		; 0..255
SUSTAINLEVEL2:		.BYTE 1		; 0/255
RELEASETIME2:		.BYTE 1		; 0..255
.endif	; !USE_EXTENDED_MIDI_CONTROL

NOTE_L:			.BYTE 1
NOTE_H:			.BYTE 1
NOTE_INTG:		.BYTE 1
PORTACNT:		.BYTE 1		; 2 / 1 / 0
LPF_I:			.BYTE 1
HPF_I:			.BYTE 1
LEVEL:			.BYTE 1		; 0..255
PITCH:			.BYTE 1		; 0..96
ADC_CHAN:		.BYTE 1		; 0..7
PREV_ADC_CHAN:		.BYTE 1		; 0..7
ADC_0:			.BYTE 1		; Panel knob values.
ADC_1:			.BYTE 1
ADC_2:			.BYTE 1
ADC_3:			.BYTE 1
ADC_4:			.BYTE 1
ADC_5:			.BYTE 1
ADC_6:			.BYTE 1
ADC_7:			.BYTE 1
OLD_ADC_0:		.BYTE 1		; Previous panel knob value
OLD_ADC_1:		.BYTE 1
OLD_ADC_2:		.BYTE 1
OLD_ADC_3:		.BYTE 1
OLD_ADC_4:		.BYTE 1
OLD_ADC_5:		.BYTE 1
OLD_ADC_6:		.BYTE 1
OLD_ADC_7:		.BYTE 1
GATE:			.BYTE 1		; 0 / 1
GATEEDGE:		.BYTE 1		; 0 / 1
TPREV_KBD_L:		.BYTE 1
TPREV_KBD_H:		.BYTE 1
TPREV_L:		.BYTE 1
TPREV_H:		.BYTE 1
DELTAT_L:		.BYTE 1		;\ Time from former course
DELTAT_H:		.BYTE 1		;/ of the main loop (1 bit = 32 µs)

ENVPHASE:		.BYTE 1		; 0=stop 1=attack 2=decay 3=sustain 4=release
ENV_FRAC_L:		.BYTE 1
ENV_FRAC_H:		.BYTE 1
ENV_INTEGR:		.BYTE 1

ENVPHASE2:		.BYTE 1		; 0=stop 1=attack 2=decay 3=sustain 4=release
ENV_FRAC_L2:		.BYTE 1
ENV_FRAC_H2:		.BYTE 1
ENV_INTEGR2:		.BYTE 1
VELOCITY_ENVMOD:	.BYTE 1

LFOPHASE:		.BYTE 1		; 0=up 1=down
LFO_FRAC_L:		.BYTE 1		;\
LFO_FRAC_H:		.BYTE 1		; > -128,000..+127,999
LFO_INTEGR:		.BYTE 1		;/
LFOVALUE:		.BYTE 1		; -128..+127

LFO2PHASE:		.BYTE 1		; 0=up 1=down
LFO2_FRAC_L:		.BYTE 1		;\
LFO2_FRAC_H:		.BYTE 1		; > -128,000..+127,999
LFO2_INTEGR:		.BYTE 1		;/
LFO2VALUE:		.BYTE 1		; -128..+127 (currently not used)

OLDWAVEA:		.BYTE 1
OLDWAVEB:		.BYTE 1
SHIFTREG_0:		.BYTE 1		;\
SHIFTREG_1:		.BYTE 1		; > shift register for
SHIFTREG_2:		.BYTE 1		;/  pseudo-random generator
LFOBOTTOM_0:		.BYTE 1		;\
LFOBOTTOM_1:		.BYTE 1		; > bottom level of LFO
LFOBOTTOM_2:		.BYTE 1		;/
LFOTOP_0:		.BYTE 1		;\
LFOTOP_1:		.BYTE 1		; > top level of LFO
LFOTOP_2:		.BYTE 1		;/
LFO2BOTTOM_0:		.BYTE 1		;\
LFO2BOTTOM_1:		.BYTE 1		; > bottom level of LFO2
LFO2BOTTOM_2:		.BYTE 1		;/
LFO2TOP_0:		.BYTE 1		;\
LFO2TOP_1:		.BYTE 1		; > top level of LFO2
LFO2TOP_2:		.BYTE 1		;/

DCOA_LEVEL:		.BYTE 1
DCOB_LEVEL:		.BYTE 1

KNOB_DEADZONE:		.BYTE 1		; not used

; increase phase for DCO A
DELTAA_0:		.byte 1
DELTAA_1:		.byte 1
DELTAA_2:		.byte 1

; increase phase for DCO B
DELTAB_0:		.byte 1
DELTAB_1:		.byte 1
DELTAB_2:		.byte 1

; Wavetable select
WAVETABLE_A:		.byte 1		; Bandlimited wavetable 0..11
WAVETABLE_B:		.byte 1		; Bandlimited wavetable 0..11

; oscillator pulse width
PULSE_WIDTH:		.byte 1
PULSE_KNOB_LIMITED:	.byte 1

.if USE_ORIGINAL_DCO_FM
; fm
WAVEB:			.byte 1
.if !USE_MIDI_FMDEPTH
FMDEPTH:		.byte 1
.endif	; !USE_MIDI_FMDEPTH
.endif	; USE_ORIGINAL_DCO_FM

; eeprom
WRITE_MODE:		.byte 1
WRITE_OFFSET:		.byte 1		; byte 0..15/63 of the patch
.if !USE_EXTENDED_PATCH
WRITE_PATCH_OFFSET:	.byte 1		; start of patch in eeprom
.else	; !USE_EXTENDED_PATCH
WRITE_PATCH_OFFSET_L:	.byte 1		; start of patch in eeprom
WRITE_PATCH_OFFSET_H:	.byte 1		; start of patch in eeprom
.endif	; !USE_EXTENDED_PATCH

; filter
SCALED_RESONANCE:	.byte 1		; not used
b_L:			.byte 1
b_H:			.byte 1
VCF_STATUS: 		.byte 1		; 0 indicates VCF off, 1 = on

.if !USE_ORIGINAL_NOISEGEN
LFSR_0:			.byte 1		;\
LFSR_1:			.byte 1		; > linear feedback shift register for
LFSR_2:			.byte 1		;/  noise  generator
.endif	; !USE_ORIGINAL_NOISEGEN

.if !USE_EXTENDED_MIDI_CONTROL & USE_MIDI_PWMDEPTH
PWMDEPTH: 		.byte	1
.endif	; USE_EXTENDED_MIDI_CONTROL & USE_MIDI_PWMDEPTH

;-----------------------------------------------------------------------------
; MIDI Control Change parameter table
;-----------------------------------------------------------------------------
;
; MIDI CC parameters with an offset from MIDICC. They are automatically
; stored for use, just use the variable name to access their value.


MIDICC:	.byte $80
	.equ MIDIMODWHEEL	= MIDICC + $01

.if USE_EXTENDED_MIDI_CONTROL
	.equ PWMDEPTH		= MIDICC + $0E	; Knob 1	14 ; (=LFO2LEVEL)
	.equ MIDI_KNOB_2	= MIDICC + $0F	; Knob 2	15
	.equ MIDI_KNOB_3	= MIDICC + $10	; Knob 3	16
	.equ MIDI_KNOB_4	= MIDICC + $11	; Knob 4	17
	.equ LFO2FREQ		= MIDICC + $12	; Knob 5	18 ; currently not used
	.equ FMDEPTH		= MIDICC + $13	; Knob 6	19 ; currently not used
	.equ MIXER_BALANCE	= MIDICC + $14	; Knob 7	20
	.equ MASTER_VOLUME	= MIDICC + $15	; Knob 8	21

	.equ ATTACKTIME		= MIDICC + $16	; Slider 1	22 ; DCA envelope
	.equ DECAYTIME		= MIDICC + $17	; Slider 2	23
	.equ SUSTAINLEVEL	= MIDICC + $18	; Slider 3	24
	.equ RELEASETIME	= MIDICC + $19	; Slider 4	25
	.equ ATTACKTIME2	= MIDICC + $1A	; Slider 5	26 ; DCF envelope
	.equ DECAYTIME2		= MIDICC + $1B	; Slider 6	27
	.equ SUSTAINLEVEL2	= MIDICC + $1C	; Slider 7	28
	.equ RELEASETIME2	= MIDICC + $1D	; Slider 8	29
.endif	; USE_EXTENDED_MIDI_CONTROL

.if USE_MORE_SWITCHES
.if USE_EXTENDED_PATCH
	.equ SW1		= MIDICC + $2C ; Reserved
	.equ SW2		= MIDICC + $2D ; Reserved
.endif	; USE_EXTENDED_PATCH
	.equ SW3		= MIDICC + $2E ; Reserved
	.equ SW4		= MIDICC + $2F ; Reserved
.endif	; USE_MORE_SWITCHES

	; Unshifted knobs (potentiometer 0 through 7)
	.equ RESONANCE		= MIDICC + $30
	.equ CUTOFF		= MIDICC + $31
	.equ LFOFREQ		= MIDICC + $32
	.equ PANEL_LFOLEVEL	= MIDICC + $33
	.equ VCFENVMOD		= MIDICC + $34
	.equ PORTAMENTO		= MIDICC + $35
	.equ PULSE_KNOB		= MIDICC + $36	; (=LFO2FREQ)
	.equ OSC_DETUNE		= MIDICC + $37

  ; Shifted knobs (potentiometer 0 through 7)
;	.equ X			= MIDICC + $38 ; Undefined
;	.equ X			= MIDICC + $39 ; Undefined
	.equ KNOB_DCF_DECAY	= MIDICC + $3A
	.equ KNOB_DCF_ATTACK	= MIDICC + $3B
	.equ KNOB_AMP_DECAY	= MIDICC + $3C
	.equ KNOB_AMP_ATTACK	= MIDICC + $3D
.if !USE_EXTENDED_PATCH
	.equ SW1		= MIDICC + $3E ; Reserved
	.equ SW2		= MIDICC + $3F ; Reserved
.else	;  !USE_EXTENDED_PATCH
;	.equ X			= MIDICC + $3E ; Undefined
;	.equ X			= MIDICC + $3F ; Undefined
.endif	;  !USE_EXTENDED_PATCH

; Panel switches 0..15
; midi cc switch messages are not stored here
; Switches 2
	.equ S_KNOB_SHIFT	= MIDICC + $40
	.equ S_OSC_FM		= MIDICC + $41
	.equ S_LFO_RANDOM	= MIDICC + $42
	.equ S_LFO_WAVE		= MIDICC + $43
	.equ S_FILTER_MODE	= MIDICC + $44
	.equ S_DISTORTION	= MIDICC + $45
	.equ S_LFO_ENABLE	= MIDICC + $46
	.equ S_LFO_DEST		= MIDICC + $47
; Switches 1
	.equ S_ANTI_ALIAS	= MIDICC + $48
	.equ S_OSCB_OCT		= MIDICC + $49
	.equ S_OSCB_ENABLE	= MIDICC + $4A
	.equ S_OSCB_WAVE	= MIDICC + $4B
	.equ S_SUSTAIN		= MIDICC + $4C
	.equ S_OSCA_NOISE	= MIDICC + $4D
	.equ S_PWM_SWEEP	= MIDICC + $4E
	.equ S_OSCA_WAVE	= MIDICC + $4F

.if USE_MORE_SWITCHES
; Switches 3
	.equ S_MIX_RING		= MIDICC + $50
	.equ S_USER_3_1		= MIDICC + $51
	.equ S_USER_3_2		= MIDICC + $52
	.equ S_TRANSPOSE	= MIDICC + $53
	.equ S_DCA_MODE		= MIDICC + $54
	.equ S_MODWHEEL_ENABLE	= MIDICC + $55
	.equ S_LFO_KBD_SYNC	= MIDICC + $56
	.equ S_DCF_KBD_TRACK	= MIDICC + $57
; Switches 4
	.equ S_OSCA_WAVE_EXT	= MIDICC + $58
	.equ S_USER_4_1		= MIDICC + $59
	.equ S_OSCB_WAVE_EXT	= MIDICC + $5A
	.equ S_USER_4_3		= MIDICC + $5B
	.equ S_USER_4_4		= MIDICC + $5C
	.equ S_USER_4_5		= MIDICC + $5D
	.equ S_LFO2_WAVE	= MIDICC + $5E
	.equ S_LFO2_RANDOM	= MIDICC + $5F
.endif	; USE_EXTENDED_BUTTON_FUNCTIONS


; Patch save/load and MIDI channel set
LED_STATUS:		.byte 1		; off/on status of front panel LED
LED_TIMER:		.byte 1		; number of blinks before reset
BUTTON_STATUS:		.byte 1		; MIDI=1, SAVE=3, LOAD=7, CLEAR=0
CONTROL_SWITCH:		.byte 1		; Last panel switch moved: 1..16, where zero indicates no movement.

.if !USE_ORIGINAL_ENVELOPE_CONTROL
MIDI_TASK_UPDATE:	.byte 1

	.equ	UPDATE_ENV_DCF	= 0
	.equ	UPDATE_ENV_DCA	= 1
	.equ	UPDATE_VOLUME	= 2
.endif	; !USE_ORIGINAL_ENVELOPE_CONTROL

.if USE_MASTER_VOLUME
VOLUME:			.byte 1
.if !USE_EXTENDED_MIDI_CONTROL
MASTER_VOLUME:			.byte 1
.endif	; !USE_EXTENDED_MIDI_CONTROL
.endif	; USE_MASTER_VOLUME

;-----------------------------------------------------------------------------

;stack: 0x0A3..0x25F

	.ESEG

.if !USE_EXTENDED_PATCH

	.ORG	0	; default patch

	.db	0x00	; RESONANCE
	.db	0xFE	; CUTOFF
	.db	0xC0	; LFOFREQ
	.db	0x00	; PANEL_LFOLEVEL
	.db	0x80	; VCFENVMOD
	.db	0x00	; PORTAMENTO
	.db	0xC0	; PULSE_KNOB
	.db	0x80	; OSC_DETUNE

	.db	-1	; X
	.db	-1	; X
	.db	0xA0	; KNOB_DCF_DECAY
	.db	0x00	; KNOB_DCF_ATTACK
	.db	0xA0	; KNOB_AMP_DECAY
	.db	0x00	; KNOB_AMP_ATTACK
	.db	0x00	; SW1 -> PATCH_SWITCH1
			;	SW_KNOB_SHIFT	= 0	; 0=lower, 1=upper
			;	SW_OSC_FM	= 1	;
			;	SW_LFO_RANDOM	= 2	; 0=norm, 1=rand
			;	SW_LFO_WAVE	= 3	; 0=tri, 1=squ
			;	SW_FILTER_MODE	= 4	; 0=lp, 1=hp
			;	SW_DISTORTION	= 5	; 0=off, 1=on
			;	SW_LFO_ENABLE	= 6	;
			;	SW_LFO_DEST	= 7	; 0=DCF, 1=DCO
	.db	0x99 	; SW2 ->PATCH_SWITCH2
			;	SW_ANTI_ALIAS	= 0 x	;
			;	SW_OSCB_OCT	= 1	; 0=down, 1=up
			;	SW_OSCB_ENABLE	= 2 	; 0=off, 1=on
			;	SW_OSCB_WAVE	= 3 x	; 0=saw, 1=squ
			;	SW_SUSTAIN	= 4 x	;
			;	SW_OSCA_NOISE	= 5	; 0=off, 1=on
			;	SW_PWM_SWEEP	= 6	; 0=off, 1=LFO2
			;	SW_OSCA_WAVE	= 7 x	; 0=saw, 1=squ

;-------------------------
.else	; !USE_EXTENDED_PATCH
;					  eeprom addr -> midi cc addr
	.ORG	0	; default patch

	; Unshifted knobs (potentiometer 0 through 7)
	.db	0x00	; RESONANCE		; $00 -> $30
	.db	0xFE	; CUTOFF		; $01 -> $31
	.db	0xC0	; LFOFREQ		; $02 -> $32
	.db	0x00	; PANEL_LFOLEVEL	; $03 -> $33
	.db	0x80	; VCFENVMOD		; $04 -> $34
	.db	0x00	; PORTAMENTO		; $05 -> $35
	.db	0xC0	; PULSE_KNOB		; $06 -> $36	; (=LFO2FREQ)
	.db	0x80	; OSC_DETUNE		; $07 -> $37

	; Shifted knobs (potentiometer 0 through 7)
	.db	-1	; X			; $08 -> $38 ; Undefined
	.db	-1	; X			; $09 -> $39 ; Undefined
	.db	0xA0	; KNOB_DCF_DECAY	; $0A -> $3A
	.db	0x00	; KNOB_DCF_ATTACK	; $0B -> $3B
	.db	0xA0	; KNOB_AMP_DECAY	; $0C -> $3C
	.db	0x00	; KNOB_AMP_ATTACK	; $0D -> $3D
	.db	-1	; X			; $0E -> $3E ; Undefined
	.db	-1	; X			; $0F -> $3F ; Undefined

	.db	0x80	; PWMDEPTH		; $10 -> $0E	; Knob 1	14 ; (=LFO2LEVEL)
	.db	-1	; MIDI_KNOB_2		; $11 -> $0F	; Knob 2	15 ; currently not used
	.db	-1	; MIDI_KNOB_3		; $12 -> $10	; Knob 3	16 ; currently not used
	.db	-1	; MIDI_KNOB_4		; $13 -> $11	; Knob 4	17
	.db	0x80	; LFO2FREQ		; $14 -> $12	; Knob 5	18
	.db	0x00	; FMDEPTH		; $15 -> $13	; Knob 6	19
	.db	0x80	; MIXER_BALANCE		; $16 -> $14	; Knob 7	20
	.db	0x80	; MASTER_VOLUME		; $17 -> $15	; Knob 8	21

	.db	0x00	; ATTACKTIME		; $18 -> $16	; Slider 1	22 ; DCA envelope
	.db	0x00	; DECAYTIME		; $19 -> $17	; Slider 2	23
	.db	0xFE	; SUSTAINLEVEL		; $1A -> $18	; Slider 3	24
	.db	0x00	; RELEASETIME		; $1B -> $19	; Slider 4	25
	.db	0x00	; ATTACKTIME2		; $1C -> $1A	; Slider 5	26 ; DCF envelope
	.db	0x00	; DECAYTIME2		; $1D -> $1B	; Slider 6	27
	.db	0xFE	; SUSTAINLEVEL2		; $1E -> $1C	; Slider 7	28
	.db	0x00	; RELEASETIME2		; $1F -> $1D	; Slider 8	29

	.db	0x00	; SW1 -> PATCH_SWITCH1	; $20 -> $2C
			;	SW_KNOB_SHIFT	= 0	; 0=lower, 1=upper
			;	SW_OSC_FM	= 1	;
			;	SW_LFO_RANDOM	= 2	; 0=norm, 1=rand
			;	SW_LFO_WAVE	= 3	; 0=tri, 1=squ
			;	SW_FILTER_MODE	= 4	; 0=lp, 1=hp
			;	SW_DISTORTION	= 5	; 0=off, 1=on
			;	SW_LFO_ENABLE	= 6	;
			;	SW_LFO_DEST	= 7	; 0=DCF, 1=DCO
	.db	0x99	; SW2 -> PATCH_SWITCH2	; $21 -> $2D
			;	SW_ANTI_ALIAS	= 0 x	;
			;	SW_OSCB_OCT	= 1	; 0=down, 1=up
			;	SW_OSCB_ENABLE	= 2 	; 0=off, 1=on
			;	SW_OSCB_WAVE	= 3 x	; 0=saw, 1=squ
			;	SW_SUSTAIN	= 4 x	;
			;	SW_OSCA_NOISE	= 5	; 0=off, 1=on
			;	SW_PWM_SWEEP	= 6	; 0=off, 1=LFO2
			;	SW_OSCA_WAVE	= 7 x	; 0=saw, 1=squ
	.db	0xF8	; SW3 -> PATCH_SWITCH3	; $22 -> $2E
			;	SW_USER_3_0	= 0	;
			;	SW_USER_3_1	= 1	;
			;	SW_MIX_RING	= 2	; 0=Mixer(default), 1=Ringmodulator
			;	SW_TRANSPOSE	= 3 x	; 0=down, 1=up (default)
			;	SW_DCA_MODE	= 4 x	; 0=gate, 1=env (default)
			;	SW_MODWHEEL_ENA	= 5 x	; 0=diable, 1=enable (default)
			;	SW_LFO_KBD_SYNC	= 6 x	; 0=off, 1=on (default)
			;	SW_DCF_KBD_TRACK= 7 x	; 0=off, 1=on (default)
	.db	0x00	; SW4 -> PATCH_SWITCH4	; $23 -> $2F
			;	SW_OSCA_WAVE_EXT= 0	; 0=normal squ or SAW, 1=extended pwm or tri
			;	SW_USER_4_2	= 1	;
			;	SW_OSCB_WAVE_EXT= 2	; 0=normal SQU or SAW, 1=extended pwm or tri
			;	SW_USER_4_3	= 3	;
			;	SW_USER_4_4	= 4	;
			;	SW_USER_4_5	= 5	;
			;	SW_LFO2_WAVE	= 6	; 0=tri, 1=squ
			;	SW_LFO2_RANDOM	= 7	; 0=norm, 1=rand

;-------------------------
; second patch

	.ORG	0x30	; 2nd patch

	; Unshifted knobs (potentiometer 0 through 7)
	.db	0x60	; RESONANCE		; $00 -> $30
	.db	0x80	; CUTOFF		; $01 -> $31
	.db	0x00	; LFOFREQ		; $02 -> $32
	.db	0x00	; PANEL_LFOLEVEL	; $03 -> $33
	.db	0x80	; VCFENVMOD		; $04 -> $34
	.db	0x00	; PORTAMENTO		; $05 -> $35
	.db	0xFE	; PULSE_KNOB		; $06 -> $36	; (=LFO2FREQ)
	.db	0x80	; OSC_DETUNE		; $07 -> $37

	; Shifted knobs (potentiometer 0 through 7)
	.db	-1	; X			; $08 -> $38 ; Undefined
	.db	-1	; X			; $09 -> $39 ; Undefined
	.db	0x00	; KNOB_DCF_DECAY	; $0A -> $3A
	.db	0x00	; KNOB_DCF_ATTACK	; $0B -> $3B
	.db	0xA0	; KNOB_AMP_DECAY	; $0C -> $3C
	.db	0x00	; KNOB_AMP_ATTACK	; $0D -> $3D
	.db	-1	; X			; $0E -> $3E ; Undefined
	.db	-1	; X			; $0F -> $3F ; Undefined

	.db	0x60	; PWMDEPTH		; $10 -> $0E	; Knob 1	14 ; (=LFO2LEVEL)
	.db	-1	; MIDI_KNOB_2		; $11 -> $0F	; Knob 2	15 ; currently not used
	.db	-1	; MIDI_KNOB_3		; $12 -> $10	; Knob 3	16 ; currently not used
	.db	-1	; MIDI_KNOB_4		; $13 -> $11	; Knob 4	17
	.db	0x80	; LFO2FREQ		; $14 -> $12	; Knob 5	18
	.db	0xFF	; FMDEPTH		; $15 -> $13	; Knob 6	19
	.db	0x80	; MIXER_BALANCE		; $16 -> $14	; Knob 7	20
	.db	0x80	; MASTER_VOLUME		; $17 -> $15	; Knob 8	21

	.db	0x00	; ATTACKTIME		; $18 -> $16	; Slider 1	22 ; DCA envelope
	.db	0xA0	; DECAYTIME		; $19 -> $17	; Slider 2	23
	.db	0x00	; SUSTAINLEVEL		; $1A -> $18	; Slider 3	24
	.db	0xA0	; RELEASETIME		; $1B -> $19	; Slider 4	25
	.db	0x00	; ATTACKTIME2		; $1C -> $1A	; Slider 5	26 ; DCF envelope
	.db	0x00	; DECAYTIME2		; $1D -> $1B	; Slider 6	27
	.db	0x00	; SUSTAINLEVEL2		; $1E -> $1C	; Slider 7	28
	.db	0x00	; RELEASETIME2		; $1F -> $1D	; Slider 8	29

	.db	0x00	; SW1 -> PATCH_SWITCH1	; $20 -> $2C
			;	SW_KNOB_SHIFT	= 0	; 0=lower, 1=upper
			;	SW_OSC_FM	= 1 	;
			;	SW_LFO_RANDOM	= 2 	; 0=norm, 1=rand
			;	SW_LFO_WAVE	= 3 	; 0=tri, 1=squ
			;	SW_FILTER_MODE	= 4	; 0=lp, 1=hp
			;	SW_DISTORTION	= 5	; 0=off, 1=on
			;	SW_LFO_ENABLE	= 6	;
			;	SW_LFO_DEST	= 7	; 0=DCF, 1=DCO
	.db	0x99	; SW2 -> PATCH_SWITCH2	; $21 -> $2D
			;	SW_ANTI_ALIAS	= 0 x	;
			;	SW_OSCB_OCT	= 1	; 0=down, 1=up
			;	SW_OSCB_ENABLE	= 2 	; 0=off, 1=on
			;	SW_OSCB_WAVE	= 3 x	; 0=saw, 1=squ
			;	SW_SUSTAIN	= 4 x	;
			;	SW_OSCA_NOISE	= 5	; 0=off, 1=on
			;	SW_PWM_SWEEP	= 6	; 0=off, 1=LFO2
			;	SW_OSCA_WAVE	= 7 x	; 0=saw, 1=squ
	.db	0xF8	; SW3 -> PATCH_SWITCH3	; $22 -> $2E
			;	SW_USER_3_0	= 0 	;
			;	SW_USER_3_1	= 1 	;
			;	SW_MIX_RING	= 2	; 0=Mixer(default), 1=Ringmodulator
			;	SW_TRANSPOSE	= 3 x	; 0=down, 1=up (default)
			;	SW_DCA_MODE	= 4 x	; 0=gate, 1=env (default)
			;	SW_MODWHEEL_ENA	= 5 x	; 0=diable, 1=enable (default)
			;	SW_LFO_KBD_SYNC	= 6 x	; 0=off, 1=on (default)
			;	SW_DCF_KBD_TRACK= 7 x	; 0=off, 1=on (default)
	.db	0x00	; SW4 -> PATCH_SWITCH4	; $23 -> $2F
			;	SW_OSCA_WAVE_EXT= 0 	; 0=normal squ or SAW, 1=extended pwm or tri
			;	SW_USER_4_1	= 1 	;
			;	SW_OSCB_WAVE_EXT= 2 	; 0=normal SQU or SAW, 1=extended pwm or tri
			;	SW_USER_4_3	= 3 	;
			;	SW_USER_4_4	= 4 	;
			;	SW_USER_4_5	= 5 	;
			;	SW_LFO2_WAVE	= 6 	; 0=tri, 1=squ
			;	SW_LFO2_RANDOM	= 7 	; 0=norm, 1=rand

;-------------------------
; 3rd patch
	.org	0x60	; 3rd patch

	; Unshifted knobs (potentiometer 0 through 7)
	.db	0xFE	; RESONANCE		; $00 -> $30
	.db	0xFE	; CUTOFF		; $01 -> $31
	.db	0xE0	; LFOFREQ		; $02 -> $32
	.db	0x30	; PANEL_LFOLEVEL	; $03 -> $33
	.db	0x00	; VCFENVMOD		; $04 -> $34
	.db	0x00	; PORTAMENTO		; $05 -> $35
	.db	0x28	; PULSE_KNOB		; $06 -> $36	; (=LFO2FREQ)
	.db	0x8F	; OSC_DETUNE		; $07 -> $37

	; Shifted knobs (potentiometer 0 through 7)
	.db	-1	; X			; $08 -> $38 ; Undefined
	.db	-1	; X			; $09 -> $39 ; Undefined
	.db	0xE8	; KNOB_DCF_DECAY	; $0A -> $3A
	.db	0x00	; KNOB_DCF_ATTACK	; $0B -> $3B
	.db	0x80	; KNOB_AMP_DECAY	; $0C -> $3C
	.db	0x00	; KNOB_AMP_ATTACK	; $0D -> $3D
	.db	-1	; X			; $0E -> $3E ; Undefined
	.db	-1	; X			; $0F -> $3F ; Undefined

	.db	0x60	; PWMDEPTH		; $10 -> $0E	; Knob 1	14 ; (=LFO2LEVEL)
	.db	-1	; MIDI_KNOB_2		; $11 -> $0F	; Knob 2	15 ; currently not used
	.db	-1	; MIDI_KNOB_3		; $12 -> $10	; Knob 3	16 ; currently not used
	.db	-1	; MIDI_KNOB_4		; $13 -> $11	; Knob 4	17
	.db	0x80	; LFO2FREQ		; $14 -> $12	; Knob 5	18
	.db	0xFF	; FMDEPTH		; $15 -> $13	; Knob 6	19
	.db	0x80	; MIXER_BALANCE		; $16 -> $14	; Knob 7	20
	.db	0x80	; MASTER_VOLUME		; $17 -> $15	; Knob 8	21

	.db	0x00	; ATTACKTIME		; $18 -> $16	; Slider 1	22 ; DCA envelope
	.db	0x80	; DECAYTIME		; $19 -> $17	; Slider 2	23
	.db	0x00	; SUSTAINLEVEL		; $1A -> $18	; Slider 3	24
	.db	0x80	; RELEASETIME		; $1B -> $19	; Slider 4	25
	.db	0x00	; ATTACKTIME2		; $1C -> $1A	; Slider 5	26 ; DCF envelope
	.db	0xE8	; DECAYTIME2		; $1D -> $1B	; Slider 6	27
	.db	0x00	; SUSTAINLEVEL2		; $1E -> $1C	; Slider 7	28
	.db	0xE8	; RELEASETIME2		; $1F -> $1D	; Slider 8	29

	.db	0x00	; SW1 -> PATCH_SWITCH1	; $20 -> $2C
			;	SW_KNOB_SHIFT	= 0	; 0=lower, 1=upper
			;	SW_OSC_FM	= 1 	;
			;	SW_LFO_RANDOM	= 2 	; 0=norm, 1=rand
			;	SW_LFO_WAVE	= 3 	; 0=tri, 1=squ
			;	SW_FILTER_MODE	= 4	; 0=lp, 1=hp
			;	SW_DISTORTION	= 5	; 0=off, 1=on
			;	SW_LFO_ENABLE	= 6	;
			;	SW_LFO_DEST	= 7	; 0=DCF, 1=DCO
	.db	0x8F	; SW2 -> PATCH_SWITCH2	; $21 -> $2D
			;	SW_ANTI_ALIAS	= 0 x	;
			;	SW_OSCB_OCT	= 1 x	; 0=down, 1=up
			;	SW_OSCB_ENABLE	= 2 x	; 0=off, 1=on
			;	SW_OSCB_WAVE	= 3 x	; 0=saw, 1=squ
			;	SW_SUSTAIN	= 4	;
			;	SW_OSCA_NOISE	= 5	; 0=off, 1=on
			;	SW_PWM_SWEEP	= 6	; 0=off, 1=LFO2
			;	SW_OSCA_WAVE	= 7 x	; 0=saw, 1=squ
	.db	0xF8	; SW3 -> PATCH_SWITCH3	; $22 -> $2E
			;	SW_USER_3_0	= 0 	;
			;	SW_USER_3_1	= 1 	;
			;	SW_MIX_RING	= 2	; 0=Mixer(default), 1=Ringmodulator
			;	SW_TRANSPOSE	= 3 x	; 0=down, 1=up (default)
			;	SW_DCA_MODE	= 4 x	; 0=gate, 1=env (default)
			;	SW_MODWHEEL_ENA	= 5 x	; 0=diable, 1=enable (default)
			;	SW_LFO_KBD_SYNC	= 6 x	; 0=off, 1=on (default)
			;	SW_DCF_KBD_TRACK= 7 x	; 0=off, 1=on (default)
	.db	0x00	; SW4 -> PATCH_SWITCH4	; $23 -> $2F
			;	SW_OSCA_WAVE_EXT= 0 	; 0=normal squ or SAW, 1=extended pwm or tri
			;	SW_USER_4_1	= 1 	;
			;	SW_OSCB_WAVE_EXT= 2 	; 0=normal SQU or SAW, 1=extended pwm or tri
			;	SW_USER_4_3	= 3 	;
			;	SW_USER_4_4	= 4 	;
			;	SW_USER_4_5	= 5 	;
			;	SW_LFO2_WAVE	= 6 	; 0=tri, 1=squ
			;	SW_LFO2_RANDOM	= 7 	; 0=norm, 1=rand

;-------------------------
; 4th patch
	.org	0x90	; 4th patch

;-------------------------
; 5th patch
	.org	0xC0	; 5th patch

;-------------------------
; 6th patch
	.org	0xF0	; 6th patch

;-------------------------
; 7th patch
	.org	0x120	; 7th patch

;-------------------------
; 8th patch
	.org	0x150	; 8th patch

;-------------------------
; 9th patch
	.org	0x180	; 9th patch

;-------------------------
; 10th patch
	.org	0x1B0	; 10th patch

;-------------------------
; 11th patch
	.org	0x1E0	; 11th patch

;-------------------------
; 12th patch
	.org	0x210	; 12th patch

;-------------------------
; 13th patch
	.org	0x240	; 13th patch

;-------------------------
; 14th patch
	.org	0x270	; 14th patch

;-------------------------
; 15th patch
	.org	0x2A0	; 15th patch

;-------------------------
; 16th patch
	.org	0x2D0	; 16th patch


.endif	; !USE_EXTENDED_PATCH

	.ORG 	0x3fe
	.db	0	; current patch
	.db	0	; midi address

;-----------------------------------------------------------------------------
;		V E C T O R   T A B L E
;-----------------------------------------------------------------------------
	.CSEG

	jmp	RESET		; RESET

	jmp	IRQ_NONE	; INT0
	jmp	IRQ_NONE	; INT1
	jmp	IRQ_NONE	; INT2

	jmp	TIM2_CMP	; TIMER2 COMP
	jmp	IRQ_NONE	; TIMER2 OVF

	jmp	IRQ_NONE	; TIMER1 CAPT
	jmp	IRQ_NONE	; TIMER1 COMPA
	jmp	IRQ_NONE	; TIMER1 COMPB
	jmp	IRQ_NONE	; TIMER1 OVF

	jmp	IRQ_NONE	; TIMER0 COMPA
	jmp	IRQ_NONE	; TIMER0 OVF

	jmp	IRQ_NONE	; SPI,STC

	jmp	UART_RXC	; UART, RX COMPLETE
	jmp	IRQ_NONE	; UART,UDRE
	jmp	IRQ_NONE	; UART, TX COMPLETE

	jmp	IRQ_NONE	; ADC CONVERSION COMPLETE

	jmp	IRQ_NONE	; EEPROM READY

	jmp	IRQ_NONE	; ANALOG COMPARATOR

	jmp	IRQ_NONE	; 2-Wire Serial Interface

	jmp	IRQ_NONE	; STORE PROGRAM MEMORY READY

IRQ_NONE:
	reti

;-----------------------------------------------------------------------------
;		R O M   T A B L E S
;-----------------------------------------------------------------------------
;
; Phase Deltas at 36363.63636 Hz sample rate
;
;		NOTE PHASE DELTA = 2 ^ 24 * Freq / SamplingFreq
;	So...	Note zero calc: 2 ^ 24 * 8.175799 / 36363.63636 = 3772.09651 (stored as 00 0E BC.19)
;
;-----------------------------------------------------------------------------

DELTA_C:
	.DW	0xBC19		;\
	.DW	0x000E		;/ note  0 ( 8.175799 Hz)
DELTA_CIS:
	.DW	0x9C66		;\
	.DW	0x000F		;/ note  1 ( 8.661957 Hz)
DELTA_D:
	.DW	0x8A09		;\
	.DW	0x0010		;/ note  2 ( 9.177024 Hz)
DELTA_DIS:
	.DW	0x85CE		;\
	.DW	0x0011		;/ note  3 ( 9.722718 Hz)
DELTA_E:
	.DW	0x908B		;\
	.DW	0x0012		;/ note  4 (10.300861 Hz)
DELTA_F:
	.DW	0xAB25		;\
	.DW	0x0013		;/ note  5 (10.913382 Hz)
DELTA_FIS:
	.DW	0xD68D		;\
	.DW	0x0014		;/ note  6 (11.562326 Hz)
DELTA_G:
	.DW	0x13C2		;\
	.DW	0x0016		;/ note  7 (12.249857 Hz)
DELTA_GIS:
	.DW	0x63D4		;\
	.DW	0x0017		;/ note  8 (12.978272 Hz)
DELTA_A:
	.DW	0xC7E3		;\
	.DW	0x0018		;/ note  9 (13.750000 Hz)
DELTA_AIS:
	.DW	0x411D		;\
	.DW	0x001A		;/ note 10 (14.567618 Hz)
DELTA_H:
	.DW	0xD0C5		;\
	.DW	0x001B		;/ note 11 (15.433853 Hz)
DELTA_C1:
	.DW	0x7831		;\
	.DW	0x001D		;/ note 12 (16.351598 Hz)

;-----------------------------------------------------------------------------
;
; Lookup Tables
;
; VCF filter cutoff - 128 bytes
; Time to Rate table for calculating amplitude envelopes - 64 bytes
; VCA non-linear level conversion - 256 bytes
;
;-----------------------------------------------------------------------------
; VCF Filter Cutoff
;
; value = (16th root of 2)**(index+1)
;
.if USE_ORIGINAL_TAB_VCF
TAB_VCF:
	.db	  1,   1,   1,   1,   1,   1,   1,   1		;   0
	.db	  1,   1,   1,   1,   1,   1,   1,   2		;   8
	.db	  2,   2,   2,   2,   2,   2,   2,   2		;  16
	.db	  2,   3,   3,   3,   3,   3,   3,   3		;  24
	.db	  4,   4,   4,   4,   4,   5,   5,   5		;  32
	.db	  5,   6,   6,   6,   7,   7,   7,   7		;  40
	.db	  8,   8,   9,   9,   9,  10,  10,  11		;  48
	.db	 11,  12,  12,  13,  14,  14,  15,  16		;  56
	.db	 16,  17,  18,  19,  19,  20,  21,  22		;  64
	.db	 23,  24,  25,  26,  28,  29,  30,  31		;  72
	.db	 33,  34,  36,  38,  39,  41,  43,  45		;  80
	.db	 47,  49,  51,  53,  56,  58,  61,  63		;  88
	.db	 66,  69,  72,  76,  79,  82,  86,  90		;  96
	.db	 94,  98, 103, 107, 112, 117, 122, 127		; 104
	.db	133, 139, 145, 152, 158, 165, 173, 181		; 112
	.db	189, 197, 206, 215, 224, 234, 245, 255		; 120

.else	; USE_ORIGINAL_TAB_VCF

; caluclated filter coeff, very similar to original values
; value = (16th root of 2)**(index+1)

TAB_VCF:
	.db	  1,   1,   1,   1,   1,   1,   1,   1		;   0
	.db	  1,   2,   2,   2,   2,   2,   2,   2		;   8
	.db	  2,   2,   2,   2,   2,   3,   3,   3		;  16
	.db	  3,   3,   3,   3,   4,   4,   4,   4		;  24
	.db	  4,   4,   5,   5,   5,   5,   5,   6		;  32
	.db	  6,   6,   6,   7,   7,   7,   8,   8		;  40
	.db	  8,   9,   9,  10,  10,  10,  11,  11		;  48
	.db	 12,  12,  13,  13,  14,  15,  15,  16		;  56
	.db	 17,  17,  18,  19,  20,  21,  22,  23		;  64
	.db	 24,  25,  26,  27,  28,  29,  31,  32		;  72
	.db	 33,  35,  36,  38,  40,  41,  43,  45		;  80
	.db	 47,  49,  52,  54,  56,  59,  61,  64		;  88
	.db	 67,  70,  73,  76,  79,  83,  87,  91		;  96
	.db	 95,  99, 103, 108, 112, 117, 123, 128		; 104
	.db	134, 140, 146, 152, 159, 166, 173, 181		; 112
	.db	189, 197, 206, 215, 225, 235, 245, 255		; 120
.endif	; USE_ORIGINAL_TAB_VCF

;-----------------------------------------------------------------------------
;Time to Rate conversion table for envelope timing.
; lfo:
;  update values for 32us update rate
;  LFO_INTEGR overflows all 256*32us = 8.192 ms
;
; formula  Tof = 256*32us*2^16/N
;          LFOfreq = 1/Tof

; Rate value =	Rmin * Q^i with Q = (Rmax/Rmin)^(1/31) = 1,286111766

TIMETORATE:
	.DW	50957		; 10.54 mS	fast lfo, attack/rise time
	.DW	39621		; 13.55 mS
	.DW	30807		; 17.43 mS
	.DW	23953		; 22.41 mS
	.DW	18625		; 28.83 mS
	.DW	14481		; 37.07 mS
	.DW	11260		; 47.68 mS
	.DW	 8755		; 61.32 mS
	.DW	 6807		; 78.87 mS
	.DW	 5293		; 101.4 mS
	.DW	 4115		; 130.5 mS
	.DW	 3200		; 167.8 mS
	.DW	 2488		; 215.8 mS
	.DW	 1935		; 277.5 mS
	.DW	 1504		; 356.9 mS
	.DW	 1170		; 459.0 mS
	.DW	  909		; 590.4 mS
	.DW	  707		; 759.3 mS
	.DW	  550		; 976.5 mS
	.DW	  427		; 1.256 S
	.DW	  332		; 1.615 S
	.DW	  258		; 2.077 S
	.DW	  201		; 2.672 S
	.DW	  156		; 3.436 S
	.DW	  121		; 4.419 S
	.DW	   94		; 5.684 S
	.DW	   73		; 7.310 S
	.DW	   57		; 9.401 S
	.DW	   44		; 12.09 S
	.DW	   35		; 15.55 S
	.DW	   27		; 20.00 S
	.DW	   19		; 28.26 S	slow lfo, attack/rise time

;-----------------------------------------------------------------------------
;
; VCA non-linear level conversion
;
; Amplitude level lookup table. Envelopes levels are calculated as linear
; and then converted to approximate an exponential saturation curve.
;
; polynomial y = a	+ bx + cx2 + dx3
; with coefficients…
;    a  0
;    b  0.210841569
;    c  0.000177823
;    d  1.14E-05
; base on these pairs from the above table
;   x: 0  96  192  255
;   y: 0  32  128  255

TAB_VCA:
	.db	  0,   0,   0,   1,   1,   1,   1,   1		;   0
	.db	  2,   2,   2,   2,   3,   3,   3,   3		;   8
	.db	  3,   4,   4,   4,   4,   5,   5,   5		;  16
	.db	  5,   6,   6,   6,   6,   7,   7,   7		;  24
	.db	  7,   8,   8,   8,   8,   9,   9,   9		;  32
	.db	  9,  10,  10,  10,  11,  11,  11,  11		;  40
	.db	 12,  12,  12,  13,  13,  13,  14,  14		;  48
	.db	 14,  15,  15,  15,  16,  16,  16,  17		;  56
	.db	 17,  18,  18,  18,  19,  19,  20,  20		;  64
	.db	 20,  21,  21,  22,  22,  23,  23,  23		;  72
	.db	 24,  24,  25,  25,  26,  26,  27,  27		;  80
	.db	 28,  28,  29,  29,  30,  30,  31,  31		;  88
	.db	 32,  33,  33,  34,  34,  35,  35,  36		;  96
	.db	 37,  37,  38,  39,  39,  40,  41,  41		; 104
	.db	 42,  43,  43,  44,  45,  45,  46,  47		; 112
	.db	 48,  48,  49,  50,  51,  51,  52,  53		; 120

	.db	 54,  55,  56,  56,  57,  58,  59,  60		; 128
	.db	 61,  62,  63,  63,  64,  65,  66,  67		; 136
	.db	 68,  69,  70,  71,  72,  73,  74,  75		; 144
	.db	 76,  77,  78,  80,  81,  82,  83,  84		; 152
	.db	 85,  86,  87,  89,  90,  91,  92,  93		; 160
	.db	 95,  96,  97,  98, 100, 101, 102, 104		; 168
	.db	105, 106, 108, 109, 110, 112, 113, 115		; 176
	.db	116, 118, 119, 120, 122, 123, 125, 126		; 184
	.db	128, 130, 131, 133, 134, 136, 138, 139		; 192
	.db	141, 142, 144, 146, 148, 149, 151, 153		; 200
	.db	154, 156, 158, 160, 162, 164, 165, 167		; 208
	.db	169, 171, 173, 175, 177, 179, 181, 183		; 216
	.db	185, 187, 189, 191, 193, 195, 197, 199		; 224
	.db	201, 203, 206, 208, 210, 212, 214, 217		; 232
	.db	219, 221, 224, 226, 228, 231, 233, 235		; 240
	.db	238, 240, 243, 245, 247, 250, 252, 255		; 248

;-----------------------------------------------------------------------------
;
; Limit maximum resonance when filter cutoff is extremely low
;

TAB_REZ:
	.db	224, 224, 224, 224, 224, 224, 224, 224		;   0 - Low value of DE
	.db	224, 228, 232, 236, 240, 244, 248, 252		;   8 - High value of FC

;-----------------------------------------------------------------------------
;
;-----------------------------------------------------------------------------

.if USE_MASTER_VOLUME
; polynom y = a	+ bx + cx2 + dx3
; ...with coefficients…
;    a  0
;    b  0,271482112
;    c  0,00085632
;    d  9,01E-06
; base on these pairs from the above table
;   x: 0  48  124  247
;   y: 0  16   64  255
;
; add a 9 values wide death zone in the middle of the table

TAB_VOLUME:
	.db	  0,   0,   1,   1,   1,   1,   2,   2		;   0
	.db	  2,   3,   3,   3,   3,   4,   4,   4		;   8
	.db	  5,   5,   5,   6,   6,   6,   6,   7		;  16
	.db	  7,   7,   8,   8,   8,   9,   9,  10		;  24
	.db	 10,  10,  11,  11,  11,  12,  12,  12		;  32
	.db	 13,  13,  14,  14,  14,  15,  15,  16		;  40
	.db	 16,  16,  17,  17,  18,  18,  19,  19		;  48
	.db	 19,  20,  20,  21,  21,  22,  22,  23		;  56
	.db	 23,  24,  24,  25,  25,  26,  26,  27		;  64
	.db	 27,  28,  28,  29,  30,  30,  31,  31		;  72
	.db	 32,  32,  33,  34,  34,  35,  35,  36		;  80
	.db	 37,  37,  38,  39,  39,  40,  41,  41		;  88
	.db	 42,  43,  43,  44,  45,  45,  46,  47		;  96
	.db	 48,  48,  49,  50,  51,  51,  52,  53		; 104
	.db	 54,  55,  55,  56,  57,  58,  59,  60		; 112
	.db	 60,  61,  62,  63,  64,  64,  64,  64		; 120

	.db	 64,  64,  64,  64,  64,  65,  66,  67		; 128
	.db	 68,  69,  70,  71,  71,  72,  73,  74		; 136
	.db	 75,  76,  77,  78,  80,  81,  82,  83		; 144
	.db	 84,  85,  86,  87,  88,  89,  90,  92		; 152
	.db	 93,  94,  95,  96,  97,  99, 100, 101		; 160
	.db	102, 103, 105, 106, 107, 109, 110, 111		; 168
	.db	112, 114, 115, 116, 118, 119, 121, 122		; 176
	.db	123, 125, 126, 128, 129, 131, 132, 134		; 184
	.db	135, 137, 138, 140, 141, 143, 144, 146		; 192
	.db	147, 149, 151, 152, 154, 156, 157, 159		; 200
	.db	161, 162, 164, 166, 167, 169, 171, 173		; 208
	.db	175, 176, 178, 180, 182, 184, 186, 187		; 216
	.db	189, 191, 193, 195, 197, 199, 201, 203		; 224
	.db	205, 207, 209, 211, 213, 215, 217, 219		; 232
	.db	222, 224, 226, 228, 230, 232, 235, 237		; 240
	.db	239, 241, 243, 246, 248, 250, 253, 255		; 248

.endif	; USE_MASTER_VOLUME

;-----------------------------------------------------------------------------
;
;-----------------------------------------------------------------------------

.if USE_UART
	.include "uart.inc"
.endif	; USE_UART

;-----------------------------------------------------------------------------
;	I N T E R R U P T   S U B R O U T I N E S
;-----------------------------------------------------------------------------
; Timer 2 compare interrupt (sampling)
;
; This is where sound is generated. This interrupt is called 36,363 times per second
; to calculate a single 16-bit value for audio output. There are ~440 instruction cycles
; (16MHZ/36,363) between samples, and these have to be shared between this routine and the
; main program loop that scans controls, receives MIDI commands and calculates envelope,
; LFO, and DCA/DCF levels.
;
; If you use too many clock cycles here there won't be sufficient time left over for
; general housekeeping tasks. The result will be sluggish and lost notes, weird timing and sadness.
;-----------------------------------------------------------------------------

; Push contents of registers onto the stack
;
TIM2_CMP:
	push	r16		; wave B
	in	r16, SREG	;\
	push	r16		;/ push SREG
	push	r17		; wave A
	push	r18
	push	r19
	push	r20
	push	r21		; PATCH_SWITCH1
	push	r22		; WAVETABLE, PATCH_SWITCH3, temp
	push	r23		; PATCH_SWITCH2
	push	r30		; ZL poiter to tables
	push	r31		; ZH
	push	r0		; MUL result
	push	r1		; dto.

	lds	r21, PATCH_SWITCH1	; Load the mode flag settings so we can check the selected waveform,
	lds	r23, PATCH_SWITCH2	; noise and distortion settings.


;-----------------------------------------------------------------------------
;
; Oscillator A & B
;
; This design uses direct frequency synthesis to generate a ramp wave. A three-byte counter (= phase) is being
; incremented by a value which is proportional to the sound frequency (= phase delta). The
; increment takes place every sampling period. The most significant byte of the counter is a sawtooth ramp.
; This is either used as a pointer to a 256 byte wavetable or for direct waveform synthesis.
; Each oscillator has its own phase and phase delta registers. The contents of each phase delta
; register depends on the frequency being generated:
;
;	PHASE DELTA = 2 ^ 24 * Freq / SamplingFreq
;
; where:
;	SamplingFreq = 40000 Hz
;	Freq = 440 * 2 ^ ((n - 69 + d) / 12)
;	where in turn:
;	n = MIDI note number. Range limited to 36 to 96 (5 octaves)
;	d = transpose/detune (in halftones)
;
;-----------------------------------------------------------------------------


;Calculate DCO A
					; If Noise switch is on, use pseudo-random shift register value
	sbrs	r23, SW_OSCA_NOISE	; Use noise if bit set, otherwise jump to calculate DCO.
	rjmp	CALC_DCOA
.if USE_ORIGINAL_NOISEGEN
	lds	r17, SHIFTREG_2
	sbrc	PHASEA_2,3
	com	r17
	sbrc	PHASEA_2,4
	com	r17
	sbrc	PHASEA_2,6
	com	r17
	sbrc	PHASEA_2,7
	com	r17
	lsl	r17
	rjmp	CALC_DCOB		; skip sample calc for DCO A if noise bit set
.else	; USE_ORIGINAL_NOISEGEN
; use galois implementation of LFSR
	lds	r19, LFSR_0		; r16
	lds	r18, LFSR_1		; r17
	lds	r17, LFSR_2		; r18
	bst	r19, 0			; save lsl of LFSR_0

	clc
	ror	r17			;\
	ror	r18			; > r17:r18:r19 =
	ror	r19			;/  = (LFSR >> 1)

	brtc	NGEN_NOXOR		; test lsb of saved LFSR_0
	ldi	r20, 0xE4		; the taps: b24, b23, b22, b17. This is bit 7, 6, 5 and 0 of the msb.
	eor	r17, r20		;
NGEN_NOXOR:
	sts	LFSR_0, r19
	sts	LFSR_1, r18
	sts	LFSR_2, r17
	rjmp	CALC_DCOB		; skip sample calc for DCO A if noise bit set
.endif	; USE_ORIGINAL_NOISEGEN

; -------------
CALC_DCOA:
	mov	r17, PHASEA_2		; sawtooth ramp for OSCA
	sbrs	r23, SW_OSCA_WAVE	; 0/1 (DCO A = saw/pwm)
	rjmp	DCOA_SAW

;Pulse wave generated by subtracting two bandlimited sawtooths, between 0 and 180 degrees out of phase

	sbrs	r23, SW_PWM_SWEEP
	lds	r20, PULSE_KNOB_LIMITED	; PWM Sweep switch is off, so load the knob value as PWM width
	sbrc	r23, SW_PWM_SWEEP
	lds	r20, PULSE_WIDTH	; PWM Sweep switch is on, load pulse width from LF02
.if USE_ORIGINAL_RAW_OSC
	sbrs	r23, SW_ANTI_ALIAS
	rjmp	RAW_PULSE		; Calculate raw pulse/PWM when anti-alias switch is off
.endif	; USE_ORIGINAL_RAW_OSC
	lds	r22, WAVETABLE_A	; Offset to the correct wavetable, based on note number (0..15)
; r17 phase
; r20 pulse width
; r22 wavetable
	; get sample a into r17
	ldi	ZL, low (2*INV_SAW0)	; Load low part of byte address into ZL
	ldi	ZH, high(2*INV_SAW0)	; Load high part of byte address into ZH
	add	ZL, r17			; Offset the wavetable by the ramp phase (i)
	adc	ZH, r22			; Wavetable 0..15
	lpm				; Load wave(i) into r17

	; get sample b out of phase into r18
	mov	r16, r20		; Grab a copy of the pulse width
	add	r16, r17		; Add phase offset for second table (pulse width + original sample)
	mov	r17, r0			; store previous sample in r17
	ldi	ZL, low (2*INV_SAW0)	; Load low part of byte address into ZL
	ldi	ZH, high(2*INV_SAW0)	; Load high part of byte address into ZH
	add	ZL, r16			; Add phase offset for second table.
	adc	ZH, r22			; Wavetable 0..15
	lpm				; Load wave(i) into r0

	; subtract wave a-b
	; first part b > a, second part b < a
	clr	r16
	sub	r17, r0
	sbc	r16, ZERO
	add	r17, r20		; add offset (pulse width)
	adc	r16, ZERO
	brge	PULSE_BOUND_CHECK	; non-negative result, so no need to limit the value
	ldi	r17, 0
	ldi	r16, 0			; value was negative, so force to zero
PULSE_BOUND_CHECK:
	tst	r16			; Check if we're larger than 255
	breq	PWM_EXIT		; no need to limit upper bound
	ldi	r17, $FF
PWM_EXIT:
	subi	r17, $80		; sign the result
	rjmp	CALC_DCOB

.if USE_ORIGINAL_RAW_OSC
; Raw Pulse wave generated on the fly. Aliases like crazy (calc'd only when anti-alias switch is OFF)
RAW_PULSE:
	cp	r17, r20
	brlo	PULSE_ZERO
	ldi	r17, 255
	subi	r17, $80		; Sign the sample
	rjmp	CALC_DCOB
PULSE_ZERO:
	ldi	r17, 0
	subi	r17, $80		; Sign the sample
	rjmp	CALC_DCOB
.endif ; USE_ORIGINAL_RAW_OSC

; Calculate DCOA sawtooth
DCOA_SAW:
	mov	r17, PHASEA_2
.if USE_ORIGINAL_RAW_OSC
	sbrs	r23, SW_ANTI_ALIAS
	rjmp	DCOA_SAW_SIGN
.endif	; USE_ORIGINAL_RAW_OSC

	lds	r22, WAVETABLE_A	; Offset to the correct wavetable, based on note number (0..15)
	ldi	ZL, low(2*INV_SAW0)	; Load low part of byte address into ZL
	ldi	ZH, high(2*INV_SAW0)	; Load high part of byte address into ZH
	add	ZL, r17			; Offset the wavetable by the ramp phase (i)
	adc	ZH, r22			; Wavetable 0..15
.if MEEBLIP_SE_ORIGINAL_CODE
	lpm				; Load wave(i) into r0
	mov	r17, r0			; Copy into DCO B
.else	; MEEBLIP_SE_ORIGINAL_CODE
	lpm	r17, z
.endif	; MEEBLIP_SE_ORIGINAL_CODE
DCOA_SAW_SIGN:
	subi	r17, $80		; -127..127 Sign oscillator
; r17 hold DCO A waveform

; -------------
;Calculate DCO B
CALC_DCOB:
	lds	r22, WAVETABLE_B	; Offset to the correct wavetable, based on note number (0..15)
.if USE_ORIGINAL_RAW_OSC
	sbrs	r23, SW_ANTI_ALIAS
	ldi	r22, 0			; Use wavetable 0 when anti-alias switch off
.endif	; USE_ORIGINAL_RAW_OSC
	mov	r16, PHASEB_2		; Use ramp value as offset when scanning wavetable
	sbrs	r23, SW_OSCB_WAVE	; 0/1 (DCO B = saw/squ)
	rjmp	LIMITED_TRIB

LIMITED_SQB:				; Square wave lookup
	ldi	ZL, low(2*SQ_LIMIT0)	; Load low part of byte address into ZL
	ldi	ZH, high(2*SQ_LIMIT0)	; Load high part of byte address into ZH
	add	ZL, r16			; Offset the wavetable by the ramp phase (i)
	adc	ZH, r22			; Wavetable 0..15
.if MEEBLIP_SE_ORIGINAL_CODE
	lpm				; Load wave(i) into r0
	mov	r16, r0			; Copy into DCO B
.else	; MEEBLIP_SE_ORIGINAL_CODE
	lpm	r16, z
.endif	; MEEBLIP_SE_ORIGINAL_CODE
	rjmp	CALC_DIST

LIMITED_TRIB:				; Triangle wave lookup
	ldi	ZL, low(2*TRI_LIMIT0)	; Load low part of byte address into ZL
	ldi	ZH, high(2*TRI_LIMIT0)	; Load high part of byte address into ZH
	add	ZL, r16			; Offset the wavetable by the ramp phase (i)
	adc	ZH, r22			; Wavetable 0..15
.if MEEBLIP_SE_ORIGINAL_CODE
	lpm				; Load wave(i) into r0
	mov	r16, r0			; Copy into DCO B
.else	; MEEBLIP_SE_ORIGINAL_CODE
	lpm	r16, z
.endif	; MEEBLIP_SE_ORIGINAL_CODE
; r16 hold DCO B waveform

CALC_DIST:
	subi	r16, $80		; -127..127 Sign Oscillator B waveform

	sbrc	r21, SW_DISTORTION	; 0/1 (OSC DIST = off/on)
	eor	r17, r16

					; Turn off OSCB if not enabled
;	sbrs	r23, SW_OSCB_ENABLE
;	ldi	r16, 0

;-----------------------------------------------------------------------------
; Sum Oscillators
;
; Combines DCOA (in r17) and DCOB (in r16) waves to produce a 16-bit signed result in HDAC:LDAC (r17:r16)
;
;-----------------------------------------------------------------------------

.if USE_ORIGINAL_DCO_FM
	sts	WAVEB,r16		; store signed DCO B wave for fm
.endif	; USE_ORIGINAL_DCO_FM

.if USE_RINGMODULATOR
	lds	r22, PATCH_SWITCH3
	sbrc	r22, SW_MIX_RING
	rjmp	RINGMOD
.endif	;USE_RINGMODULATOR

.if USE_ORIGINAL_MIXER
; mixer:
; Mixer out = (A*x + B*(1-x))/4   x=0..1
	ldi	r22, $7f		; DCOA_LEVEL = 0.5 (128)
	mulsu	r17, r22		; signed DCO A wave * level
	movw	r30, r0			; store value in temp register
	sbrs	r23, SW_OSCB_ENABLE	; if OSC B disabled add OSC A twice
	mov	r16, r17		; (A*x + A*(1-x))/4   x=0..1
	mulsu	r16, r22		; signed DCO B wave * level
	add	r30, r0
	adc	r31, r1			; sum scaled waves
	movw	r16, r30		; place signed output in HDAC:LDAC

	; rotate right a couple of times to make a couple of bits of headroom for resonance.
	asr	r17			;\
	ror	r16			;/ r17:r16 = r17:r16 asr 1
.if USE_ORIGINAL_MIXER_LEVEL
	asr	r17			;\
	ror	r16			;/ r17:r16 = r17:r16 asr 1
.endif	; USE_ORIGINAL_MIXER_LEVEL
	movw	OSC_OUT_L, r16		; keep a copy for highpass filter

.else ; USE_ORIGINAL_MIXER
; mixer:
; Mixer out = (A*x + B*(1-x))/4   x=0..1
	lds	r22, DCOA_LEVEL		; x=0..254/256
	mulsu	r17, r22		; signed DCO A wave * level
	movw	r30, r0			; store value in temp register
	lds	r22, DCOB_LEVEL		; (1-x)=254/256..0
	sbrs	r23, SW_OSCB_ENABLE	; if OSC B disabled add OSC A twice
	mov	r16, r17		; (A*x + A*(1-x))/4   x=0..1
	mulsu	r16, r22		; signed DCO B wave * level
	add	r30, r0
	adc	r31, r1			; sum scaled waves
	movw	r16, r30		; place signed output in HDAC:LDAC

	; rotate right a couple of times to make a couple of bits of headroom for resonance.
	asr	r17			;\
	ror	r16			;/ r17:r16 = r17:r16 asr 1
.if USE_ORIGINAL_MIXER_LEVEL
	asr	r17			;\
	ror	r16			;/ r17:r16 = r17:r16 asr 1
.endif	; USE_ORIGINAL_MIXER_LEVEL
.if USE_RINGMODULATOR
	rjmp	MIXER_END

;-----------------------------------------------------------------------------
; Ringmodulator
;-----------------------------------------------------------------------------
; multiply DCO A (in r17) with DCO B (in r16) output
; skips mixer

RINGMOD:
.if USE_RINGMODULATOR_BALANCE
; (A*x * B*(1-x))/4   x=0..1
; ABx - ABxx = ABx * (1-x)	outpult level max. A*B/4
;
	lds	r22, DCOA_LEVEL		; x=0..254/256
	mulsu	r17, r22		; signed DCO A wave * level
	mov	r17, r1
	lds	r22, DCOB_LEVEL		; (1-x)=254/256..0
	mulsu	r16, r22		; signed DCO B wave * level
	mov	r16, r1
.endif	; USE_RINGMODULATOR_BALANCE

; ring moudulator out = (A*B) or (A*B)/4
	muls	r17, r16		; modulation

.if USE_ORIGINAL_DCO_FM
	sts	WAVEB,r16		; store signed DCO B wave for fm
.endif	; USE_ORIGINAL_DCO_FM
	movw	r16, r0			; place signed output in HDAC:LDAC

.if USE_ORIGINAL_MIXER_LEVEL
.if USE_RINGMODULATOR_BALANCE
	lsl	r16			; increase ringmodulator output +6db
	rol	r17
.else	; USE_RINGMODULATOR_BALANCE
	; rotate right a couple of times to make a couple of bits of headroom for resonance.
	asr	r17			;\
	ror	r16			;/ r17:r16 = r17:r16 asr 1
.endif	; USE_RINGMODULATOR_BALANCE
.endif	; USE_ORIGINAL_MIXER_LEVEL

MIXER_END:
.endif	; USE_RINGMODULATOR
	movw	OSC_OUT_L, r16		; keep a copy for highpass filter
.endif	; USE_ORIGINAL_MIXER

.if !SKIP_FILTER
;----------------------------------------------------------------------
;DCF:
;----------------------------------------------------------------------

;-----------------------------------------------------------------------------
; Digitally Controlled Filter
;
; A 2-pole resonant low pass filter:
;
; a += F * ((in - a) + q * 4 * (a - b));
; b += F * (a - b);
;
; F = LPF (cutoff)
; Q = RESONANCE
; b => output
;
; f = (1-F)/2+32
; q = Q-f = Q-(1-F)/2+32
;
; Input 16-Bit signed HDAC:LDAC (r17:r16), already scaled to minimize clipping (reduced to 25% of full code).
;
;-----------------------------------------------------------------------------

	;calc (in - a) ; both signed
	sub	LDAC, a_L
	sbc	HDAC, a_H
					;check for overflow / do hard clipping
	brvc	OVERFLOW_1		;if overflow bit is clear jump to OVERFLOW_1

					;sub overflow happened -> set to min
					;0b1000.0000 0b0000.0001 -> min
					;0b0111.1111 0b1111.1111 -> max

	ldi	LDAC, 0b00000001
	ldi	HDAC, 0b10000000

OVERFLOW_1:				;when overflow is clear

					;(in-a) is now in HDAC:LDAC as signed
					;now calc q*(a-b)
.if USE_ORIGINAL_Q_CALC
					; Scale resonance based on filter cutoff
	lds	r22, SCALED_RESONANCE
	lds	r20, LPF_I		;load 'F' value
	ldi	r21, 0xff

	sub	r21, r20		; 1-F
	lsr	r21
	ldi	r18, Q_OFFSET
	add	r21, r18		; f = (1-F)/2+32

	sub	r22, r21		; q = Q-f = Q-(1-F/2)+32 = Q+F/2+32-1/2
	brcc	OVERFLOW_2		; if no overflow occured
	ldi	r22, 0x00		;0x00 because of unsigned
OVERFLOW_2:
.else	; USE_ORIGINAL_Q_CALC
	lds	r22, SCALED_RESONANCE	;load filter Q value, unsigned
.endif	; USE_ORIGINAL_Q_CALC

	mov	r20, a_L		;\
	mov	r21, a_H		;/ load 'a' , signed
	lds	z_H, b_H		;\
	lds	z_L, b_L		;/ load 'b', signed
	sub	r20, z_L		;\
	sbc	r21, z_H		;/ (a-b) signed
	brvc	OVERFLOW_3		;if overflow is clear jump to OVERFLOW_3
					;0b1000.0000 0b0000.0001 -> min
					;0b0111.1111 0b1111.1111 -> max
	ldi	r20, 0b00000001
	ldi	r21, 0b10000000

OVERFLOW_3:

	lds	r18, PATCH_SWITCH1	; Check Low Pass/High Pass panel switch.
	sbrs	r18, SW_FILTER_MODE
	rjmp	CALC_LOWPASS
SKIP_REZ:
	movw	z_L,r20			; High Pass selected, so just load r21:r20 into z_H:z_L to disable Q
	rjmp	DCF_ADD			; Skip lowpass calc

CALC_LOWPASS:
	; skip resonance calculation if VCF is turned off (value of 0)
	lds	r18, VCF_STATUS
	tst	r18
	breq	SKIP_REZ
					; mul signed:unsigned -> (a-b) * q
					; 16x8 into 16-bit
					; r19:r18 = r21:r20 (ah:al) * r22 (b)

	mulsu	r21, r22		; (signed)ah * b
	movw	r18, r0
	mul	r20, r22		; al * b
	add	r18, r1
	adc	r19, ZERO
	rol	r0			; r0.7 --> Cy
	brcc	NO_ROUND		; LSByte < $80, so don't round up
	inc	r18
NO_ROUND:
	clc				; (a-b) * q * 4
	lsl	r18
	rol	r19
OVERFLOW_3A:
	clc
	lsl	r18
	rol	r19
OVERFLOW_3B:
	movw	z_L,r18			;q*(a-b) in z_H:z_L as signed

					;add both
					;both signed
					;((in-a)+q*(a-b))
					;=> HDAC:LDAC + z_H:z_L
 DCF_ADD:

	add	LDAC, z_L
	adc	HDAC, z_H

	brvc	OVERFLOW_4		;if overflow is clear
					;0b1000.0000 0b0000.0001 -> min
					;0b0111.1111 0b1111.1111 -> max

	ldi	LDAC, 0b11111111
	ldi	HDAC, 0b01111111

OVERFLOW_4:

					;Result is a signed value in HDAC:LDAC
					;calc * f
					;((in-a)+q*(a-b))*f

	lds	r20, LPF_I		;load lowpass 'F' value
	lds	r18, PATCH_SWITCH1
	sbrc	r18, SW_FILTER_MODE	; Check LP/HP switch.
	lds	r20, HPF_I		; Switch set, so load 'F' for HP

					; mul signed unsigned HDAC*F
					; 16x8 into 16-bit
					; r19:r18 = HDAC:LDAC (ah:al) * r20 (b)

	mulsu	HDAC, r20		; (signed)ah * b
	movw	r18, r0
	mul	LDAC, r20		; al * b
	add	r18, r1			; signed result in r19:r18
	adc	r19, ZERO
	rol	r0			; r0.7 --> Cy
	brcc	NO_ROUND2		; LSByte < $80, so don't round up
	inc	r18
NO_ROUND2:
					;Add result to 'a'
					;a+=f*((in-a)+q*(a-b))

	add	a_L, r18
	adc	a_H, r19
	brvc	OVERFLOW_5		;if overflow is clear
					;0b1000.0000 0b0000.0001 -> min
					;0b0111.1111 0b1111.1111 -> max
	ldi	z_L, 0b11111111
	ldi	z_H, 0b01111111
	mov	a_L, z_L
	mov	a_H, z_H

OVERFLOW_5:
					;calculated a+=f*((in-a)+q*(a-b)) as signed value and saved in a_H:a_L
					;calc 'b'
					;b += f * (a*0.5 - b);

	mov	z_H, a_H		;\
	mov	z_L, a_L		;/ load 'a' as signed

	lds	temp, b_L		;\
	lds	temp2, b_H		;/ load b as signed

	sub	z_L, temp		;\
	sbc	z_H, temp2		;/ (a - b) signed

	brvc	OVERFLOW_6		;if overflow is clear
					;0b1000.0000 0b0000.0001 -> min
					;0b0111.1111 0b1111.1111 -> max

	ldi	z_L, 0b00000001
	ldi	z_H, 0b10000000

OVERFLOW_6:

	lds	r20, LPF_I		;load lowpass 'F' value
	lds	r18, PATCH_SWITCH1
	sbrc	r18, SW_FILTER_MODE	; Check LP/HP switch.
	lds	r20, HPF_I		; Switch set to HP, so load 'F' for HP

					; mul signed unsigned (a-b) * F
					; 16x8 into 16-bit
					; r19:r18 = z_H:z_L (ah:al) * r20 (b)
	mulsu	z_H, r20		; (signed)ah * b
	movw	r18, r0
	mul	z_L, r20		; al * b
	add	r18, r1			; signed result in r19:r18
	adc	r19, ZERO


	add	temp,  r18		;\ add result to 'b' , signed
	adc	temp2, r19		;/ b +=(a-b)*f

	brvc	OVERFLOW_7		;if overflow is clear

					;0b1000.0000 0b0000.0001 -> min
					;0b0111.1111 0b1111.1111 -> max

	ldi	temp,  0b11111111
	ldi	temp2, 0b01111111

OVERFLOW_7:

	sts	b_L, temp		;\
	sts	b_H, temp2		;/ save value of 'b'


	mov	LDAC, temp		;B now contains the filtered signal in HDAC:LDAC
	mov	HDAC, temp2		; output sample HDAC:LDAC = r17:r16


					; If in HP filter mode, just use (filter input - filter output)

	lds	r18, PATCH_SWITCH1	; Check if LP or HP filter
	sbrs	r18, SW_FILTER_MODE
	rjmp	DCA			; LP, so jump to DCA
	sub	OSC_OUT_L, LDAC		; HP filter, so output = filter input - output
	sbc	OSC_OUT_H, HDAC		;
	movw	LDAC, OSC_OUT_L		; output sample HDAC:LDAC = r17:r16
.endif	; SKIP_FILTER

.if !SKIP_AMP
;-----------------------------------------------------------------------------
; Digitally Controlled Amplifier
;
; Multiply the output waveform by the 8-bit value in LEVEL.
; r17:r16 - output from filter 16b signed
; r18     - output from DCA envelope generator
;-----------------------------------------------------------------------------
;
DCA:
	movw	r30, r16
	lds	r18, LEVEL
	cpi	r18, 255
	breq	T2_AEXIT	        ; don't multiply when LEVEL==255, use
	mulsu	r17, r18		; multiply samples high byte
	movw	r30, r0
	mul	r16, r18		; multipliy samples low byte
	add	r30, r1
	adc	r31, ZERO
T2_AEXIT:
.else	; !SKIP_AMP
	movw	r30, r16
.endif	;  !SKIP_AMP

.if USE_MASTER_VOLUME
; r31:30  sample
; r18	  volume +12db..-36db (255..0), 0db = 64
	movw	r16, r30		; sample in r31:r30 -> r17:r16
	lds	r18, VOLUME
	cpi	r18, $40
	breq	VOL_AEXIT	        ; don't multiply when VOLUME = 0x40 (0db), use
	mulsu	r17, r18		; multiply samples high byte
	movw	r30, r0
	mul	r16, r18		; multipliy samples low byte
	add	r30, r1
	adc	r31, ZERO
	lsl	r0			; multiply by 4
	rol	r30
	rol	r31
	lsl	r0
	rol	r30
	rol	r31
VOL_AEXIT:
.endif	; USE_MASTER_VOLUME

;-----------------------------------------------------------------------------
; Output Sample
;
; Write the 16-bit signed output of the DCA to the DAC.
;-----------------------------------------------------------------------------
;

;write sample (r31:r30) to DAC:

	sbi	PORTD, DAC_WR		; Set WR high
	subi	r31, 128		; U2 --> PB
	cbi	PORTD, DAC_AB		; Select DAC port A
	out	PORTC, r31		; output most significant byte
	cbi	PORTD, DAC_WR		; Pull WR low to load buffer A
	sbi	PORTD, DAC_WR		; Set WR high
	sbi	PORTD, DAC_AB		; Select DAC port B
	out	PORTC, r30		; output least significant byte
	cbi	PORTD, DAC_WR		; Pull WR low to load buffer B
	sbi	PORTD, DAC_WR		; Set WR high again

; Increment Oscillator A & B phase

	ldi	r30, low(DELTAA_0)
	ldi	r31, high(DELTAA_0)
	ld	r16, z+
	add	PHASEA_0, r16
	ld	r16,z+
	adc	PHASEA_1, r16
	ld	r16,z+
	adc	PHASEA_2, r16
	ld	r16,z+
	add	PHASEB_0, r16
	ld	r16,z+
	adc	PHASEB_1, r16
	ld	r16, z+
	adc	PHASEB_2,r16

;-----------------------------------------------------------------------------
; Frequency Modulation
;-----------------------------------------------------------------------------
;
.if USE_ORIGINAL_DCO_FM
dco_fm:

	lds	r30, PATCH_SWITCH2
	sbrc	r30, SW_OSCA_NOISE	;
	rjmp	END_SAMPLE_LOOP		; If DCOA waveform is set to Noise, skip FM

	sbrs	r30, SW_OSCB_ENABLE	; Skip FM is OSCB is turned off
	rjmp	END_SAMPLE_LOOP

	lds	r30, PATCH_SWITCH1	; Get FM switch value
	sbrs	r30, SW_OSC_FM
	ldi	r17, 0			; Set FM depth to 0 if switch is off
	sbrc	r30, SW_OSC_FM
.if !USE_MIDI_FMDEPTH
	ldi	r17, 255		; Set FM depth to 255 if switch is on
.else	; !USE_MIDI_FMDEPTH
	lds	r17, FMDEPTH
.endif	; !USE_MIDI_FMDEPTH
					; mod * depth
	tst	r17			; skip if FM depth is zero
	breq	END_SAMPLE_LOOP
	lds	r16, WAVEB

	mulsu	r16, r17		; mod(=waveb) * depth -> r19:r18
	movw	r18, r0

					; delta * mod * depth
	lds	r16, DELTAA_0
	clr	r17
	mulsu	r19, r16
	sbc	r17, r17		; sign extension
	add	PHASEA_0, r1
	adc	PHASEA_1, r17
	adc	PHASEA_2, r17

	lds	r16, DELTAA_1
	mulsu	r19, r16
	add	PHASEA_0, r0
	adc	PHASEA_1, r1
	adc	PHASEA_2, r17

	lds	r16, DELTAA_2
	mulsu	r19, r16
	add	PHASEA_1, r0
	adc	PHASEA_2, r1
.endif ; USE_ORIGINAL_DCO_FM

;-----------------------------------------------------------------------------
; End of Sample Interrupt
;
; Pop register values off stack and return to our regularly scheduled programming.
;-----------------------------------------------------------------------------
;

END_SAMPLE_LOOP:

	pop	r1
	pop	r0
	pop	r31
	pop	r30
	pop	r23
	pop	r22
	pop	r21
	pop	r20
	pop	r19
	pop	r18
	pop	r17
	pop	r16			;\
	out	SREG, r16		;/ pop SREG
	pop	r16
	reti

; -------------
; UART receiver (MIDI IN)
; -------------
UART_RXC:

	push	r16
	in	r16, SREG		;\
	push	r16			;/ push SREG

	in	r16, UDR		; read received byte in r16
	cbi	UCR, 7			; RXCIE=0 (disable UART interrupts)
	sei				; enable other interrupts
	push	r17

	tst	r16			;\ jump when
	brpl	INTRX_DATA		;/ r16.7 == 0 (MIDI data byte)

;MIDI status byte (1xxxxxxx) in r16:
	mov	r17, r16
	andi	r17, 0xF0
	cpi	r17, 0x80
	breq	INTRX_ACCEPT		; 8x note off
	cpi	r17, 0x90
	breq	INTRX_ACCEPT		; 9x note on
	cpi	r17, 0xB0
	breq	INTRX_ACCEPT		; Bx control change
	cpi	r17, 0xE0
	breq	INTRX_ACCEPT		; Ex pitch bend
	ldi	r17, 0			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0
	rjmp	INTRX_EXIT		; Ax polyphonic aftertouch
					; Cx program change
					; Dx channel aftertouch
					; Fx system

INTRX_ACCEPT:
	sts	MIDIPHASE, r17		; phase = 80 90 B0 E0
	andi	r16, 0x0F		;\
	inc	r16			; > store MIDI channel 1..16
	sts	MIDICHANNEL, r16	;/
	lds	r17, SETMIDICHANNEL	;0 for OMNI or 1..15
	tst	r17
	breq	INTRX_ACPT_X		; end when OMNI
	cp	r17, r16		; compare set channel to the incoming channel
	breq	INTRX_ACPT_X		; end when right channel
	ldi	r17, 0			;\ otherwise:
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0 (no data service)

INTRX_ACPT_X:
	rjmp	INTRX_EXIT

;MIDI data byte (0xxxxxxx) in r16:
INTRX_DATA:
	lds	r17, MIDIPHASE
	cpi	r17, 0x80		;\
	breq	INTRX_NOFF1		; \
	cpi	r17, 0x81		; / note off
	breq	INTRX_NOFF2		;/
	rjmp	INTRX_NOTEON

INTRX_NOFF1:
	inc	r17			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0x81
	sts	MIDIDATA0, r16		; MIDIDATA0 = d
	rjmp	INTRX_EXIT

INTRX_NOFF2:
	dec	r17			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0x80
	rjmp	INTRXNON2_OFF

;9x note on:
INTRX_NOTEON:
	cpi	r17, 0x90		;\
	breq	INTRX_NON1		; \
	cpi	r17, 0x91		; / note on
	breq	INTRX_NON2		;/
	rjmp	INTRX_CTRL

INTRX_NON1:
	inc	r17			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0x91
	sts	MIDIDATA0, r16		; MIDIDATA0 = d
	rjmp	INTRX_EXIT

INTRX_NON2:
	dec	r17			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0x90
	tst	r16			;\
	brne	INTRXNON2_ON		;/ jump when velocity != 0

;turn note off:
INTRXNON2_OFF:
	lds	r16, MIDIDATA0
	lds	r17, MIDINOTEPREV
	cp	r16, r17
	brne	INTRXNON2_OFF1
	ldi	r17, 255		;\ remove previous note
	sts	MIDINOTEPREV, r17	;/ from buffer

INTRXNON2_OFF1:
	lds	r17, MIDINOTE
	cp	r16, r17		;\
	brne	INTRXNON2_OFF3		;/ exit when not the same note
	lds	r17, MIDINOTEPREV
	cpi	r17, 255
	breq	INTRXNON2_OFF2
	sts	MIDINOTE, r17		; previous note is valid
	ldi	r17, 255		;\ remove previous note
	sts	MIDINOTEPREV, r17	;/ from buffer

INTRXNON2_OFF3:
	rjmp	INTRX_EXIT

INTRXNON2_OFF2:
	ldi	r17, 255		;\ remove last note
	sts	MIDINOTE, r17		;/
	ldi	r17, 0			;\
	sts	GATE, r17		;/ GATE = 0

	sbi	PORT_MIDI_LED, MIDI_LED	; LED on
	rjmp	INTRX_EXIT

;turn note on:
INTRXNON2_ON:
	sts	MIDIVELOCITY, r16	; store velocity
	lds	r17, MIDINOTE		;\ move previous note
	sts	MIDINOTEPREV, r17	;/ into buffer
	lds	r17, MIDIDATA0		;\
	sts	MIDINOTE, r17		;/ MIDINOTE = note#
	ldi	r17, 1
	sts	GATE, r17		; GATE = 1
	sts	GATEEDGE, r17		; GATEEDGE = 1

	cbi	PORT_MIDI_LED, MIDI_LED	; LED off
	rjmp	INTRX_EXIT

; -------------
;Bx control change:
INTRX_CTRL:
	cpi	r17, 0xB0		;\
	breq	INTRX_CC1		; \
	cpi	r17, 0xB1		; / control change
	breq	INTRX_CC2		;/
	rjmp	INTRX_PBEND

INTRX_CC1:
; controller number in r16
	inc	r17			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0xB1
	sts	MIDIDATA0, r16		; MIDIDATA0 = controller#
	rjmp	INTRX_EXIT

INTRX_CC2:
; parameter value in r16
	dec	r17			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0xB0
	lds	r17, MIDIDATA0		; controller #

;Store MIDI CC in table
	push	r26			; store contents of r27 and r26 on stack
	push	r27

	cpi	r17, $30		; Just save a controller # < $30
	brlo	INTRX_GOSAVE

	cpi	r17, $40		; save, update old knob value and status
	brlo	INTRX_KNOB

	cpi	r17, $50		; save, update old switch value and status
	brlo	INTRX_SW

.if USE_MORE_SWITCHES
	cpi	r17, $60		; for addional switches  (PATCHSWITCH3, PATCH_SWITCH4)
	brlo	INTRX_GOSW_EXT
.endif	; USE_MORE_SWITCHES

INTRX_GOSAVE:
	rjmp	INTRX_SAVE		; Save all other controllers

.if USE_MORE_SWITCHES
INTRX_GOSW_EXT:
	rjmp	INTRX_SW_EXT
.endif	;  USE_MORE_SWITCHES

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

INTRX_KNOB:
; save the value in the MIDI table
	ldi	r26,low(MIDICC)
	ldi	r27,high(MIDICC)
	add	r26,r17
	adc	r27,zero
	lsl	r16			; shift MIDI data to 0..254 to match knob value
	st	x,r16			; store in MIDI CC table

					; Get ADC_X and write it into OLD_ADC_X
	subi	r17, $30		; reduce to 0..15
	cbr	r17, $f8		; Clear highest 5 bits, leaving knob 0..7

	ldi	r26,low(ADC_0)
	ldi	r27,high(ADC_0)
	add	r26,r17
	adc	r27,zero
	ld	r16, x			; Fetch ADC_X into r16

	ldi	r26,low(OLD_ADC_0)
	ldi	r27,high(OLD_ADC_0)
	add	r26,r17
	adc	r27,zero
	st	x, r16			; Store ADC_X in OLD_ADC_X

	ldi	r26,low(KNOB0_STATUS)
	ldi	r27,high(KNOB0_STATUS)
	add	r26,r17
	adc	r27,zero
	ldi	r17, 0
	st	x, r17			; Clear KNOBX_STATUS
	rjmp	INTRX_CCEND

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

INTRX_SW:
	subi	r17, $40		; MIDI CC # --> switch offset 0..15
	cpi	r17, $08
	BRLO	INTRX_SWITCH1		; Jump to Switch 1 if switch 0..7 selected.

INTRX_SWITCH2:
; r16 parameter value = switch state: 0..63=off, 64..127=on
; r17 contoller# = switch
	cbr	r17, $f8		; Clear highest 5 bits, leaving switch 0..7
	lds	r26, PATCH_SWITCH2
	bst	r16, 6			; load MSB of MIDI CC value into SREG T bit
	cpi	r17, 0
	brne	INTRX_S1
	bld	r26, SW_ANTI_ALIAS	; Set bit in PATCH_SWITCH2
	rjmp	INTRX_SEXIT
INTRX_S1:
	cpi	r17, 1
	brne	INTRX_S2
	bld	r26, SW_OSCB_OCT	; Set bit in PATCH_SWITCH2
	rjmp	INTRX_SEXIT
INTRX_S2:
	cpi	r17, 2
	brne	INTRX_S3
	bld	r26, SW_OSCB_ENABLE	; Set bit in PATCH_SWITCH2
	rjmp	INTRX_SEXIT
INTRX_S3:
	cpi	r17, 3
	brne	INTRX_S4
	bld	r26, SW_OSCB_WAVE	; Set bit in PATCH_SWITCH2
	rjmp	INTRX_SEXIT
INTRX_S4:
	cpi	r17, 4
	brne	INTRX_S5
	bld	r26, SW_SUSTAIN		; Set bit in PATCH_SWITCH2
.if !USE_ORIGINAL_ENVELOPE_CONTROL
; set update_env_dca & update_env_dcf flags
	lds	r27, MIDI_TASK_UPDATE
	ori	R27, (1<<UPDATE_ENV_DCF) | (1<< UPDATE_ENV_DCA)
	sts	MIDI_TASK_UPDATE, r27
.endif	; !USE_ORIGINAL_ENVELOPE_CONTROL

	rjmp	INTRX_SEXIT
INTRX_S5:
	cpi	r17, 5
	brne	INTRX_S6
	bld	r26, SW_OSCA_NOISE	; Set bit in PATCH_SWITCH2
	rjmp	INTRX_SEXIT
INTRX_S6:
	cpi	r17, 6
	brne	INTRX_S7
	bld	r26, SW_PWM_SWEEP	; Set bit in PATCH_SWITCH2
	rjmp	INTRX_SEXIT
INTRX_S7:
	bld	r26, SW_OSCA_WAVE	; Set bit in PATCH_SWITCH2

INTRX_SEXIT:				; Finished switch scan, store updated switch bytes
	sts	PATCH_SWITCH2, r26
	rjmp	INTRX_CCEND

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

INTRX_SWITCH1:
; r16 parameter value = switch state: 0..63=off, 64..127=on
; r17 contoller# = switch
	cbr	r17, $f8		; Clear highest 5 bits, leaving switch 0..7
	lds	r26, PATCH_SWITCH1
	bst	r16, 6			; load MSB of MIDI CC value into SREG T bit

	cpi	r17, 0
	brne	INTRX_SW1
	bld	r26, SW_KNOB_SHIFT	; Set bit in PATCH_SWITCH1
	rjmp	INTRX_SWEXIT
INTRX_SW1:
	cpi	r17, 1
	brne	INTRX_SW2
	bld	r26, SW_OSC_FM		; Set bit in PATCH_SWITCH1
	rjmp	INTRX_SWEXIT
INTRX_SW2:
	cpi	r17, 2
	brne	INTRX_SW3
	bld	r26, SW_LFO_RANDOM	; Set bit in PATCH_SWITCH1
	rjmp	INTRX_SWEXIT
INTRX_SW3:
	cpi	r17, 3
	brne	INTRX_SW4
	bld	r26, SW_LFO_WAVE	; Set bit in PATCH_SWITCH1
	rjmp	INTRX_SWEXIT
INTRX_SW4:
	cpi	r17, 4
	brne	INTRX_SW5
	bld	r26, SW_FILTER_MODE	; Set bit in PATCH_SWITCH1
	rjmp	INTRX_SWEXIT
INTRX_SW5:
	cpi	r17, 5
	brne	INTRX_SW6
	bld	r26, SW_DISTORTION	; Set bit in PATCH_SWITCH1
	rjmp	INTRX_SWEXIT
INTRX_SW6:
	cpi	r17, 6
	brne	INTRX_SW7
	bld	r26, SW_LFO_ENABLE	; Set bit in PATCH_SWITCH1
	rjmp	INTRX_SWEXIT
INTRX_SW7:
	bld	r26, SW_LFO_DEST	; Set bit in PATCH_SWITCH1

INTRX_SWEXIT:				; Finished switch scan, store updated switch bytes
	sts	PATCH_SWITCH1, r26
	rjmp	INTRX_CCEND

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

.if USE_MORE_SWITCHES
INTRX_SW_EXT:
	subi	r17, $50		; MIDI CC # --> switch offset 0..15
	cpi	r17, $08
	BRLO	INTRX_SWITCH3		; Jump to Switch 1 if switch 0..7 selected.

INTRX_SWITCH4:
; r16 parameter value = switch state: 0..63=off, 64..127=on
; r17 contoller# = switch
	cbr	r17, $f8		; Clear highest 5 bits, leaving switch 0..7
	lds	r26, PATCH_SWITCH4
	bst	r16, 6			; load MSB of MIDI CC value into SREG T bit

	cpi	r17, 0
	brne	INTRX_SW41
	bld	r26, SW_OSCA_WAVE_EXT	; Set bit in PATCH_SWITCH4
	rjmp	INTRX_SW4EXIT
INTRX_SW41:
	cpi	r17, 1
	brne	INTRX_SW42
	bld	r26, SW_USER_4_1	; Set bit in PATCH_SWITCH4
	rjmp	INTRX_SW4EXIT
INTRX_SW42:
	cpi	r17, 2
	brne	INTRX_SW43
	bld	r26, SW_OSCB_WAVE_EXT	; Set bit in PATCH_SWITCH4
	rjmp	INTRX_SW4EXIT
INTRX_SW43:
	cpi	r17, 3
	brne	INTRX_SW44
	bld	r26, SW_USER_4_3	; Set bit in PATCH_SWITCH4
	rjmp	INTRX_SW4EXIT
INTRX_SW44:
	cpi	r17, 4
	brne	INTRX_SW45
	bld	r26, SW_USER_4_4	; Set bit in PATCH_SWITCH4
	rjmp	INTRX_SW4EXIT
INTRX_SW45:
	cpi	r17, 5
	brne	INTRX_SW46
	bld	r26, SW_USER_4_5	; Set bit in PATCH_SWITCH4
	rjmp	INTRX_SW4EXIT
INTRX_SW46:
	cpi	r17, 6
	brne	INTRX_SW47
	bld	r26, SW_LFO2_WAVE	; Set bit in PATCH_SWITCH4
	rjmp	INTRX_SW4EXIT
INTRX_SW47:
	bld	r26, SW_LFO2_RANDOM	; Set bit in PATCH_SWITCH4

INTRX_SW4EXIT:				; Finished switch scan, store updated switch bytes
	sts	PATCH_SWITCH4, r26
	rjmp	INTRX_CCEND

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

INTRX_SWITCH3:
; r16 parameter value = switch state: 0..63=off, 64..127=on
; r17 contoller# = switch
	cbr	r17, $f8		; Clear highest 5 bits, leaving switch 0..7
	lds	r26, PATCH_SWITCH3
	bst	r16, 6			; load MSB of MIDI CC value into SREG T bit

	cpi	r17, 0
	brne	INTRX_SW31
	bld	r26, SW_USER_3_0	; Set bit in PATCH_SWITCH3
	rjmp	INTRX_SW3EXIT
INTRX_SW31:
	cpi	r17, 1
	brne	INTRX_SW32
	bld	r26, SW_USER_3_1	; Set bit in PATCH_SWITCH3
	rjmp	INTRX_SW3EXIT
INTRX_SW32:
	cpi	r17, 2
	brne	INTRX_SW33
	bld	r26, SW_MIX_RING	; Set bit in PATCH_SWITCH3
	rjmp	INTRX_SW3EXIT
INTRX_SW33:
	cpi	r17, 3
	brne	INTRX_SW34
	bld	r26, SW_TRANSPOSE	; Set bit in PATCH_SWITCH3
	rjmp	INTRX_SW3EXIT
INTRX_SW34:
	cpi	r17, 4
	brne	INTRX_SW35
	bld	r26, SW_DCA_MODE	; Set bit in PATCH_SWITCH3
	rjmp	INTRX_SW3EXIT
INTRX_SW35:
	cpi	r17, 5
	brne	INTRX_SW36
	bld	r26, SW_MODWHEEL_ENABLE	; Set bit in PATCH_SWITCH3
	rjmp	INTRX_SW3EXIT
INTRX_SW36:
	cpi	r17, 6
	brne	INTRX_SW37
	bld	r26, SW_LFO_KBD_SYNC	; Set bit in PATCH_SWITCH3
	rjmp	INTRX_SW3EXIT
INTRX_SW37:
	bld	r26, SW_DCF_KBD_TRACK	; Set bit in PATCH_SWITCH24

INTRX_SW3EXIT:				; Finished switch scan, store updated switch bytes
	sts	PATCH_SWITCH3, r26
	rjmp	INTRX_CCEND
.endif	; USE_MORE_SWITCHES

;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

INTRX_SAVE:
; r16 parameter  value
; r17 contoller#
	ldi	r26,low(MIDICC)
	ldi	r27,high(MIDICC)
	add	r26,r17
	adc	r27,zero
	lsl	r16			; shift MIDI data to 0..254 to match knob value
	st	x,r16			; store in MIDI CC table

.if USE_EXTENDED_MIDI_CONTROL
; for some midi controllers do a special processing

.if !USE_ORIGINAL_ENVELOPE_CONTROL
	cpi	r17, $16
	brlo	INTRX_CCLOW
	breq	INTRX_AMP_ATTACK
	cpi	r17, $1A
	brlo	INTRX_AMP_DECAY
	breq	INTRX_DCF_ATTACK
	cpi	r17, $1E
	brcc	INTRX_CCHIGH

INTRX_DCF_DECAY:
	ldi	r17, 2			; knob 2
	rjmp	INTRX_UPD_KNOB

INTRX_AMP_DECAY:
	ldi	r17, 4			; knob 4
	rjmp	INTRX_UPD_KNOB

INTRX_DCF_ATTACK:
	sts	KNOB_DCF_ATTACK, r16	; sync with ATTACKTIME2
	ldi	r17, 3			; knob 3
	rjmp	INTRX_UPD_KNOB

INTRX_AMP_ATTACK:
	sts	KNOB_AMP_ATTACK, r16	; sync with ATTACKTIME
	ldi	r17, 5			; knob 5

INTRX_UPD_KNOB:
; update knob status and setup OLD_ADC value
	ldi	r26,low(ADC_0)
	ldi	r27,high(ADC_0)
	add	r26,r17
	adc	r27,zero
	ld	r16, x			; Fetch ADC_X into r16

	ldi	r26,low(OLD_ADC_0)
	ldi	r27,high(OLD_ADC_0)
	add	r26,r17
	adc	r27,zero
	st	x, r16			; Store ADC_X in OLD_ADC_X

	ldi	r26,low(KNOB0_STATUS)
	ldi	r27,high(KNOB0_STATUS)
	add	r26,r17
	adc	r27,zero
	ldi	r17, 0
	st	x, r17			; Clear KNOBX_STATUS
	rjmp	INTRX_CCEND
.endif	; USE_MORE_SWITCHES

; -------------
INTRX_CCLOW:	; midi cc $0E..$15
.if USE_MASTER_VOLUME
	cpi	r17, 0x15		; MASTER_VOLUME
	brne	INTRX_CC3
	push	r16
	lds	r16, MIDI_TASK_UPDATE	; are there any open update tasks?
	ori	r16, (1<<UPDATE_VOLUME)
	sts	MIDI_TASK_UPDATE, r16
	pop	r16
INTRX_CC3:
.endif	; USE_MASTER_VOLUME
	cpi	r17, 0x14		; MIXER_BALANCE
	brne	INTRX_CCEND

; r16 midi cc MIXER_BALANCE value 0..254
; set DCOA_LEVEL, DCOB_LEVEL
	ldi	r17, $fe		; value 0..254 for DCOB volume calcs
	sub	r17, r16		; knob 254.0 for DCOA volume calcs
	cpi	r17, $02
	BRSH	SET_DCOA		; skip if knob value >= 2
	ldi	r17, $0

SET_DCOA:
	sts	DCOA_LEVEL, r17		; Store DCOA level (0..254)

	cpi	r16, $2
	BRSH	SET_DCOB		; skip if knob value >= 2
	ldi	r16, $0
SET_DCOB:
	sts	DCOB_LEVEL, r16		; Store DCOB level(0..254)
.endif	; USE_EXTENDED_MIDI_CONTROL
;	rjmp	INTRX_CCEND

; -------------
INTRX_CCHIGH:	; midi parameter $1e..2F
	; currently nothing to do

INTRX_CCEND:
	pop	r27			; reload old contents of r27 and r 26
	pop	r26
	rjmp	INTRX_EXIT

;Ex pitch bender:
INTRX_PBEND:
	cpi	r17, 0xE0		;\
	breq	INTRX_PB1		; \
	cpi	r17, 0xE1		; / pitch bend
	breq	INTRX_PB2		;/
	rjmp	INTRX_EXIT

INTRX_PB1:
	inc	r17			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0xE1
	sts	MIDIDATA0, r16		; MIDIDATA0 = dFine  0..127
	rjmp	INTRX_EXIT

INTRX_PB2:
	dec	r17			;\
	sts	MIDIPHASE, r17		;/ MIDIPHASE = 0xE0
	lds	r17,MIDIDATA0		;\
	lsl	r17			;/ r17 = dFine*2	0..254
	lsl	r17			;\ r16,r17 = P.B.data
	rol	r16			;/ 0..255,996
	subi	r16, 128		; r16,r17 = -128,000..+127,996
	sts	MIDIPBEND_L, r17	;\
	sts	MIDIPBEND_H, r16	;/ store P.BEND value
	rjmp	INTRX_EXIT

INTRX_EXIT:
	pop	r17
	pop	r16			;\
	out	SREG, r16		;/ pop SREG
	pop	r16
	sbi	UCR, 7			; RXCIE=1
	reti

;-----------------------------------------------------------------------------
;	M A I N   L E V E L   S U B R O U T I N E S
;-----------------------------------------------------------------------------

;=============================================================================
;	Delay subroutines
;=============================================================================

WAIT_10US:
	push	r16			; 3+2
	ldi	r16, 50			; 1

W10U_LOOP:
	dec	r16			; 1\
	brne	W10U_LOOP		; 2/1 / 49*3 + 2
	pop	r16			; 2
	ret				; 4

;=============================================================================
;	I/O subroutines
;=============================================================================

;-----------------------------------------------------------------------------
;A/D conversion (start)
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r18 = channel #	0..7
;Out:   -
;Used:  -
;-----------------------------------------------------------------------------
ADC_START:
	out	ADMUX, r18		; set multiplexer
	sbi	ADCSRA, 6		; ADSC=1
	ret

;-----------------------------------------------------------------------------
;A/D conversion (end)
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	-
;Out:	r16 = result		0..255
;Used:	SREG,r17
;-----------------------------------------------------------------------------
ADC_END:
ADCE_LOOP:
	sbis	ADCSRA, 4		;\
	rjmp	ADCE_LOOP		;/ wait for ADIF==1
	sbi	ADCSRA, 4		; clear ADIF
	in	r16, ADCL		;\
	in	r17, ADCH		;/ r17:r16 = 000000Dd:dddddddd
	lsr	r17			;\
	ror	r16			;/ r17:r16 = 0000000D:dddddddd
	lsr	r17			;\
	ror	r16			;/ r16 = Dddddddd
	ret

;=============================================================================
;	arithmetic subroutines
;=============================================================================

;-----------------------------------------------------------------------------
; 16 bit arithmetical shift right (division by 2^n)
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r17:r16 = x
;	r18 = n (shift count)	0..16
;Out:	r17:r16 = x asr n
;Used:	SREG
;-----------------------------------------------------------------------------
ASr16:
	tst	r18
	breq	ASr16_EXIT
	push	r18

ASr16_LOOP:
	asr	r17			;\
	ror	r16			;/ r17,r16 = r17,r16 asr 1
	dec	r18
	brne	ASr16_LOOP
	pop	r18

ASr16_EXIT:
	ret

;-----------------------------------------------------------------------------
; 32 bit logical shift right
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r19:r18:r17:r16 = x
;	r20 = n (shift count)
;Out:	r19:r18:r17:r16 = x >> n
;Used:	SREG
;-----------------------------------------------------------------------------
SHR32:
	tst	r20
	breq	SHR32_EXIT
	push	r20

SHR32_LOOP:
	lsr	r19
	ror	r18
	ror	r17
	ror	r16
	dec	r20
	brne	SHR32_LOOP
	pop	r20

SHR32_EXIT:
	ret

;-----------------------------------------------------------------------------
; 32 bit logical shift left
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r19:r18:r17:r16 = x
;	r20 = n (shift count)
;Out:	r19:r18:r17:r16 = x << n
;Used:	SREG
;-----------------------------------------------------------------------------
SHL32:
	tst	r20
	breq	SHL32_EXIT
	push	r20

SHL32_LOOP:
	lsl	r16
	rol	r17
	rol	r18
	rol	r19
	dec	r20
	brne	SHL32_LOOP
	pop	r20

SHL32_EXIT:
	ret

;-----------------------------------------------------------------------------
;8 bit x 8 bit multiplication (unsigned)
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r16 = x			0..255
;	r17 = y			0,000..0,996
;Out:	r17,r16 = x * y		0,000..254,004
;Used:	SREG,r18-r20
;-----------------------------------------------------------------------------
MUL8X8U:

	MUL	r16, r17
	movw	r16,r0
	ret

;-----------------------------------------------------------------------------
;32 bit x 16 bit multiplication (unsigned)
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r19:r18:r17:r16 = x		0..2^32-1
;	r23:r22 = y			0,yyyyyyyyyyyyyyyy  0..0,9999847
;Out:	r19:r18:r17:r16 = x * y	0..2^32-1
;Used:	SREG,r20-r29
;-----------------------------------------------------------------------------
MUL32X16:
; multiply with high multiplier x
	mul	r19, r23		; ax
	movw	r29:r28, r1:r0
	mul	r18, r23		; bx
	movw	r21:r20, r1:r0
	mul	r17, r23		; cx
	movw	r27:r26, r1:r0
	mul	r16, r23		; dx
	mov	r25, r0
	add	r26, r1
	adc	r27, r20
	adc	r28, r21
	adc	r29, ZERO
; multiply with low multiplier y
	mul	r19, r22		; ay
	movw	r21:r20, r1:r0
	mul	r17, r22		; cy
	add	r25, r0
	adc	r26, r1
	adc	r27, r20
	adc	r28, r21
	adc	r29, ZERO
	mul	r18, r22		; by
	movw	r21:r20, r1:r0
	mul	r16, r22		; dy
	mov	r24, r0
	add	r25, r1
	adc	r26, r20
	adc	r27, r21
	adc	r28, ZERO
	adc	r29, ZERO

	mov	r16,r26			;\
	mov	r17,r27			; \
	mov	r18,r28			; / x * y
	mov	r19,r29			;/

	ret

;-----------------------------------------------------------------------------
; Load 32 bit phase value from ROM
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r30 = index
;Out:	r19:r18:r17:r16 = value
;Used:	SREG,r0,r30,r31
;-----------------------------------------------------------------------------
LOAD_32BIT:
	lsl	r30			; r30 *= 2
	ldi	r31, 0
	adiw	r30, DELTA_C		; Z = ROM address
	add	r30, r30
	adc	r31, r31
	lpm	r16, z+
	lpm	r17, z+
	lpm	r18, z+
	lpm	r19, z+
	ret

;-----------------------------------------------------------------------------
; Load phase delta from ROM
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r23,r22 = indexs = 0,0..12,0 = note
;       r20 = 0..11, octave
;Out:	r19:r18:r17:r16 = delta
;Used:	SREG,r0,r21,r24-r31
;-----------------------------------------------------------------------------
LOAD_DELTA:
	push	r22
	push	r23
	mov	r30, r23
	rcall	LOAD_32BIT		; load delta value for specified note
	mov	r24, r16
	mov	r25, r17
	mov	r26, r18
	mov	r27, r19		; r27-r24 = delta[n]
	mov	r30, r23
	inc	r30			; next note
	rcall	LOAD_32BIT		; load delata value for specified note
	sub	r16, r24
	sbc	r17, r25
	sbc	r18, r26		; r27-r24 = delta[n+1]
	sbc	r19, r27		; r19-r16 = delta[n+1]-delta[n]
	push	r24			; save delta[n]
	push	r25
	push	r26
	push	r27
	mov	r23, r22
	ldi	r22, 0
	push	r20			; r22 * (delta[n+1]-delta[n])
	rcall	MUL32X16		; r19:r18:r17:r16 = r19:r18:r17:r16 * r23:r22
	pop	r20			; recal delta[n]
	pop	r27
	pop	r26
	pop	r25
	pop	r24
	add	r16, r24		; delta[n] + r22 * (delta[n+1]-delta[n])
	adc	r17, r25
	adc	r18, r26
	adc	r19, r27
	pop	r23
	pop	r22
	ret

;-----------------------------------------------------------------------------
;note number recalculation
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r23 = n		0..139 = m12 + 12*n12
;Out:	r23 = m12	0..11 = note
;	r20 = n12	0..11 = octave
;Used:	SREG
;-----------------------------------------------------------------------------
NOTERECALC:
	ldi	r20,0			; n12 = 0
	rjmp	NRC_2

NRC_1:
	subi	r23, 12			; m12 -= 12
	inc	r20			; n12++

NRC_2:
	cpi	r23, 12
	brsh	NRC_1			; repeat while m12 >= 12
	ret

;-----------------------------------------------------------------------------
;read a byte from a table
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r16 = i		0..255
;	r31:r30 = &Tab
;Out:	r16 = Tab[i]	0..255
;Used:	SREG,r30,r31
;-----------------------------------------------------------------------------
TAB_BYTE:
	add	r30, r30		;\
	adc	r31, r31		;/ Z = 2 * &Tab
	add	r30, r16
	adc	r31, ZERO
	lpm	r16, z
	ret

;-----------------------------------------------------------------------------
;read a word from a table
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r16 = i			0..255
;	r31:r30 = &Tab
;Out:	r19:r18 = Tab[i]	0..65535
;Used:	SREG,r0,r30,r31
;-----------------------------------------------------------------------------
TAB_WORD:
	add	r30, r16
	adc	r31, ZERO
	add	r30, r30		;\
	adc	r31, r31		;/ Z = 2 * &Tab
	lpm	r18, z+			; LSByte
	lpm	r19, z			; MSByte
	ret

;-----------------------------------------------------------------------------
;"time" --> "rate" conversion
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r16 = time			0..255
;Out:	r19:r18:r17:r16 = rate		0x001B0000..0xFFFF0000
;Used:	SREG,r0,r30,r31
;-----------------------------------------------------------------------------
ADCTORATE:
	lsr	r16
	lsr	r16
	lsr	r16			;0..31
	ldi	r30, low( TIMETORATE)
	ldi	r31, high(TIMETORATE)
	rcall	TAB_WORD		;r19:r18 = rate
	clr	r16
	clr	r17
	ret

;-----------------------------------------------------------------------------
;conversion of the "detune B" potentiometer function
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r16 = x		0..255
;Out:	r17,r16 = y	0,000..255,996
;Used:	SREG,r18-r30
;-----------------------------------------------------------------------------
NONLINPOT:
	ldi	r22, 0
	mov	r23, r16
	cpi	r23, 112
	brlo	NLP_I
	cpi	r23, 144
	brlo	NLP_II
	rjmp	NLP_III

NLP_I:
	ldi	r16, 0			;\  r18,r17:r16 = m =
	ldi	r17, 32			; > = 126/112 =
	ldi	r18, 1			;/  = 1,125
	ldi	r30, 0			;\ r31,r30 = n =
	ldi	r31, 0			;/ = 0,0
	rjmp	NLP_CONT

NLP_II:
	ldi	r16, 8			;\  r18,r17:r16 = m =
	ldi	r17, 33			; > = (130-126)/(143-112) =
	ldi	r18, 0			;/  = 0,129032258
	ldi	r30, 140		;\ r31,r30 = n =
	ldi	r31, 111		;/ = 126 - m*112 = 111,5483871
	rjmp	NLP_CONT

NLP_III:
	ldi	r16, 183		;\  r18,r17:r16 = m =
	ldi	r17, 29			; > = (255-130)/(255-143) =
	ldi	r18, 1			;/  = 1,116071429
	ldi	r30, 103		;\ r31,r30 = n =
	ldi	r31, 226		;/ 255 - m*255 = -29,59821429

NLP_CONT:
	ldi	r19, 0
	rcall	MUL32X16		; r19:r18:r17:r16 = r19:r18:r17:r16 * r23:r22
	add	r16, r30
	adc	r17, r31
	ret

;-----------------------------------------------------------------------------
; Write byte to eeprom memory
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r16	= value		0..255
;	r18:r17 = eeprom memory address
;Used:	r16, r17, r18
;-----------------------------------------------------------------------------
EEPROM_write:
					; Wait for completion of previous write
	sbic	EECR,EEWE
	rjmp	EEPROM_write		; loop until eeprom is ready
	in	temp_SREG, SREG		; save SREG
	cli				; disable interrupts during timed eeprom sequence
	out	EEARH, r18
	out 	EEARL, r17		; single byte offset from WRITE_OFFSET
	out 	EEDR,  r16		; Write data (r16) to data register
	sbi 	EECR,EEMWE		; Write logical one to EEMWE
	sbi 	EECR,EEWE		; Start eeprom write by setting EEWE
	out	SREG, temp_SREG		; restore SREG (restarts interrupts if enabled)
	ret

;-----------------------------------------------------------------------------
; Read byte from eeprom memory
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r18:r17 = eeprom memory address
;Out:	r16	= value  0..255
;Used:	r16, r17, r18
;-----------------------------------------------------------------------------
EEPROM_read:

	sbic 	EECR,EEWE		; Wait for completion of previous write
	rjmp	EEPROM_read
	in	temp_SREG, SREG		; save SREG
	cli				; disable interrupts during timed eeprom sequence
	out 	EEARH, r18		; Set up address (r18:r17) in address register
	out	EEARL, r17
	sbi	EECR, EERE		; Start eeprom read by writing EERE
	in	r16, EEDR		; Read data from data register
	out	SREG, temp_SREG 	; restore SREG (restarts interrupts if enabled)
	ret

;-----------------------------------------------------------------------------
; Clear knob status
; Set knob status to 'unmoved' and save current knob positions
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	    --
;Out:		--
;Used:	    r16
;-----------------------------------------------------------------------------

CLEAR_KNOB_STATUS:			;	set knob status to 'unmoved' and save current knob positions
	clr	r16
	sts	KNOB0_STATUS, r16	;
	sts	KNOB1_STATUS, r16	;
	sts	KNOB2_STATUS, r16	;
	sts	KNOB3_STATUS, r16	;
	sts	KNOB4_STATUS, r16	;
	sts	KNOB5_STATUS, r16	;
	sts	KNOB6_STATUS, r16	;
	sts	KNOB7_STATUS, r16	;

	lds	r16, ADC_0		; Save current pot positions for future comparison
	sts	OLD_ADC_0,r16
	lds	r16, ADC_1
	sts	OLD_ADC_1,r16
	lds	r16, ADC_2
	sts	OLD_ADC_2,r16
	lds	r16, ADC_3
	sts	OLD_ADC_3,r16
	lds	r16, ADC_4
	sts	OLD_ADC_4,r16
	lds	r16, ADC_5
	sts	OLD_ADC_5,r16
	lds	r16, ADC_6
	sts	OLD_ADC_6,r16
	lds	r16, ADC_7
	sts	OLD_ADC_7,r16
	ret

;-----------------------------------------------------------------------------
; Load patch
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
;In:	r16 = patch number
;Out:	Loads 16/48 byte patch into MIDI CC table
;Used:	r16-r19, r28, r29
;-----------------------------------------------------------------------------

; r16	patch number
; r17	Low byte of eeprom address
; r18	High byte of eeprom address
; r19	MIDI CC table offset
; r28	\
; r28	/ Y pointer to midi CC

LOAD_PATCH:
					; Enter with patch number in r16
	lsl	r16
	lsl	r16
	lsl	r16
	lsl	r16			; multiply patch number by 16 to get starting address of patch in eeprom
	mov	r17, r16		; Low byte of eeprom address
	ldi	r18, 0			; High byte of eeprom address
.if USE_EXTENDED_PATCH
	lsl	r17			; 3 * patch number -> load 48 byte
	rol	r18			;  (r18:r17)*2+(0:r16)
	add	r17, r16		; Low byte of eeprom address
	adc	r18, ZERO		; High byte of eeprom address
.endif	; USE_EXTENDED_PATCH

	ldi	r19, $30 		; MIDI CC table offset,  load start addr of first block
					; Get byte from eeprom
PATCH_LOOP:
.if USE_EXTENDED_PATCH
; map           eeprom address -> midi cc address
; meeblip midi cc	 0..0F -> 30..3F  first midi cc block
; my midi cc extentions 10..1F -> 0E..1D  second midi cc block
; my switches SW1..SW4	20..23 -> 2C..2F  third midi cc block
;
; r19     midi cc address
; r18:r17 eeprom byte offset (incremented in loop)
	cpi 	r19, $40		; in first midi cc block? addr < end adder of first block
	brne	LD_PATCH_1		; yes
	ldi	r19, $0E		; no: second midi cc block, load start addr of second block
LD_PATCH_1:
	cpi 	r19, $1E		; in second midi cc block? addr < end adder of second block
	brne	LD_PATCH_2		; yes
	ldi	r19, $2C		; no: third midi cc block, load start addr of third block
LD_PATCH_2:
.endif	; USE_EXTENDED_PATCH
	rcall	EEPROM_READ		; address r18:r17, returns patch(i) in r16

	ldi	r28, low (MIDICC)
	ldi	r29, high(MIDICC)
	add	r28, r19
	adc	r29, zero
	st	Y, r16			; store in MIDI CC table

	inc	r17			; increment eeprom address
.if USE_EXTENDED_PATCH
	adc	r18, ZERO
.endif	; USE_EXTENDED_PATCH
	inc	r19			; increment midi cc offset
.if !USE_EXTENDED_PATCH
	cpi	r19, $40		; are we finished loading 16 bytes? last midi cc addr?
.else	; !USE_EXTENDED_PATCH
	cpi	r19, $30		; are we finished loading 36 bytes? last midi cc addr?
.endif	; !USE_EXTENDED_PATCH
	brne	PATCH_LOOP

	; copy switch bytes from MIDI table to current patch
	lds	r16, SW1
	sts	PATCH_SWITCH1, r16
	lds	r16, SW2
	sts	PATCH_SWITCH2, r16
.if USE_EXTENDED_MIDI_CONTROL
	; copy also other switches
	lds	r16, SW3
	sts	PATCH_SWITCH3, r16
	lds	r16, SW4
	sts	PATCH_SWITCH4, r16
.endif	; USE_EXTENDED_MIDI_CONTROL
	lds	r16, SWITCH1
	sts	OLD_SWITCH1, r16
	lds	r16, SWITCH2
	sts	OLD_SWITCH2, r16

	; flag knobs as not moved
	rcall	CLEAR_KNOB_STATUS
	ret

;-----------------------------------------------------------------------------
; Scan a pot and update its value if it has been moved
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
; In:	r16 - the new pot value
;	r20 - the current conversion channel (0..7)
; Out:	r17 - 1 if the pot has changed, 0 otherwise
; Used:	r16-r20, r28, r29, SREG
;-----------------------------------------------------------------------------
POT_SCAN:
	ldi	r28, low(KNOB0_STATUS)
	ldi	r29, high(KNOB0_STATUS)
	add	r28, r20
	adc	r29, ZERO
	ld	r18, Y			; load KNOBN_STATUS value into r18
	sbrc	r18, 0			; Check bit 0
	rjmp	LOAD_ADC		; KNOBN_STATUS is set, so just update parameter
	mov	r19, r16

	ldi	r28, low(OLD_ADC_0)
	ldi	r29, high(OLD_ADC_0)
	add	r28, r20
	adc	r29, ZERO
	ld	r17, Y			; load OLD_ADC_N value into r17
	sub	r19, r17
	brpl	DEAD_CHECK
	neg	r19
DEAD_CHECK:
	cpi	r19, PARAM_DEAD_ZONE
	brlo	NO_CHANGE		; Skip ahead if pot change is < the deadzone limit
	sbr	r18,1			; Update knob status bit and continue -- pot moved
	ldi	r28, low(KNOB0_STATUS)
	ldi	r29, high(KNOB0_STATUS)
	add	r28, r20		;
	adc	r29, ZERO
	st	Y, r18			; save updated KNOBN_STATUS
	rjmp	LOAD_ADC		;

NO_CHANGE:
	ldi	r17, 0			; flag pot unchanged
	ret

LOAD_ADC:
	ldi	r17, 1			; flag pot changed
	ret

;-----------------------------------------------------------------------------
;	M A I N   P R O G R A M
;-----------------------------------------------------------------------------
RESET:
	cli				; disable interrupts

;JTAG Disable - Set JTD in MCSCSR
	lds	r16, MCUCSR		; Read MCUCSR
	sbr	r16, 1 << JTD		; Set jtag disable flag
	out	MCUCSR, r16		; Write MCUCSR
	out	MCUCSR, r16		; and again as per datasheet

;initialize stack:
	ldi	r16, low(RAMEND)
	ldi	r17, high(RAMEND)
	out	SPL, r16
	out	SPH, r17

;initialize variables:
	clr	ZERO
	clr	PHASEA_0
	clr	PHASEA_1
	clr	PHASEA_2
	clr	PHASEB_0
	clr	PHASEB_1
	clr	PHASEB_2

	clr	a_L			; clear DCF registers	(lp_L)
	clr	a_H			;			(lp_H)
	clr	z_L			;
	clr	z_H			;
	clr	temp			;
	clr	temp2			;

	ldi	r16, 5
	sts	KNOB_DEADZONE, r16	; not used

	ldi	r16, 0
	sts	LED_STATUS, r16		; LED status = off
	sts	LED_TIMER, r16		;
	sts	BUTTON_STATUS, r16	; no buttons pressed
	sts	CONTROL_SWITCH, r16	; no switches flipped
	sts	SETMIDICHANNEL, r16	; Default MIDI channel to zero (omni)
	sts	SWITCH3, r16		; MIDI/Save/Load switch hasn't been scanned yet, so clear it
.if USE_ORIGINAL_DCO_FM
	sts	FMDEPTH, r16		; FM Depth = 0
.endif	; USE_ORIGINAL_DCO_FM

.if !USE_ORIGINAL_ENVELOPE_CONTROL
	sts	MIDI_TASK_UPDATE, r16	; no active task
.endif	;  !USE_ORIGINAL_ENVELOPE_CONTROL

.if USE_EXTENDED_MIDI_CONTROL
	sts	RESONANCE, r16		; Resonance = 0
	sts	PORTAMENTO, r16		; Portamento = 0
	sts	SETMIDICHANNEL, r16	; 0 for OMNI or 1..15
.endif	; USE_EXTENDED_MIDI_CONTROL
	sts	GATE, r16		; GATE = 0
	sts	GATEEDGE, r16		; GATEEDGE = 0
	sts	LEVEL, r16		; LEVEL = 0
	sts	ENV_FRAC_L, r16		;\
	sts	ENV_FRAC_H, r16		; > ENV = 0
	sts	ENV_INTEGR, r16		;/
.if USE_INIT_LFO2
	sts	ENV_FRAC_L2, r16	;\
	sts	ENV_FRAC_H2, r16	; > ENV = 0
	sts	ENV_INTEGR2, r16	;/
.endif	; USE_INIT_LFO2
	sts	ADC_CHAN, r16		;ADC_CHAN = 0
	sts	NOTE_L, r16		;\
	sts	NOTE_H, r16		; >
	sts	NOTE_INTG, r16		;/
	sts	MIDIPBEND_L, r16	;\
	sts	MIDIPBEND_H, r16	;/ P.BEND = 0
	sts	MIDIMODWHEEL, r16	; MOD.WHEEL = 0
	sts	KNOB_SHIFT, r16		; Initialize panel shift switch = 0 (unshifted)
	sts	VCF_STATUS, r16		; Flag VCF as off (0)

	ldi	r16, 2
	sts	PORTACNT, r16		; PORTACNT = 2

	ldi	r16, 255
	sts	POWER_UP, r16		; Set power_up flag to 255 to force first initialization of panel switches
	sts	WRITE_MODE, r16		; Patch write mode defaults to "off"
	sts	LPF_I, r16		; no DCF
	sts	HPF_I, r16
	sts	MIDINOTE, r16		; note# = 255
	sts	MIDINOTEPREV, r16	; note# = 255

	ldi	r16, 0x5E		;\
	ldi	r17, 0xB4		; \
	ldi	r18, 0x76		;  \ initialising of
	sts	SHIFTREG_0, r16		;  / shift register
	sts	SHIFTREG_1, r17		; /
	sts	SHIFTREG_2, r18		;/

.if !USE_ORIGINAL_NOISEGEN
;	ldi	r16, 0x5E		;\
;	ldi	r17, 0xB4		; \
;	ldi	r18, 0x76		;  \ initialising of
	sts	LFSR_0, r16		;  / lienear feedback shift register
	sts	LFSR_1, r17		; /
	sts	LFSR_2, r18		;/
.endif	; !USE_ORIGINAL_NOISEGEN

	ldi	r16, 0			;\
	ldi	r17, 0			; > Amin = 0
	ldi	r18, 0			;/
	sts	LFOBOTTOM_0, r16	;\
	sts	LFOBOTTOM_1, r17	; > store Amin for LFO
	sts	LFOBOTTOM_2, r18	;/

.if USE_MIDI_PWMDEPTH
	sts	LFO2BOTTOM_0, r16	;\
	sts	LFO2BOTTOM_1, r17	; > store Amin for LFO2
	sts	LFO2BOTTOM_2, r18	;/
.endif	; USE_MIDI_PWMDEPTH

	ldi	r16, 255		;\
	ldi	r17, 255		; > Amax = 255,999
	ldi	r18, 255		;/
	sts	LFOTOP_0, r16		;\
	sts	LFOTOP_1, r17		; > store Amax for LFO
	sts	LFOTOP_2, r18		;/

.if !USE_MIDI_PWMDEPTH
	ldi	r18, 20
	sts	LFO2BOTTOM_0, r16	;\
	sts	LFO2BOTTOM_1, r17	; > store Amin for LFO2
	sts	LFO2BOTTOM_2, r18	;/

	ldi	r18, 100
.endif	; !USE_MIDI_PWMDEPTH
	sts	LFO2TOP_0, r16		;\
	sts	LFO2TOP_1, r17		; > store Amax for LFO2
	sts	LFO2TOP_2, r18		;/

;initialize sound parameters:
	ldi	r16, 0
	sts	LFOPHASE, r16		;
	sts	LFO2PHASE, r16		;
	sts	ENVPHASE, r16		;
.if USE_INIT_LFO2
	sts	ENVPHASE2, r16		;
.endif	; USE_INIT_LFO2

	sts	DETUNEB_FRAC, r16	;\
	sts	DETUNEB_INTG, r16	;/ detune = 0
	sts	LFOLEVEL, r16		;

.if !USE_ORIGINAL_MIXER
	ldi	r16, 127
	sts	DCOA_LEVEL, r16
	sts	DCOB_LEVEL, r16
.endif	; !USE_ORIGINAL_MIXER

.if USE_MASTER_VOLUME
	ldi	r16, $80
	sts	MASTER_VOLUME, r16
	ldi	r16, $40
	sts	VOLUME, r16
.endif	; USE_MASTER_VOLUME

.if USE_EXTENDED_MIDI_CONTROL
; defalut values for midi cc parameter
	ldi	r16, 4			;\ Set LFO to slow sweep for PWM modulation
	sts	LFO2FREQ, r16		;/

	ldi	r16, 128
	sts	PWMDEPTH, r16

	ldi	r16, 0
	sts	ATTACKTIME2, r16
	sts	RELEASETIME2, r16
	sts	ATTACKTIME, r16
	sts	RELEASETIME, r16

	ldi	r16, 128
	sts	DECAYTIME2, r16
	sts	SUSTAINLEVEL2, r16
	sts	SUSTAINLEVEL, r16
	sts	DECAYTIME, r16

	; SW_KNOB_SHIFT = upper
;	ldi	r16, (1<<SW_KNOB_SHIFT)
;	sts	PATCH_SWITCH1, r16
;
;	ldi	r16, 0x00
;	sts	PATCH_SWITCH2, r16

	; SW_DCA_MODE = env, SW_MODWHEEL_ENABLE = on
	; SW_LFO_KBD_SYNC = on, SW_DCF_KBD_TRACK = on
	ldi	r16, (1<<SW_DCA_MODE)     | (1<<SW_MODWHEEL_ENABLE) |\
		     (1<<SW_LFO_KBD_SYNC) | (1<<SW_DCF_KBD_TRACK)   |\
		     (1<<SW_TRANSPOSE)   | (0<<SW_MIX_RING)
	sts	PATCH_SWITCH3, r16
	sts	SW3, r16

	ldi	r16, 0x00
	sts	PATCH_SWITCH4, r16
	sts	SW4, r16
.endif	; USE_EXTENDED_MIDI_CONTROL


;initialize port A:
	ldi	r16, 0x00		;\
	out	PORTA, r16		;/ PA = zzzzzzzz
	ldi	r16, 0x00		;\
	out	DDRA, r16		;/ PA = iiiiiiii    all inputs (panel pots)

;initialize port B:
.if MEEBLIP_V1_27_HW
	ldi	r16, 0xFF		;\
	out	PORTB, r16		;/ PB = pppppppp
	ldi	r16, 0x00		;\
	out	DDRB, r16		;/ PB = iiiiiiii    all inputs
.else	; MEEBLIP_V1_27_HW
	ldi	r16, 0xFF		;\
	out	PORTB, r16		;/ PB = pppppppp
	ldi	r16, 0x40		;\ PB6 MISO = MIDI_LED
	out	DDRB, r16		;/ PB = ioiiiiii    all inputs except PB6 (MIDI-LED)
.endif	; MEEBLIP_V1_27_HW

;initialize port C:
	ldi	r16, 0x00		;\
	out	PORTC, r16		;/ PC = 00000000
	ldi	r16, 0xFF		;\
	out	DDRC, r16		;/ PC = oooooooo    all outputs (DAC)

;initialize port D:
.if MEEBLIP_V1_27_HW
	ldi	r16, 0xFC		;\
	out	PORTD, r16		;/ PD = 1111110z
	ldi	r16, 0xFE		;\
	out	DDRD, r16		;/ PD = oooooooi    all outputs except PD0 (MIDI-IN)
.else	; MEEBLIP_V1_27_HW
	ldi	r16, 0xFE		;\
	out	PORTD, r16		;/ PD = 1111111z
	ldi	r16, 0xFE		;\
	out	DDRD, r16		;/ PD = oooooooi    all outputs except PD0 (MIDI-IN)
.endif	; MEEBLIP_V1_27_HW

; Turn Power/MIDI LED on at power up

	sbi	PORT_MIDI_LED, MIDI_LED	; LED on

; initialize DAC port pins

	sbi	PORTD, DAC_WR		; Set WR high
	cbi	PORTD, DAC_AB		; Pull DAC AB port select low

;initialize Timer0:
	ldi	r16, 0x00		;\
	out	TCCR0, r16		;/ stop Timer 0

;initialize Timer1:
	ldi	r16, 0x04		;\ prescaler = CK/256
	out	TCCR1B, r16		;/ (clock = 32µs)

;initialize Timer2:
	ldi	r16, 54	;\
	out	OCR2, r16		;/ OCr2 = 54 gives 36363.63636 Hz sample rate at ~440 cycles per sample loop.
	ldi	r16, 0x0A		;\ clear timer on compare,
	out	TCCR2, r16		;/ set prescaler = CK/8

.if USE_UART
;initialize UART:
	call	usart_init_default_baudrate	; set baudrate to 38400
.else	; USE_UART
	ldi	r16, high((cpu_frequency / (baud_rate * 16)) - 1)
	out	UBRRH, r16
	ldi	r16, low((cpu_frequency / (baud_rate * 16)) - 1)
	out	UBRRL, r16

; enable receiver and receiver interrupt
	ldi	r16, (1<<RXCIE)|(1<<RXEN)	;\
	out	UCR, r16			;/ RXCIE=1, RXEN=1
.endif	; USE_UART

;initialize ADC:
	ldi	r16, 0x86		;\
	out	ADCSRA, r16		;/ ADEN=1, clk = 125 kHz

;initialize interrupts:
	ldi	r16, 0x80		;\
	out	TIMSK, r16		;/ OCIE2=1

	sei				; Interrupt Enable

;start conversion of the first A/D channel:
	lds	r18, ADC_CHAN
	rcall	ADC_START

;store initial pot positions as OLD_ADC values to avoid snapping to new value unless knob has been moved.

	; Store value of Pot ADC0
	ldi	r28, low(ADC_0)
	ldi	r29, high(ADC_0)
	add	r28, r18
	adc	r29, ZERO
	rcall	ADC_END			; r16 = AD(i)
	st	Y, r16			; AD(i) --> ADC_i

	inc	r18			; Now do ADC1
	rcall	ADC_START		; start conversion of next channel
	inc	r28
	rcall	ADC_END			; r16 = AD(i)
	st	Y, r16			; AD(i) --> ADC_i

	inc	r18			; Now do ADC2
	rcall	ADC_START		; start conversion of next channel
	inc	r28
	rcall	ADC_END			; r16 = AD(i)
	st	Y, r16			; AD(i) --> ADC_i

	inc	r18			; Now do ADC3
	rcall	ADC_START		; start conversion of next channel
	inc	r28
	rcall	ADC_END			; r16 = AD(i)
	st	Y, r16			; AD(i) --> ADC_i

	inc	r18			; Now do ADC4
	rcall	ADC_START		; start conversion of next channel
	inc	r28
	rcall	ADC_END			; r16 = AD(i)
	st	Y, r16			; AD(i) --> ADC_i

	inc	r18			; Now do ADC5
	rcall	ADC_START		; start conversion of next channel
	inc		r28
	rcall	ADC_END			; r16 = AD(i)
	st	Y, r16			; AD(i) --> ADC_i

	inc	r18			; Now do ADC6
	rcall	ADC_START		; start conversion of next channel
	inc	r28
	rcall	ADC_END			; r16 = AD(i)
	st	Y, r16			; AD(i) --> ADC_i

	inc	r18			; Now do ADC7
	rcall	ADC_START		; start conversion of next channel
	inc	r28
	rcall	ADC_END			; r16 = AD(i)
	st	Y, r16			; AD(i) --> ADC_i

	ldi	r18, 0
	sts	ADC_CHAN,r18
	rcall	ADC_START		; start conversion of ADC0 for main loop

; Load current patch from eeprom

	ldi	r18, $03
	ldi	r17, $FE		; Set eeprom address to $03FE (second last byte)
	rcall	EEPROM_READ		; load current patch number into r16
	rcall	LOAD_PATCH		; load patch into synthesis engine

; Load MIDI channel from eeprom

	ldi	r18, $03
	ldi	r17, $FF		; Set eeprom address to $03FF (last byte)
	rcall	EEPROM_READ		; load MIDI CHANNEL into r16
	sts	SETMIDICHANNEL, r16

;initialize the keyboard scan time
	in	r16, TCNT1L		;\
	in	r17, TCNT1H		;/ r17:r16 = TCNT1 = t
	sts	TPREV_KBD_L, r16
	sts	TPREV_KBD_H, r17

.if USE_UART
	call	usart_startup_message

; print knob parameter values from eeprom to serial port
	call	usart_print_sound_parameter

; print switch/modeflags status to serial port

; reinitialize UART for midi
init_uart_midi:
	sbis	UCSRA, UDRE		; before change the baudrate wait for transmitter ready
	rjmp	init_uart_midi

	ldi	r16, high((cpu_frequency / (baud_rate * 16)) - 1)
	out	UBRRH, r16
	ldi	r16, low((cpu_frequency / (baud_rate * 16)) - 1)
	out	UBRRL, r16

; enable receiver and receiver interrupt
	ldi	r16, (1<<RXCIE)|(1<<RXEN);\
	out	UCR, r16		;/ RXCIE=1, RXEN=1
.endif	; USE_UART

;-----------------------------------------------------------------------------
; Main Program Loop
;
; This is where everything but sound generation happens. This loop is interrupted 36,636 times per second by the
; sample interrupt routine. When it's actually allowed to get down to work, it scans the panel switches every 100ms,
; scans the knobs a lot more than that, calculates envelopes, processes the LFO and parses MIDI input.
;
; In its spare time, Main Program Loop likes to go for long walks, listen to classical music and perform
; existential bit flipping.
;-----------------------------------------------------------------------------
;


MAINLOOP:
	;---------------------
	; scan panel switches:
	;---------------------
;begin:

	in	r16, TCNT1L		;\
	in	r17, TCNT1H		;/ r17:r16 = t
	lds	r18, TPREV_KBD_L	;\
	lds	r19, TPREV_KBD_H	;/ r19:r18 = t0
	sub	r16, r18		;\
	sbc	r17, r19		;/ r17:r16 = t - t0
	subi	r16, LOW(KBDSCAN)	;\
	sbci	r17, HIGH(KBDSCAN)	;/ r17:r16 = (t-t0) - 100ms
	brsh	MLP_SCAN		;\
.if USE_ORIGINAL_ENVELOPE_CONTROL
	rjmp	MLP_WRITE		;/ skip scanning if (t-t0) < 100ms
.else	; USE_ORIGINAL_ENVELOPE_CONTROL
	rjmp	MLP_UPDATE		;/ skip scanning if (t-t0) < 100ms
.endif	; USE_ORIGINAL_ENVELOPE_CONTROL

MLP_SCAN:

	; blink LED if LED/save/load button has been pressed
	lds	r16, SWITCH3
	tst	r16
	breq	CONTROL_EXIT		; Set button status: 1=MIDI, 2=SAVE, 4=LOAD
.if USE_EXTENDED_BUTTON_FUNCTIONS
	lds	r17, BUTTON_STATUS
	tst	r17
	breq	MLP_ONEBTN
	cp	r16, r17
	breq	MLP_ONEBTN
	ori	r16, 0x80		; set bit 7 to indicate we have more then one button pressed
	or	r16, r17
MLP_ONEBTN:
.endif	; USE_EXTENDED_BUTTON_FUNCTIONS
	sts	BUTTON_STATUS, r16	; Set control status to MIDI, turn on control timer
	ldi	r16, 63
	sts	LED_TIMER, r16
CONTROL_EXIT:
	lds	r16, BUTTON_STATUS
	tst	r16
	breq	MLP_LED_END		; button hasn't been pressed, so skip

	; flip panel LED state until button timer is finished
.if !USE_EXTENDED_BUTTON_FUNCTIONS
	lds	r16, LED_STATUS
	ldi	r17, 64
.else	; !USE_EXTENDED_BUTTON_FUNCTIONS
	ldi	r17, 32
	andi	r16, 0x80
	breq	MLP_SLOW_LED
	ldi	r17, 128
MLP_SLOW_LED:
	lds	r16, LED_STATUS
.endif	; !USE_EXTENDED_BUTTON_FUNCTIONS
	add	r16, r17
	cpi	r16, $80
	brlo	MLP_LED_OFF
	lds	r17, LED_TIMER
	dec	r17
	breq	MLP_LED_RESET
	sbi	PORT_MIDI_LED, MIDI_LED	; LED on
	rjmp	MLP_LED_EXIT
MLP_LED_OFF:
	lds	r17, LED_TIMER
	dec	r17
	breq	MLP_LED_RESET
	cbi	PORT_MIDI_LED, MIDI_LED	; LED off
	rjmp	MLP_LED_EXIT
MLP_LED_RESET:
	sbi	PORT_MIDI_LED, MIDI_LED	; Clear everything
	ldi	r16, 0
	sts	BUTTON_STATUS, r16
MLP_LED_EXIT:
	sts	LED_STATUS, r16
	sts	LED_TIMER, r17
MLP_LED_END:

	in	r16, TCNT1L
	in	r17, TCNT1H
	sts	TPREV_KBD_L, r16	;\
	sts	TPREV_KBD_H, r17	;/ t0 = t

;reading:
	ldi	r16, 0x10		; inverted state of PD outputs
	ldi	r17, 0x01		; mask
	ldi	r18, 0x10		; mask
	ldi	r19, 0x00		; bits of SWITCH1
	ldi	r20, 0x00		; bits of SWITCH2
	ldi	r21, 0x00		; bits of SWITCH3

MLP_SWLOOP:
	in	r30, PORTD
	ori	r30, 0xF0
	eor	r30, r16
	out	PORTD, r30		; `set' keyboard ROW to scan
	rcall	WAIT_10US
	in	r30, PINB		; `read' keyboard COL for key status

.if MEEBLIP_V1_27_HW
; original code for mmeblip hardware
	sbrs	r30, 0			;\
	or	r19, r17		;/ set bit when PB0==0
	sbrs	r30, 1			;\
	or	r19, r18		;/ set bit when PB1==0
	sbrs	r30, 2			;\
	or	r20, r17		;/ set bit when PB2==0
	sbrs	r30, 3			;\
	or	r20, r18		;/ set bit when PB3==0
.else	; MEEBLIP_V1_27_HW
; On my hardware, the switches are mounted vice versa.
	sbrc	r30, 0			;\
	or	r19, r17		;/ set bit when PB0==0
	sbrc	r30, 1			;\
	or	r19, r18		;/ set bit when PB1==0
	sbrc	r30, 2			;\
	or	r20, r17		;/ set bit when PB2==0
	sbrc	r30, 3			;\
	or	r20, r18		;/ set bit when PB3==0
.endif	; MEEBLIP_V1_27_HW
	sbrs	r30, 4			;\
	or	r21, r17		;/ set bit when PB4==0
	lsl	r17
	lsl	r18
	lsl	r16
	brne	MLP_SWLOOP
	in	r16, PORTD
	ori	r16, 0xF0		; OR 1111 0000
	out	PORTD, r16		; just resets the ROW selector bits
	sts	SWITCH1, r19
	sts	SWITCH2, r20
	sts	SWITCH3, r21		; V04

; capture switch values on power up

	lds	r17, POWER_UP		; Is this the first time through this code since synth was turned on?
	sbrs	r17, 0
	rjmp	MLP_SWITCH
	sts	OLD_SWITCH1, r19	; Yes: make OLD_SWITCHx = SWITCHx (so we know when they've been flipped)
	sts	OLD_SWITCH2, r20
	clr	r17
	sts	POWER_UP, r17		; Clear the POWER_UP flag so we don't reinitialize

; -------------
;service:
.if USE_ORIGINAL_SWITCH_MAP | !USE_MORE_SWITCHES
MLP_SWITCH:
	; Compare SWITCH1 to OLD_SWITCH1. Skip if unchanged.

	lds	r16, SWITCH1
	lds	r17, OLD_SWITCH1
	cp	r16, r17
	breq	MLP_SWITCH2		; Switch 1 unchanged, so leave as is.
	lds	r18, PATCH_SWITCH1

	; Compare bits in OLD_SWITCH1 and SWITCH1. If different, copy SWITCH1 bit to PATCH_SWITCH1 bit.
	clr	r30			; Register used to indicate which switch 1..16 has been flipped
	; Perform an exclusive OR on r17 and r16. Changed bits are flagged as 1's.
	eor	r17, r16
	sbrs	r17, 0
	rjmp	MLP_BIT1		; Exit if bit is not set
	bst	r16, 0
	bld	r18, SW_KNOB_SHIFT	; copy bit from SWITCH1 to PATCH_SWITCH1
	; Flag dual parameter knobs unmoved because this is the KNOB_SHIFT switch
	push	r16
	rcall	CLEAR_KNOB_STATUS
	pop	r16
	ldi	r30, 16			; Flag switch 16 moved
MLP_BIT1:
	sbrs	r17, 1
	rjmp	MLP_BIT2
	bst	r16, 1
	bld	r18, SW_OSC_FM
	ldi	r30, 15			; Flag switch 15 moved
MLP_BIT2:
	sbrs	r17, 2
	rjmp	MLP_BIT3
	bst	r16, 2
	bld	r18, SW_LFO_RANDOM
	ldi	r30, 14			; Flag switch 14 moved
MLP_BIT3:
	sbrs	r17, 3
	rjmp	MLP_BIT4
	bst	r16, 3
	bld	r18, SW_LFO_WAVE
	ldi	r30, 13			; Flag switch 13 moved
MLP_BIT4:
	sbrs	r17, 4
	rjmp	MLP_BIT5
	bst	r16, 4
	bld	r18, SW_FILTER_MODE
	ldi	r30, 8			; Flag switch 8 moved
MLP_BIT5:
	sbrs	r17, 5
	rjmp	MLP_BIT6
	bst	r16, 5
	bld	r18, SW_DISTORTION
	ldi	r30, 7			; Flag switch 7 moved
MLP_BIT6:
	sbrs	r17, 6
	rjmp	MLP_BIT7
	bst	r16, 6
	bld	r18, SW_LFO_ENABLE
	ldi	r30, 6			; Flag switch 6 moved
MLP_BIT7:
	sbrs	r17, 7
	rjmp	MLP_PATCH1SAVE
	bst	r16, 7
	bld	r18, SW_LFO_DEST
	ldi	r30, 5			; Flag switch 5 moved

MLP_PATCH1SAVE:
	sts	CONTROL_SWITCH, r30	; Number of last switch flipped: 1..16, where zero indicates none flipped

	; DON'T save changes to PATCH_SWITCH1 if BUTTON_STATUS is SAVE (2) because
	;   we're about to write the patch that has just been changed by flipping the switch.
	lds	r17, BUTTON_STATUS
	cpi	r17, (1<<SAVE_BUTTON)
	breq	MLP_SKIP_PATCH1
	sts	PATCH_SWITCH1, r18	; Store changes	to patch
MLP_SKIP_PATCH1:
	sts	OLD_SWITCH1, r16	; Keep a copy of panel switches so we know if things change next time

MLP_SWITCH2:
	; Compare SWITCH2 to OLD_SWITCH2. Skip if unchanged.

	lds	r16, SWITCH2
	lds	r17, OLD_SWITCH2
	cp	r16, r17
	breq	MLP_SW_EXIT		; Switch 2 unchanged, so leave as is.
	lds	r18, PATCH_SWITCH2

	; Compare bits in OLD_SWITCH2 and SWITCH2. If different,
	; copy SWITCH2 bit to PATCH_SWITCH2 bit.
	clr	r30			; Register used to indicate which switch 1..16 has been flipped

	; Perform an exclusive OR on r17 and r16. Changed bits are flagged as 1's.
	eor	r17, r16
	sbrs	r17, 0
	rjmp	MLP_BIT1A		; Exit if bit is not set
	bst	r16, 0			; copy bit from SWITCH2 to PATCH_SWITCH2
	bld	r18, SW_ANTI_ALIAS
	ldi	r30, 12			; Flag switch 12 moved
MLP_BIT1A:
	sbrs	r17, 1
	rjmp	MLP_BIT2A
	bst	r16, 1
	bld	r18, SW_OSCB_OCT
	ldi	r30, 11			; Flag switch 11 moved
MLP_BIT2A:
	sbrs	r17, 2
	rjmp	MLP_BIT3A
	bst	r16, 2
	bld	r18, SW_OSCB_ENABLE
	ldi	r30, 10			; Flag switch 10 moved
MLP_BIT3A:
	sbrs	r17, 3
	rjmp	MLP_BIT4A
	bst	r16, 3
	bld	r18, SW_OSCB_WAVE
	ldi	r30, 9			; Flag switch 9 moved
MLP_BIT4A:
	sbrs	r17, 4
	rjmp	MLP_BIT5A
	bst	r16, 4
	bld	r18, SW_SUSTAIN
	ldi	r30, 4			; Flag switch 4 moved
.if !USE_ORIGINAL_ENVELOPE_CONTROL
; set update_env_dca & update_env_dcf flags
	lds	r22, MIDI_TASK_UPDATE
	ori	r22, (1<<UPDATE_ENV_DCF) | (1<< UPDATE_ENV_DCA)
	sts	MIDI_TASK_UPDATE, r22
.endif	; !USE_ORIGINAL_ENVELOPE_CONTROL

MLP_BIT5A:
	sbrs	r17, 5
	rjmp	MLP_BIT6A
	bst	r16, 5
	bld	r18, SW_OSCA_NOISE
	ldi	r30, 3			; Flag switch 3 moved
MLP_BIT6A:
	sbrs	r17, 6
	rjmp	MLP_BIT7A
	bst	r16, 6
	bld	r18, SW_PWM_SWEEP
	ldi	r30, 2			; Flag switch 2 moved
MLP_BIT7A:
	sbrs	r17, 7
	rjmp	MLP_PATCH2SAVE
	bst	r16, 7
	bld	r18, SW_OSCA_WAVE
	ldi	r30, 1			; Flag switch 1 moved

MLP_PATCH2SAVE:
	sts	CONTROL_SWITCH, r30	; Number of last switch flipped: 1..16, where zero indicates none flipped

	; DON'T save changes to PATCH_SWITCH2 if BUTTON_STATUS is SAVE (2) because
	;   we're about to write the patch that has just been changed by flipping the switch.
	lds	r17, BUTTON_STATUS
	cpi	r17, (1<<SAVE_BUTTON)
	breq	MLP_SKIP_PATCH2
	sts	PATCH_SWITCH2, r18	; Store changes	to patch
MLP_SKIP_PATCH2:
	sts	OLD_SWITCH2, r16	; Keep a copy of panel switches so we know if things change next time

MLP_SW_EXIT:

.else	; USE_ORIGINAL_SWITCH_MAP
; -------------
MLP_SWITCH:
	; Compare SWITCH1 to OLD_SWITCH1. Skip if unchanged.

	.def	REG_PATCH_SWITCH1 = r18
	.def	REG_PATCH_SWITCH2 = r19
	.def	REG_PATCH_SWITCH3 = r20
	.def	REG_PATCH_SWITCH4 = r21

	; Keep a copy so we know if things change next time
	lds	r18, PATCH_SWITCH1
	lds	r19, PATCH_SWITCH2
	lds	r20, PATCH_SWITCH3
	lds	r21, PATCH_SWITCH4

	; Compare SWITCH1 to OLD_SWITCH1. Skip if unchanged.
	lds	r16, SWITCH1
	lds	r17, OLD_SWITCH1
	cp	r16, r17
	breq	MLP_SWITCH2		; Switch 1 unchanged, so leave as is.
	sts	OLD_SWITCH1, r16	; Keep a copy of panel switches so we know if things change next time

; Compare bits in OLD_SWITCH1 and SWITCH1. If different, copy SWITCH1 bits to PATCH_SWITCHx bits.
	clr	r30			; Register used to indicate which switch 1..16 has been flipped
	; Perform an exclusive OR on r17 and r16. Changed bits are flagged as 1's.
	eor	r17, r16
MLP_BIT0:
	sbrs	r17, 0
	rjmp	MLP_BIT1		; Exit if bit is not set
	bst	r16, 0			; copy bit from SWITCH1 to PATCH_SWITCH1
	bld	REG_PATCH_SWITCH1, SW_KNOB_SHIFT
	; Flag dual parameter knobs unmoved because this is the KNOB_SHIFT switch
	push	r16
	rcall	CLEAR_KNOB_STATUS
	pop	r16
	ldi	r30, 16			; Flag switch 16 moved
MLP_BIT1:
	sbrs	r17, 1
	rjmp	MLP_BIT2
	bst	r16, 1
.if USE_ORIGINAL_DCO_FM
	bld	REG_PATCH_SWITCH1, SW_OSC_FM
.else	; USE_ORIGINAL_DCO_FM
	bld	REG_PATCH_SWITCH3, SW_TRANSPOSE
.endif	; USE_ORIGINAL_DCO_FM
	ldi	r30, 15			; Flag switch 15 moved
MLP_BIT2:
	sbrs	r17, 2
	rjmp	MLP_BIT3
	bst	r16, 2
	bld	REG_PATCH_SWITCH1, SW_LFO_RANDOM
	ldi	r30, 14			; Flag switch 14 moved
MLP_BIT3:
	sbrs	r17, 3
	rjmp	MLP_BIT4
	bst	r16, 3
	bld	REG_PATCH_SWITCH1, SW_LFO_WAVE
	ldi	r30, 13			; Flag switch 13 moved
MLP_BIT4:
	sbrs	r17, 4
	rjmp	MLP_BIT5
	bst	r16, 4
	bld	REG_PATCH_SWITCH1, SW_FILTER_MODE
	ldi	r30, 8			; Flag switch 8 moved
MLP_BIT5:
	sbrs	r17, 5
	rjmp	MLP_BIT6
	bst	r16, 5
	bld	REG_PATCH_SWITCH1, SW_DISTORTION
	ldi	r30, 7			; Flag switch 7 moved
MLP_BIT6:
	sbrs	r17, 6
	rjmp	MLP_BIT7
	bst	r16, 6
	bld	REG_PATCH_SWITCH1, SW_LFO_ENABLE
	ldi	r30, 6			; Flag switch 6 moved
MLP_BIT7:
	sbrs	r17, 7
	rjmp	MLP_PATCH1SAVE
	bst	r16, 7
	bld	REG_PATCH_SWITCH1, SW_LFO_DEST
	ldi	r30, 5			; Flag switch 5 moved

MLP_PATCH1SAVE:
	sts	CONTROL_SWITCH, r30	; Number of last switch flipped: 1..16, where zero indicates none flipped

MLP_SWITCH2:
; Compare SWITCH2 to OLD_SWITCH2. Skip if unchanged.

	lds	r16, SWITCH2
	lds	r17, OLD_SWITCH2
	cp	r16, r17
	breq	MLP_SW_LEAVE		; Switch 2 unchanged, so leave as is.
	sts	OLD_SWITCH2, r16	; Keep a copy of panel switches so we know if things change next time

	; Compare bits in OLD_SWITCH2 and SWITCH2. If different,
	; copy SWITCH2 bit to PATCH_SWITCH2 bit.
	clr	r30			; Register used to indicate which switch 1..16 has been flipped

	; Perform an exclusive OR on r17 and r16. Changed bits are flagged as 1's.
	eor	r17, r16
MLP_BIT0A:
	sbrs	r17, 0
	rjmp	MLP_BIT1A		; Exit if bit is not set
	bst	r16, 0			; copy bit from SWITCH2 to PATCH_SWITCHx
.if USE_ORIGINAL_RAW_OSC
	bld	REG_PATCH_SWITCH2, SW_ANTI_ALIAS
.else	; USE_ORIGINAL_RAW_OSC
	bld	REG_PATCH_SWITCH3, SW_MIX_RING
.endif	; USE_ORIGINAL_RAW_OSC
	ldi	r30, 12			; Flag switch 12 moved
MLP_BIT1A:
	sbrs	r17, 1
	rjmp	MLP_BIT2A
	bst	r16, 1
	bld	REG_PATCH_SWITCH2, SW_OSCB_OCT
	ldi	r30, 11			; Flag switch 11 moved
MLP_BIT2A:
	sbrs	r17, 2
	rjmp	MLP_BIT3A
	bst	r16, 2
	bld	REG_PATCH_SWITCH2, SW_OSCB_ENABLE
	ldi	r30, 10			; Flag switch 10 moved
MLP_BIT3A:
	sbrs	r17, 3
	rjmp	MLP_BIT4A
	bst	r16, 3
	bld	REG_PATCH_SWITCH2, SW_OSCB_WAVE
	ldi	r30, 9			; Flag switch 9 moved
MLP_BIT4A:
	sbrs	r17, 4
	rjmp	MLP_BIT5A
	bst	r16, 4
	bld	REG_PATCH_SWITCH2, SW_SUSTAIN
	ldi	r30, 4			; Flag switch 4 moved
.if !USE_ORIGINAL_ENVELOPE_CONTROL
; set update_env_dca & update_env_dcf flags
	lds	r22, MIDI_TASK_UPDATE
	ori	r22, (1<<UPDATE_ENV_DCF) | (1<< UPDATE_ENV_DCA)
	sts	MIDI_TASK_UPDATE, r22
.endif	; !USE_ORIGINAL_ENVELOPE_CONTROL

MLP_BIT5A:
	sbrs	r17, 5
	rjmp	MLP_BIT6A
	bst	r16, 5
	bld	REG_PATCH_SWITCH2, SW_OSCA_NOISE
	ldi	r30, 3			; Flag switch 3 moved
MLP_BIT6A:
	sbrs	r17, 6
	rjmp	MLP_BIT7A
	bst	r16, 6
	bld	REG_PATCH_SWITCH2, SW_PWM_SWEEP
	ldi	r30, 2			; Flag switch 2 moved
MLP_BIT7A:
	sbrs	r17, 7
	rjmp	MLP_PATCH2SAVE
	bst	r16, 7
	bld	REG_PATCH_SWITCH2, SW_OSCA_WAVE
	ldi	r30, 1			; Flag switch 1 moved

MLP_PATCH2SAVE:
	sts	CONTROL_SWITCH, r30	; Number of last switch flipped: 1..16, where zero indicates none flipped

MLP_SW_LEAVE:
	; DON'T save changes to PATCH_SWITCH2 if BUTTON_STATUS is SAVE (2) because
	;   we're about to write the patch that has just been changed by flipping the switch.
	lds	r17, BUTTON_STATUS
	cpi	r17, (1<<SAVE_BUTTON)
	breq	MLP_SW_EXIT

	; Store changes	to patch
	sts	PATCH_SWITCH1, REG_PATCH_SWITCH1	; Store changes
	sts	PATCH_SWITCH2, REG_PATCH_SWITCH2	; Store changes
	sts	PATCH_SWITCH3, REG_PATCH_SWITCH3	; Store changes
	sts	PATCH_SWITCH4, REG_PATCH_SWITCH4	; Store changes
MLP_SW_EXIT:
	.undef	REG_PATCH_SWITCH1
	.undef	REG_PATCH_SWITCH2
	.undef	REG_PATCH_SWITCH3
	.undef	REG_PATCH_SWITCH4
.endif	; USE_ORIGINAL_SWITCH_MAP
; -------------

; Set MIDI channel based on panel switches:
	lds	r17, BUTTON_STATUS
	tst	r17
	breq	SKIP_CONTROL_SET
	lds	r16, CONTROL_SWITCH	; number of switch which has changed 1..16, 0 no switch changed
	tst	r16
	breq	SKIP_CONTROL_SET	; continue only if a switch was moved
	cpi	r17, (1<<MIDI_BUTTON)
	breq	SET_MIDI_CH
	cpi	r17, (1<<SAVE_BUTTON)
	breq	MLP_SAVE

.if USE_EXTENDED_BUTTON_FUNCTIONS
	andi	r17, 0x7f		; mask out flag for more then one button pressed
	cpi	r17, (1<<SAVE_BUTTON) | (1<<MIDI_BUTTON)	; extended function set 1
	breq	MLP_EXTFUNC_11
	cpi	r17, (1<<LOAD_BUTTON) | (1<<MIDI_BUTTON)	; extended function set 2
	breq	MLP_EXTFUNC_21
	cpi	r17, (1<<LOAD_BUTTON) | (1<<SAVE_BUTTON)	; extended function set 3
	breq	MLP_EXTFUNC_31
	cpi	r17, (1<<LOAD_BUTTON)
	brne	SKIP_CONTROL_SET
.endif	; USE_EXTENDED_BUTTON_FUNCTIONS

MLP_LOAD:
	dec	r16			; Shift patch offset to 0..15
	ldi	r18, $03
	ldi	r17, $FE		; Set eeprom address to $03FE (second last byte in memory)
	rcall	EEPROM_WRITE		; Write current patch # to memory so it will be loaded at power up
	rcall	LOAD_PATCH		; Fetches the patch correspondng to the control switch number stored in r16
	rjmp	END_CONTROL_SET		; reset control button

.if USE_READ_ONLY_PATCH_ZERO | USE_EXTENDED_BUTTON_FUNCTIONS
SKIP_CONTROL_SET:				; ! clean up
	rjmp	SKIP_CONTROL_SET_1
.endif	; USE_READ_ONLY_PATCH_ZERO | USE_EXTENDED_BUTTON_FUNCTIONS

.if USE_EXTENDED_BUTTON_FUNCTIONS
SET_MIDI_CH:
	rjmp	SET_MIDI_CH_1
MLP_EXTFUNC_11:
	rjmp	MLP_EXTFUNC_1
MLP_EXTFUNC_21:
	rjmp	MLP_EXTFUNC_2
MLP_EXTFUNC_31:
	rjmp	MLP_EXTFUNC_3
.endif	; USE_EXTENDED_BUTTON_FUNCTIONS

MLP_SAVE:
.if USE_READ_ONLY_PATCH_ZERO
	cpi	r16, 1			; set patch zero to read only, prohibit it from overwriting
	brne	MLP_SAVE_1
	ldi	r17, 255
	sts	WRITE_MODE, r17		; Set write mode to 255 (off)
	rjmp	END_CONTROL_SET
MLP_SAVE_1:
.endif	; USE_READ_ONLY_PATCH_ZERO
	lds	r17, WRITE_MODE
	tst	r17
	breq	END_CONTROL_SET		; Skip if we're already in write mode
	dec	r16			; Shift patch offset to 0..15
	ldi	r18, $03
	ldi	r17, $FE		; Set eeprom address to $03FE (second last byte in memory)
	rcall	EEPROM_WRITE		; Write current patch # to memory so it will be loaded at power up

	ldi	r17, 0
	sts	WRITE_MODE, r17		; Set knob status to 'unmoved' and save current knob positions
	sts	WRITE_OFFSET, r17	; start address in eeprom for current patch

; compute patch base address
	lsl	r16
	lsl	r16
	lsl	r16
	lsl	r16			; multiply patch number by 16 to get starting address of patch in eeprom
.if !USE_EXTENDED_PATCH
	sts	WRITE_PATCH_OFFSET, r16	; switch # 0..15/47
.else	; !USE_EXTENDED_PATCH
; compute 16 bit patch base address (16 patch sets each with 48 bytes)
	mov	r17, r16		; 3 * patch number -> load 48 byte
	ldi	r18, 0			;  (r18:r17)*2+(0:r16)
	lsl	r17
	rol	r18
	add	r17, r16		; Low byte of eeprom address
	adc	r18, ZERO		; High byte of eeprom address
	sts	WRITE_PATCH_OFFSET_L, r17
	sts	WRITE_PATCH_OFFSET_H, r18
.endif	; !USE_EXTENDED_PATCH
	lds	r16, PATCH_SWITCH1
	sts	SW1, r16
	lds	r16, PATCH_SWITCH2
	sts	SW2, r16		; Copy the patch switches into the MIDI CC table
.if USE_EXTENDED_MIDI_CONTROL
	; copy also other switches
	lds	r16, PATCH_SWITCH3
	sts	SW3, r16
	lds	r16, PATCH_SWITCH4
	sts	SW4, r16		; Copy the patch switches into the MIDI CC table
.endif	; USE_EXTENDED_MIDI_CONTROL
	rcall	CLEAR_KNOB_STATUS
	rjmp	END_CONTROL_SET		; reset control button

.if !USE_EXTENDED_BUTTON_FUNCTIONS
SET_MIDI_CH:
.else	; !USE_EXTENDED_BUTTON_FUNCTIONS
SET_MIDI_CH_1:
.endif	; !USE_EXTENDED_BUTTON_FUNCTIONS
					; Set MIDI channel - adjust the panel switches so that channel 16 sets the synth in omni mode
	inc	r16			; Shift value to 2..17 to make room for omni
	cpi	r16, 17
	brne	NOT_OMNI
	ldi	r16, 1			; If it was channel 16, force it to channel 1
NOT_OMNI:
	dec	r16			; subtract 1 from channel numbers
	sts	SETMIDICHANNEL, r16	; 0 for OMNI or channel 1..15
	ldi	r18, $03
	ldi	r17, $FF		; Set eeprom address to $03FF (last byte in memory)
	rcall	EEPROM_WRITE		; Write midi channel# (r16) to eeprom
.if USE_EXTENDED_BUTTON_FUNCTIONS
	rjmp	END_CONTROL_SET
MLP_EXTFUNC_1:
	cpi	r16, 1
	brne	MLP_EXTFUNC_1_0
.if USE_UART
	call	debug_entry
	call	usart_print_sound_parameter
	call	debug_exit
.endif	; USE_UART
MLP_EXTFUNC_1_0:

MLP_EXTFUNC_2:	; more extended function not implemented
MLP_EXTFUNC_3:

.endif	; USE_EXTENDED_BUTTON_FUNCTIONS
END_CONTROL_SET:
	sbi	PORT_MIDI_LED, MIDI_LED; Clear control button parameters and leave the LED on
	ldi	r16, 0			;
	sts	BUTTON_STATUS, r16	;
	sts	LED_STATUS, r16		;
	sts	LED_TIMER, r16		;
	sts	CONTROL_SWITCH, r16	;

.if !USE_EXTENDED_BUTTON_FUNCTIONS & !USE_READ_ONLY_PATCH_ZERO
SKIP_CONTROL_SET:
.else	; !USE_EXTENDED_BUTTON_FUNCTIONS & !USE_READ_ONLY_PATCH_ZERO
SKIP_CONTROL_SET_1:
.endif	; !USE_EXTENDED_BUTTON_FUNCTIONS & !USE_READ_ONLY_PATCH_ZERO

;-----------------------------------------------------------------------------
; do some update asks on changed switch flags
;-----------------------------------------------------------------------------

.if !USE_ORIGINAL_ENVELOPE_CONTROL | USE_MASTER_VOLUME
MLP_UPDATE:
	lds	r22, MIDI_TASK_UPDATE	; are there any open update tasks?
	tst	r22
	breq	MLP_UPDATE_DONE

.if !USE_ORIGINAL_ENVELOPE_CONTROL
; check update_env_dcf flag
	bst	r22, UPDATE_ENV_DCF
	brtc	MLP_UPD_DCF_SU_END

	lds	r16, KNOB_DCF_DECAY	; Grab knob patch value just in case it hasn't been changed
	lds	r19, PATCH_SWITCH2
	sbrs	r19, SW_SUSTAIN
	rjmp	MLP_UPD_DCF_SU_OFF	; skip if sustain switch is off
	ldi	r19, 255		; Sustain is on, so...
	sts	SUSTAINLEVEL2, r19	; Set sustain to maximum
	sts	DECAYTIME2, r19		; Set decay to maximum
	sts	RELEASETIME2, r16	; Set release time to value of decay knob
	rjmp	MLP_UPD_DCF_SU_DONE

MLP_UPD_DCF_SU_OFF:
	ldi	r19, 0			; Sustain is off, so...
	sts	SUSTAINLEVEL2, r19	; Set sustain to minimum
	sts	DECAYTIME2, r16		; Set decay time to value of decay knob
	sts	RELEASETIME2, r16	; Set release time to value of decay knob
MLP_UPD_DCF_SU_DONE:
; clear update_env_dca flag
	andi	r22, ~(1<<UPDATE_ENV_DCF)
MLP_UPD_DCF_SU_END:

; check update_env_dca flag
	bst	r22, UPDATE_ENV_DCA
	brtc	MLP_UPD_DCA_SU_END

	lds	r16, KNOB_AMP_DECAY	; Grab knob patch value just in case it hasn't been changed
	lds	r19, PATCH_SWITCH2
	sbrs	r19, SW_SUSTAIN
	rjmp	MLP_UPD_DCA_SU_OFF	; Skip if sustain is off...
	ldi	r19, 255		; Sustain is on, so...
	sts	SUSTAINLEVEL, r19	; Set sustain to maximum
	sts	DECAYTIME, r19		; Set decay to maximum
	sts	RELEASETIME, r16	; Set release time to value of decay knob
	rjmp	MLP_UPD_DCA_SU_DONE
MLP_UPD_DCA_SU_OFF:
	ldi	r19, 0			; Sustain is off, so...
	sts	SUSTAINLEVEL, r19	; Set sustain to minimum
	sts	DECAYTIME, r16		; Set decay time to value of decay knob
	sts	RELEASETIME, r16	; Set release time to value of decay knob
MLP_UPD_DCA_SU_DONE:
; clear update_env_dca flag
	andi	r22, ~(1<<UPDATE_ENV_DCA)
MLP_UPD_DCA_SU_END:
.endif	; USE_ORIGINAL_ENVELOPE_CONTROL

.if USE_MASTER_VOLUME
	bst	r22, UPDATE_VOLUME
	brtc	MLP_UPD_VOLUME_END

	lds	r16, MASTER_VOLUME
	ldi	r30, low( TAB_VOLUME)	;\
	ldi	r31, high( TAB_VOLUME)	;/ Z = &Tab
	rcall	TAB_BYTE		; r0 = 2..255
	sts	VOLUME, r16

; clear update_volume flag
	andi	r22, ~(1<<UPDATE_VOLUME)
MLP_UPD_VOLUME_END:
.endif	; USE_MASTER_VOLUME

	sts	MIDI_TASK_UPDATE, r22
MLP_UPDATE_DONE:
.endif	; !USE_ORIGINAL_ENVELOPE_CONTROL | USE_MASTER_VOLUME


;-----------------------------------------------------------------------------
; Asynchronous EEPROM write
;
; Because EEPROM writes are slow, MeeBlip executes the main program and audio
; interrupts while eeprom writes happen in the ; background. A new byte is only
; written if the eeprom hardware flags that it's finished the previous write.
;-----------------------------------------------------------------------------
;

MLP_WRITE:
	lds	r16, WRITE_MODE
	sbrc	r16,7
	rjmp	MLP_SKIPSCAN		; Nothing to write, so skip

	sbic	EECR,EEWE
	rjmp	MLP_SKIPSCAN		; Skip if we're not finished the last write

	; Get the parameter value from MIDI CC table
; r17: midi cc offset
; r18: eeprom offset
	lds	r18, WRITE_OFFSET	; r18 contains the byte offset in the patch

; map           eeprom address ->  midi cc address
; meeblip midi cc	 0..0F -> 30..3F
; my midi cc extentions	10..1F -> 0E..1D
; my switches SW1..SW4	20..23 -> 2C..2F

	; setup start address of midi cc parameter
	ldi	r17, $30		; first block	 0..0F -> 30..3F
.if USE_EXTENDED_PATCH
	cpi 	r18, $10
	brlo	MLP_WR_1
	ldi	r17, $0E - $10		; second block	10..1F -> 0E..1D
	cpi 	r18, $20
	brlo	MLP_WR_1
	ldi	r17, $2C - $20		; third block	20..23 -> 2C..2F
MLP_WR_1:
.endif	; USE_EXTENDED_PATCH
	; calculate the address to the midi cc parameter
	add	r17, r18		; Add the byte offset in the MIDI CC table

; r17: midi cc offset  <= r18 + (midi base addr + addr modifier)
; r18: eeprom offset   <= r18 + patch base addr
;
	ldi	r28, low (MIDICC)
	ldi	r29, high(MIDICC)
	add	r28, r17
	adc	r29, zero
	ld	r17, Y			; Patch parameter(i) stored in r17

.if !USE_EXTENDED_PATCH
	lds	r16, WRITE_PATCH_OFFSET
	add	r16, r18		; r16 contains the eeprom address of the byte to write, parameter is in r17
	ldi	r19, 0			; eeprom data are in the lowest page of the eeprom
.else	; !USE_EXTENDED_PATCH
	lds	r16, WRITE_PATCH_OFFSET_L
	lds	r19, WRITE_PATCH_OFFSET_H
	add	r16, r18		; r16 contains the eeprom address of the byte to write, parameter is in r17
	adc	r19, ZERO		; High byte of eeprom address
.endif	; !USE_EXTENDED_PATCH

;-----------------------------------------------------------------------------
; Store a single parameter value to eeprom. r16 is the offset, r17 is the data
;-----------------------------------------------------------------------------
;
; r16 eeprom (low) addr
; r17 data
; r18 eeprom offset (WRITE_OFFSET)
; r19 eeprom high address

;WRITE_BYTE:
	out	EEARH, r19
	out	EEARL, r16		; single byte offset from WRITE_OFFSET
	out	EEDR, r17		; Write data (r17) to data register
	in 	temp_SREG, SREG 	; store SREG value
	cli				; Disable global interrupts during timed write
	sbi	EECR,EEMWE		; Write logical one to EEMWE
	sbi	EECR,EEWE		; Start eeprom write by setting EEWE
	out	SREG, temp_SREG 	; restore SREG value (I-bit - restarts global interrupts)

	; check for all data written to eeprom
.if !USE_EXTENDED_PATCH
	cpi	r18, 15			; If eeprom write offset is at the end patch (byte 15)
.else	; !USE_EXTENDED_PATCH
	cpi	r18, $23		; If eeprom write offset is at the end patch (byte 47)
.endif	; !USE_EXTENDED_PATCH
	breq	CLEAR_WRITE
	inc	r18
	sts	WRITE_OFFSET, r18 	; increment and store eeprom offset for next parameter
	rjmp	MLP_SKIPSCAN

CLEAR_WRITE:
	ldi	r17, 255
	sts	WRITE_MODE, r17		; Set write mode to 255 (off)

;-----------------------------------------------------------------------------
; Read potentiometer values
;-----------------------------------------------------------------------------
;

MLP_SKIPSCAN:

	;---------------------
	;read potentiometers:
	;---------------------

	rcall	ADC_END			; r16 = AD(i)
	lds	r18, ADC_CHAN		;\
	sts	PREV_ADC_CHAN, r18	; keep track of which ADC channel we're processing.
	ldi	r28, low(ADC_0)
	ldi	r29, high(ADC_0)
	add	r28, r18
	adc	r29, ZERO
	st	Y, r16			; AD(i) --> ADC_i

;next channel:
	inc	r18
	andi	r18, 0x07
	sts	ADC_CHAN,r18
	rcall	ADC_START		; start conversion of next channel


;-----------------------------------------------------------------------------
; Store knob values based on KNOB SHIFT switch setting
;
; Pots 2-5 have two parameters with the KNOB SHIFT
; switch used to select knob bank 0 or 1.
;
; To make things more challenging, the ADC value read from each pot might fluctuate
; through several values. This will cause the synth to think the pot has been moved and update
; the parameter value. To avoid this, require the pot to be moved a level of at least
; X before updating (deadzone check). To reduce processing time, a knob status byte
; tracks whether the pots have been moved since the KNOB SHIFT switch was updated.
; If the status bit is set, we can just skip the deadzone check and update.
;-----------------------------------------------------------------------------
;
; First process the unshifted pots (0, 1, 6, 7)
;

	lds	r20, PREV_ADC_CHAN	; Only process the most recently scanned pot, store ADC channel 0..7 in r20

CHECK_0:
	cpi	r20, 0			; Update knob 0 - filter resonance?
	brne	CHECK_1
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
	breq	EXIT_CHECK_0		; Skip update if pot hasn't been updated
	sts	RESONANCE, r16
EXIT_CHECK_0:
.if USE_ORIGINAL_RESONANCE_LIMIT
	lds	r16, RESONANCE		; Limit resonance
	cpi	r16, 252		;\
	BRLO	LOAD_REZ		; | Limit maximum knob resonance to 0xf6
	ldi	r16, 252		;/
LOAD_REZ:
	sts	RESONANCE,r16
.endif	; USE_ORIGINAL_RESONANCE_LIMIT
	rjmp	DONE_KNOBS

CHECK_1:
	cpi	r20, 1			; Update knob 1 - filter cutoff?
	brne	CHECK_6
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
	breq	EXIT_KNOBA		; Skip update if pot hasn't been updated
	sts	CUTOFF,r16
	rjmp	DONE_KNOBS

CHECK_6:
	cpi	r20, 6			; Update knob 6, PWM?
	brne	CHECK_7
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
	breq	EXIT_CHECK_6		; Skip update if pot hasn't been updated
	sts	PULSE_KNOB, r16

EXIT_CHECK_6:
	; Store limited PULSE_WIDTH values
	lds	r16, PULSE_KNOB		; grab the patch value, just in case it hasn't been updated
	lsr	r16			; Divide it by two (we only need 0-50% pulse)
.if USE_ORIGINAL_PWM_LIMIT
	subi	r16, 18
	brcc	LIMIT_PWM1
	ldi	r16, 4			; restrict PULSE_KNOB_LIMITED to 4-115 (avoid 0% and 50% pulse)
	rjmp	LIMIT_PWM3
LIMIT_PWM1:
.endif	; USE_ORIGINAL_PWM_LIMIT
	cpi	r16, 4
	brlo	LIMIT_PWM2
	rjmp	LIMIT_PWM3
LIMIT_PWM2:
	ldi	r16, 4
LIMIT_PWM3:
	sts	PULSE_KNOB_LIMITED, r16
	rjmp	DONE_KNOBS

CHECK_7:
	cpi	r20, 7			; Update knob 7, OSC Detune?
	brne	KNOB_SHIFT_CHECK
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
	breq	EXIT_CHECK7		; Skip update if pot hasn't been updated
	sts	OSC_DETUNE, r16
EXIT_CHECK7:
	lds	r16, OSC_DETUNE		; grab the patch value, just in case it hasn't been updated
	rcall	NONLINPOT		; AD6.1 --> DCO B detune with non-linear knob (center is tuned)
	subi	r17, 128
	sts	DETUNEB_FRAC, r16	; Value -128.000..+127.996
	sts	DETUNEB_INTG, r17
EXIT_KNOBA:
	rjmp	DONE_KNOBS


KNOB_SHIFT_CHECK:

; Now process the shifted pots ( 2, 3, 4, 5)
; Check which bank of knob parameters we're updating.

	lds	r19, PATCH_SWITCH1
	sbrc	r19, SW_KNOB_SHIFT	; If knob Shift bit set, jump to process bank 1
	jmp	KNOB_BANK_1

;-----------------------------------------------------------------------------
; KNOB BANK 0 - unshifted ( glide, filter envelope amount, LFO depth, LFO rate)
;-----------------------------------------------------------------------------

	cpi	r20, 2			; Update knob 2, LFO rate?
	brne	CHECK_3
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
.if USE_ORIGINAL_ENVELOPE_CONTROL
	breq	EXIT_CHECK2A		; Skip update if pot not updated, post-process DCF_DECAY
	sts	LFOFREQ, r16
	rjmp	EXIT_CHECK2A
.else	;USE_ORIGINAL_ENVELOPE_CONTROL
	breq	EXIT_KNOBB		; Skip update if pot hasn't been updated
	sts	LFOFREQ, r16
	rjmp	DONE_KNOBS
.endif	;USE_ORIGINAL_ENVELOPE_CONTROL

CHECK_3:
	cpi	r20, 3			; Update knob 3, LFO depth?
	brne	CHECK_4
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
	breq	EXIT_KNOBB		; Skip update if pot hasn't been updated
	sts	LFOLEVEL,r16
	sts	PANEL_LFOLEVEL, r16
	rjmp	DONE_KNOBS

CHECK_4:
	cpi	r20, 4			; Update knob 4, Filter envelope amount?
	brne	CHECK_5
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
.if USE_ORIGINAL_ENVELOPE_CONTROL
	breq	EXIT_CHECK4A		; Skip update if pot hasn't been updated(post process AMP_DECAY parameters)
	sts	VCFENVMOD, r16
	rjmp	EXIT_CHECK4A
.else	; USE_ORIGINAL_ENVELOPE_CONTROL
	breq	EXIT_KNOBB		; Skip update if pot hasn't been updated
	sts	VCFENVMOD, r16
	rjmp	DONE_KNOBS
.endif	; USE_ORIGINAL_ENVELOPE_CONTROL

CHECK_5:
	cpi	r20, 5			; Update knob 5, Glide/Portamento?
	brne	EXIT_KNOBB
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
	breq	EXIT_KNOBB		; Skip update if pot hasn't been updated
	lsr	r16			; 50% of original value +
	mov	r17, r16
	lsr	r17			; 25% of original =
	add	r16, r17		; 75% of original
	sts	PORTAMENTO,r16
EXIT_KNOBB:
	rjmp	DONE_KNOBS
;-----------------------------------------------------------------------------
; KNOB BANK 1
;-----------------------------------------------------------------------------
KNOB_BANK_1:

CHECK_2A:
	cpi	r20, 2			; Update knob 2^, Filter decay?
	brne	CHECK_3A
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
.if USE_ORIGINAL_ENVELOPE_CONTROL
	breq	EXIT_CHECK2A		; Skip update if pot hasn't been updated
	sts	KNOB_DCF_DECAY, r16
EXIT_CHECK2A:
	lds	r16, KNOB_DCF_DECAY	; Grab knob patch value just in case it hasn't been changed
	lds	r19, PATCH_SWITCH2
	sbrs	r19, SW_SUSTAIN
	rjmp	SUSTAIN_OFF		; skip if sustain switch is off
	ldi	r19, 255		; Sustain is on, so...
	sts	SUSTAINLEVEL2, r19	; Set sustain to maximum
	sts	DECAYTIME2, r19		; Set decay to maximum
	sts	RELEASETIME2, r16	; Set release time to value of decay knob
	rjmp	DONE_KNOBS

SUSTAIN_OFF:
	ldi	r19, 0			; Sustain is off, so...
	sts	SUSTAINLEVEL2, r19	; Set sustain to minimum
	sts	DECAYTIME2, r16		; Set decay time to value of decay knob
	sts	RELEASETIME2, r16	; Set release time to value of decay knob
	rjmp	DONE_KNOBS
.else	; USE_ORIGINAL_ENVELOPE_CONTROL
	breq	DONE_KNOBS		; Skip update if pot hasn't been updated
	sts	KNOB_DCF_DECAY, r16
; set update_env_dcf flags
	lds	r16, MIDI_TASK_UPDATE
	ori	r16, (1<<UPDATE_ENV_DCF)
	sts	MIDI_TASK_UPDATE, r16
	rjmp	DONE_KNOBS
.endif	; USE_ORIGINAL_ENVELOPE_CONTROL

CHECK_3A:
	cpi	r20, 3			; Update knob 3^, Filter attack?
	brne	CHECK_4A
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
	breq	DONE_KNOBS		; Skip update if pot hasn't been updated
	sts	KNOB_DCF_ATTACK, r16
.if !USE_ORIGINAL_ENVELOPE_CONTROL
	sts	ATTACKTIME2, r16
.endif	; !USE_ORIGINAL_ENVELOPE_CONTROL
	rjmp	DONE_KNOBS

CHECK_4A:
	cpi	r20, 4			; Update knob 4^, Amplitude decay?
	brne	CHECK_5A
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
.if USE_ORIGINAL_ENVELOPE_CONTROL
	breq	EXIT_CHECK4A		; Skip update if pot hasn't been updated
	sts	KNOB_AMP_DECAY, r16
EXIT_CHECK4A:
	lds	r16, KNOB_AMP_DECAY	; Grab knob patch value just in case it hasn't been changed
	lds	r19, PATCH_SWITCH2
	sbrs	r19, SW_SUSTAIN
	rjmp	DCF_SUSTAIN_OFF		; Skip if sustain is off...
	ldi	r19, 255		; Sustain is on, so...
	sts	SUSTAINLEVEL, r19	; Set sustain to maximum
	sts	DECAYTIME, r19		; Set decay to maximum
	sts	RELEASETIME, r16	; Set release time to value of decay knob
	rjmp	DONE_KNOBS
DCF_SUSTAIN_OFF:
	ldi	r19, 0			; Sustain is off, so...
	sts	SUSTAINLEVEL, r19	; Set sustain to minimum
	sts	DECAYTIME, r16		; Set decay time to value of decay knob
	sts	RELEASETIME, r16	; Set release time to value of decay knob
	rjmp	DONE_KNOBS
.else	; USE_ORIGINAL_ENVELOPE_CONTROL
	breq	DONE_KNOBS		; Skip update if pot hasn't been updated
	sts	KNOB_AMP_DECAY, r16
; set update_env_dca flags
	lds	r16, MIDI_TASK_UPDATE
	ori	r16, (1<<UPDATE_ENV_DCA)
	sts	MIDI_TASK_UPDATE, r16
	rjmp	DONE_KNOBS
.endif	; USE_ORIGINAL_ENVELOPE_CONTROL

CHECK_5A:
	cpi	r20, 5			; Update knob 5^, Amplitude attack?
	brne	DONE_KNOBS
	rcall	POT_SCAN		; If so, check if parameter should be updated with pot value in r16
	tst	r17
	breq	DONE_KNOBS		; Skip update if pot hasn't been updated
	sts	KNOB_AMP_ATTACK, r16
.if !USE_ORIGINAL_ENVELOPE_CONTROL
	sts	ATTACKTIME, r16
.endif	; !USE_ORIGINAL_ENVELOPE_CONTROL

DONE_KNOBS:

;-----------------------------------------------------------------------------
; Add MIDI velocity code here
;-----------------------------------------------------------------------------

MIDI_VELOCITY:
	; Add velocity control of filter

	lds 	R16, MIDIVELOCITY		; Value is 0..127
	lsl	R16
	lds	r17, VCFENVMOD
	mul	r16, r17
; !!!! todo: add switch to enable midi velocity to dcf
;	mov	r1, r17
	sts	VELOCITY_ENVMOD, r1

;-----------------------------------------------------------------------------

	;-------------
	;calculate dT:
	;-------------
	in	r22, TCNT1L		;\
	in	r23, TCNT1H		;/ r23:r22 = TCNT1 = t
	mov	r18, r22		;\
	mov	r19, r23		;/ r19:r18 = t
	lds	r16, TPREV_L		;\
	lds	r17, TPREV_H		;/ r17:r16 = t0
	sub	r22, r16		;\ r23:r22 = t - t0 = dt
	sbc	r23, r17		;/ (1 bit = 32 µs)
	sts	TPREV_L, r18		;\
	sts	TPREV_H, r19		;/ t0 = t
	sts	DELTAT_L, r22		;\
	sts	DELTAT_H, r23		;/ r23:r22 = dT

	;----
	;LFO:
	;----

;calculate dA:
	lds	r16, LFOFREQ		;\
	com	r16			;/ r16 = 255 - ADC0
	rcall	ADCTORATE		; r19:r18:r17:r16 = rate of rise/fall
	lds	r22, DELTAT_L		;\
	lds	r23, DELTAT_H		;/ r23:r22 = dT
	rcall	MUL32X16		; r18:r17:r16 = dA
; cacluate lfo waveform (sawtooth)
	lds	r19, LFO_FRAC_L		;\
	lds	r20, LFO_FRAC_H		; > A
	lds	r21, LFO_INTEGR		;/
	subi	r21, 128
	ldi	r31, 0			; flag = 0 (no over/underflow)
	lds	r30, LFOPHASE
	tst	r30			; check for zero
	brne	MLP_LFOFALL		; 1: falling

;rising phase:

MLP_LFORISE:
	lds	r22, LFOTOP_0		;\
	lds	r23, LFOTOP_1		; > r24:r23:r22 = Amax
	lds	r24, LFOTOP_2		;/
	add	r19, r16		;\
	adc	r20, r17		; > A += dA
	adc	r21, r18		;/
	brcs	MLP_LFOTOP
	cp	r19, r22		;\
	cpc	r20, r23		; > A - Amax
	cpc	r21, r24		;/
	brlo	MLP_LFOX		; skip when A < Amax

;A reached top limit:

MLP_LFOTOP:
	mov	r19, r22		;\
	mov	r20, r23		; > A = Amax
	mov	r21, r24		;/
	ldi	r30, 1			; begin of falling
	ldi	r31, 1			; flag = 1 (Amax overflow)
	rjmp	MLP_LFOX

;falling phase:

MLP_LFOFALL:
	lds	r22, LFOBOTTOM_0	;\
	lds	r23, LFOBOTTOM_1	; > r24:r23:r22 = Amin
	lds	r24, LFOBOTTOM_2	;/
	sub	r19, r16		;\
	sbc	r20, r17		; > A -= dA
	sbc	r21, r18		;/
	brcs	MLP_LFOBOTTOM
	cp	r22, r19		;\
	cpc	r23, r20		; > Amin - A
	cpc	r24, r21		;/
	brlo	MLP_LFOX		; skip when A > Amin

;A reached bottom limit:

MLP_LFOBOTTOM:
	mov	r19, r22		;\
	mov	r20, r23		; > A = Amin
	mov	r21, r24		;/
	ldi	r30, 0			; begin of rising
	ldi	r31, 1			; flag = 1 (Amin underflow)

MLP_LFOX:
	sts	LFOPHASE, r30
	subi	r21, 128		; r21,r20:r19 = LFO tri wave: -128,000..+127,999
	sts	LFO_FRAC_L, r19		;\
	sts	LFO_FRAC_H, r20		; > store LFO value
	sts	LFO_INTEGR, r21		;/

;switch norm/rand:

;determine Amin i Amax:
	ldi	r16, 0			;\
	ldi	r17, 0			; > Amin when not LFO==tri
	ldi	r18, 0			;/  and not LFO==rand
	lds	r30, PATCH_SWITCH1
	sbrs	r30, SW_LFO_RANDOM	; LFO random if switch is set
	RJMP	MLP_LFOAWR
	tst	r31			; test for over/underflow
	breq	MLP_LFOAX
	lds	r16, SHIFTREG_0		;\
	lds	r17, SHIFTREG_1		; \ Amin = pseudo-random number
	lds	r18, SHIFTREG_2		; /	0,000..127,999
	andi	r18, 0x7F		;/

MLP_LFOAWR:
; store Amin, Amax values for squ and rand
	sts	LFOBOTTOM_0, r16	;\
	sts	LFOBOTTOM_1, r17	; > store Amin
	sts	LFOBOTTOM_2, r18	;/
	com	r16			;\
	com	r17			; > Amax = 255,999 - Amin
	com	r18			;/	128,000..255,999
	sts	LFOTOP_0, r16		;\
	sts	LFOTOP_1, r17		; > store Amax
	sts	LFOTOP_2, r18		;/

MLP_LFOAX:
	lds	r16, PATCH_SWITCH1
	sbrs	r16, SW_LFO_RANDOM
	rjmp	MLP_LFONORM
	tst	r31			; flag == 1 ? - branch on not over- or underflow
	breq	MLP_LFONWR		; jump when not
	lds	r21, SHIFTREG_2
	rjmp	MLP_LFOWR

MLP_LFONORM:

;switch tri/squ:
	lds	r16, PATCH_SWITCH1	;\ Z=0: triangle
	sbrs	r16, SW_LFO_WAVE	;/ Z=1: square
	rjmp	MLP_LFOWR
	lsl	r21			; Cy = (LFO < 0)
	ldi	r21, 127		;\
	adc	r21, ZERO		;/ r21 = -128 or +127

MLP_LFOWR:
	sts	LFOVALUE, r21		; -128..+127

; Modulation wheel: Use highest value (Front panel or MIDI)
MLP_LFONWR:
.if USE_MORE_SWITCHES
	lds	R16, PATCH_SWITCH3
	sbrs	r16, SW_MODWHEEL_ENABLE
	rjmp	MLP_LFOMWX		; skip when MOD.WHEEL = off
.endif	; USE_MORE_SWITCHES
	lds	r16, PANEL_LFOLEVEL
	lds	r17, MIDIMODWHEEL
	cp	r16, r17
	brsh	MLP_LFOLWR
	mov	r16, r17		; MOD.WHEEL is greater

MLP_LFOLWR:
	sts	LFOLEVEL, r16

MLP_LFOMWX:

	;----
	;LFO2 (Used to sweep PWM waveform)
	;----

;calculate dA:
	lds	r16, PULSE_KNOB		; Use PULSE_KNOB as PWM Sweep rate. (=LFO2FREQ)
	lsr	r16
	lsr	r16			; Limit PWM sweep rate to 0..63 to avoid PWM aliasing
	com	r16			;/ r16 = 255 - ADC0
	rcall	ADCTORATE		; r19:r18:r17:r16 = rate of rise/fall
	lds	r22, DELTAT_L		;\
	lds	r23, DELTAT_H		;/ r23:r22 = dT
	rcall	MUL32X16		; r18:r17:r16 = dA
; cacluate lfo waveform (sawtooth)
	lds	r19, LFO2_FRAC_L	;\
	lds	r20, LFO2_FRAC_H	; > A
	lds	r21, LFO2_INTEGR	;/
	subi	r21, 128
	ldi	r31, 0			; flag = 0 (no over/underflow)
	lds	r30, LFO2PHASE
	tst	r30
	brne	MLP_LFO2FALL

;rising phase:

MLP_LFO2RISE:
	lds	r22, LFO2TOP_0		;\
	lds	r23, LFO2TOP_1		; > r24:r23:r22 = Amax
	lds	r24, LFO2TOP_2		;/
	add	r19, r16		;\
	adc	r20, r17		; > A += dA
	adc	r21, r18		;/
	brcs	MLP_LFO2TOP
	cp	r19, r22		;\
	cpc	r20, r23		; > A - Amax
	cpc	r21, r24		;/
	brlo	MLP_LFO2X		; skip when A < Amax

;A reached top limit:

MLP_LFO2TOP:
	mov	r19, r22		;\
	mov	r20, r23		; > A = Amax
	mov	r21, r24			;/
	ldi	r30, 1			; begin of falling
	ldi	r31, 1			; flag = 1 (Amax overflow)
	rjmp	MLP_LFO2X

;falling phase:

MLP_LFO2FALL:
	lds	r22, LFO2BOTTOM_0	;\
	lds	r23, LFO2BOTTOM_1	; > r24:r23:r22 = Amin
	lds	r24, LFO2BOTTOM_2	;/
	sub	r19, r16		;\
	sbc	r20, r17		; > A -= dA
	sbc	r21, r18		;/
	brcs	MLP_LFO2BOTTOM
	cp	r22, r19		;\
	cpc	r23, r20		; > Amin - A
	cpc	r24, r21		;/
	brlo	MLP_LFO2X		; skip when A > Amin

;A reached bottom limit:

MLP_LFO2BOTTOM:
	mov	r19, r22		;\
	mov	r20, r23		; > A = Amin
	mov	r21, r24		;/
	ldi	r30, 0			; begin of rising
	ldi	r31, 1			; flag = 1 (Amin underflow)

MLP_LFO2X:
	sts	LFO2PHASE, r30
	subi	r21, 128		; r21,r20:r19 = LFO2 tri wave: -128,000..+127,999
	sts	LFO2_FRAC_L, r19	;\
	sts	LFO2_FRAC_H, r20	; > store LFO2 value
	sts	LFO2_INTEGR, r21	;/

; -------------
.if USE_EXTENDED_LFO2
;switch norm/rand:

;determine Amin i Amax:
	ldi	r16, 0			;\
	ldi	r17, 0			; > Amin when not LFO==tri
	ldi	r18, 0			;/  and not LFO==rand
	lds	r30, PATCH_SWITCH4
	sbrs	r30, SW_LFO2_RANDOM	; LFO random if switch is set
	RJMP	MLP_LFO2AWR
	tst	r31			; test for over/underflow
	breq	MLP_LFO2AX		; branch on not over- or underflow
	lds	r16, SHIFTREG_0		;\
	lds	r17, SHIFTREG_1		; \ Amin = pseudo-random number
	lds	r18, SHIFTREG_2		; /	0,000..127,999
	andi	r18, 0x7F		;/

MLP_LFO2AWR:
; store Amin, Amax values for squ and rand
	sts	LFO2BOTTOM_0, r16	;\
	sts	LFO2BOTTOM_1, r17	; > store Amin
	sts	LFO2BOTTOM_2, r18	;/
	com	r16			;\
	com	r17			; > Amax = 255,999 - Amin
	com	r18			;/	128,000..255,999
	sts	LFO2TOP_0, r16		;\
	sts	LFO2TOP_1, r17		; > store Amax
	sts	LFO2TOP_2, r18		;/

MLP_LFO2AX:
	lds	r16, PATCH_SWITCH4
	sbrs	r16, SW_LFO2_RANDOM
	rjmp	MLP_LFO2NORM
	tst	r31			; flag == 1 ? - branch on not over- or underflow
	breq	MLP_LFO2NWR		; jump when not
	lds	r21, SHIFTREG_2
	rjmp	MLP_LFO2WR

MLP_LFO2NORM:

;switch tri/squ:
	lds	r16, PATCH_SWITCH4	;\ Z=0: triangle
	sbrs	r16, SW_LFO2_WAVE	;/ Z=1: square
	rjmp	MLP_LFO2WR
	lsl	r21			; Cy = (LFO < 0)
	ldi	r21, 127		;\
	adc	r21, ZERO		;/ r21 = -128 or +127
MLP_LFO2WR:
	sts	LFO2VALUE, r21		; -128..+127 (currently not used)
.endif	; USE_EXTENDED_LFO2

MLP_LFO2NWR:

.if USE_MIDI_PWMDEPTH
; calculate pwm width
; in:  r21 LFO2VALUE = -128..+127
;      r22 PWMDEPTH = 0..255
; out: r21 PULSE_WIDTH = 0..255

	lds	r22, PWMDEPTH		; (=LFO2LEVEL)
	mulsu	r21, r22		; PWMDEPTH *  (=LFO2LEVEL*LFO2VALUE)
	mov	r21, r1
	subi	r21, $80		; remove sign

; next lines are useful when we remove the switch SW_PWM_SWEEP in the ISR
;	lds	r22, PATCH_SWITCH1
;	sbrs	r22, SW_PWM_SWEEP	; pwm sweep = off
;	lds	r21, PULSE_KNOB_LIMITED

	sts	PULSE_WIDTH, r21	; Update pulse width value

; -------------
.else	; USE_MIDI_PWMDEPTH
	subi	r21, $80		; remove sign (LFO2_INTEGR = LFO2VALUE)
	sts	PULSE_WIDTH, r21	; Update pulse width value
.endif	; USE_MIDI_PWMDEPTH

	;----
	;ENV: (DCA)
	;----
;check envelope phase:
	lds	r17, ENVPHASE
	lds	r16, KNOB_AMP_ATTACK

MLP_PHASE:
	cpi	r17, 1
	breq	MLP_ENVAR		; when "attack"
	lds	r16, DECAYTIME
	cpi	r17, 2
	breq	MLP_ENVAR		; when "decay"
	lds	r16, RELEASETIME
	cpi	r17, 4
	breq	MLP_ENVAR		; when "release"
	lds	r22, SUSTAINLEVEL
	cpi	r17, 3			; when sustain
	breq	MLP_ESUSTAIN
	rjmp	MLP_EEXIT		; when "stop" or "sustain"

;calculate dL:

MLP_ENVAR:
	rcall	ADCTORATE		; r19:r18:r17:r16 = rate of rise/fall
	lds	r22, DELTAT_L		;\
	lds	r23, DELTAT_H		;/ r23:r22 = dT
	rcall	MUL32X16		; r18:r17:r16 = dL

;add/subtract dL to/from L:
	lds	r19, ENV_FRAC_L		;\
	lds	r20, ENV_FRAC_H		; > r21:r20:r19 = L
	lds	r21, ENV_INTEGR		;/
	lds	r22, ENVPHASE
	cpi	r22, 4
	breq	MLP_ERELEASE

MLP_EATTACK:
	cpi	r22, 2
	breq	MLP_EDECAY
	add	r19, r16		;\
	adc	r20, r17		; > r21:r20:r19 = L + dL
	adc	r21, r18		;/
	brcc	MLP_ESTORE		; overflow? L + dL < 256?

;L reached top limit:			; yes
	ldi	r19, 255		;\
	ldi	r20, 255		; > L = Lmax
	ldi	r21, 255		;/
	ldi	r16, 2			; now decay
	rjmp	MLP_ESTOREP

MLP_EDECAY:
	sub	r19, r16		;\
	sbc	r20, r17		; > r21:r20:r19 = L - dL
	sbc	r21, r18		;/
	brcs	MLP_BOTTOM		; Exit if we went past bottom level. L - dL < 0?
	lds	r22, SUSTAINLEVEL
	cp	r22, r21		; sustain level < L - dL?
	brlo	MLP_ESTORE		; yes: Keep going if we haven't hit sustain level
	ldi	r16, 3			; now sustain
	sts	ENVPHASE, r16		; store phase
MLP_ESUSTAIN:
	clr	r19			; correct sustain level after decay is done
	clr	r20
	mov	r21, r22
	rjmp	MLP_ESTORE

MLP_ERELEASE:
	sub	r19, r16		;\
	sbc	r20, r17		; > r21:r20:r19 = L - dL
	sbc	r21, r18		;/
	brcc	MLP_ESTORE

;L reached bottom limit:
MLP_BOTTOM:
	ldi	r19, 0			;\
	ldi	r20, 0			; > L = 0
	ldi	r21, 0			;/
	ldi	r16, 0			; stop

MLP_ESTOREP:
	sts	ENVPHASE, r16		; store phase

MLP_ESTORE:
	sts	ENV_FRAC_L, r19		;\
	sts	ENV_FRAC_H, r20		; > store L
	sts	ENV_INTEGR, r21		;/
MLP_EEXIT:

	;----
	;ENV 2 (DCF):
	;----
;check envelope phase:
	lds	r17, ENVPHASE2
	lds	r16, KNOB_DCF_ATTACK

MLP_PHASE2:
	cpi	r17, 1
	breq	MLP_ENVAR2		; when "attack"
	lds	r16, DECAYTIME2
	cpi	r17, 2
	breq	MLP_ENVAR2		; when "decay"
	lds	r16, RELEASETIME2
	cpi	r17, 4
	breq	MLP_ENVAR2		; when "release"
	lds	r22, SUSTAINLEVEL2
	cpi	r17, 3			; when sustain
	breq	MLP_ESUSTAIN2
	rjmp	MLP_EEXIT2		; when "stop" or "sustain"

;calculate dL:

MLP_ENVAR2:
	rcall	ADCTORATE		; r19:r18:r17:r16 = rate of rise/fall
	lds	r22, DELTAT_L		;\
	lds	r23, DELTAT_H		;/ r23:r22 = dT
	rcall	MUL32X16		; r18:r17:r16 = dL

;add/subtract dL to/from L:
	lds	r19, ENV_FRAC_L2	;\
	lds	r20, ENV_FRAC_H2	; > r21:r20:r19 = L
	lds	r21, ENV_INTEGR2	;/
	lds	r22, ENVPHASE2
	cpi	r22, 4
	breq	MLP_ERELEASE2

MLP_EATTACK2:
	cpi	r22, 2
	breq	MLP_EDECAY2
	add	r19, r16		;\
	adc	r20, r17		; > r21:r20:r19 = L + dL
	adc	r21, r18		;/
	brcc	MLP_ESTORE2

;L reached top limit:
	ldi	r19, 255		;\
	ldi	r20, 255		; > L = Lmax
	ldi	r21, 255		;/
	ldi	r16, 2			; now decay
	rjmp	MLP_ESTOREP2

MLP_EDECAY2:
	sub	r19, r16		;\
	sbc	r20, r17		; > r21:r20:r19 = L - dL
	sbc	r21, r18		;/
	brcs	MLP_BOTTOM2		; Exit if we went past bottom level
	lds	r22, SUSTAINLEVEL2
	cp	r22, r21
	brlo	MLP_ESTORE2		; Keep going if we haven't hit sustain level
	ldi	r16, 3			; now sustain
	sts	ENVPHASE2, r16		; store phase
MLP_ESUSTAIN2:
	clr	r19			; correct sustain level after decay is done
	clr	r20
	mov	r21, r22
	rjmp	MLP_ESTORE2

MLP_ERELEASE2:
	sub	r19, r16		;\
	sbc	r20, r17		; > r21:r20:r19 = L - dL
	sbc	r21, r18		;/
	brcc	MLP_ESTORE2

;L reached bottom limit:
MLP_BOTTOM2:
	ldi	r19, 0			;\
	ldi	r20, 0			; > L = 0
	ldi	r21, 0			;/
	ldi	r16, 0			; stop
	sts	VCF_STATUS, r16		; Flag VCF as off when we hit the bottom limit.

MLP_ESTOREP2:
	sts	ENVPHASE2, r16		; store phase

MLP_ESTORE2:
	sts	ENV_FRAC_L2, r19	;\
	sts	ENV_FRAC_H2, r20	; > store L
	sts	ENV_INTEGR2, r21	;/
MLP_EEXIT2:
	; End of Envelope 2

	;-----
	;GATE:
	;-----
	lds	r16, GATE		; set on note on, cleared on note off
	tst	r16			; check GATE
	brne	MLP_KEYON

;no key is pressed:
MLP_KEYOFF:
	ldi	r16,4			;\
	; don't restart envelope 1 for release if it's already stopped.
	lds	r17, ENVPHASE
	tst	r17
	breq	MLP_NOTEON		; Don't put envelope 1 in release mode if it is already stopped

	sts	ENVPHASE, r16		;/ "release"
	; don't restart envelope 2 for release if it's already stopped.
	lds	r17, ENVPHASE2
	tst	r17
	breq	MLP_NOTEON		; Don't put envelope 2 in release mode if it is already stopped
			;
	sts	ENVPHASE2, r16		; "release" for envelope 2
	rjmp	MLP_NOTEON

;key is pressed:
MLP_KEYON:
	lds	r16, GATEEDGE
	tst	r16			; Z=0 when key has just been pressed
	breq	MLP_NOTEON

;key has just been pressed:
	ldi	r16, 0			;\
	sts	GATEEDGE, r16		;/ GATEEDGE = 0
	lds	r16, PORTACNT		;\
	tst	r16			; \
	breq	MLP_KEYON1		;  > if ( PORTACNT != 0 )
	dec	r16			; /    PORTACNT--
	sts	PORTACNT, r16		;/

MLP_KEYON1:

;envelope starts:
	ldi	r16, 1			;\
	sts	ENVPHASE, r16		;/ attack
	sts	ENVPHASE2, r16		; attack for envelope 2
	sts	VCF_STATUS, r16		; Flag VCF as on

.if USE_INIT_ENV_GENERATORS
; can cause hearable clicks. This occurs when the envelope generator
; is retriggert before the envelope reaches zero. In this case the next
; lines forces the level to zero and we hear the breakdown.
	ldi	r16, 0
	sts	ENV_FRAC_L, r16		;\
	sts	ENV_FRAC_H, r16		; > ENV = 0
	sts	ENV_INTEGR, r16		;/

	sts	ENV_FRAC_L2, r16	;\
	sts	ENV_FRAC_H2, r16	; > ENV2 = 0
	sts	ENV_INTEGR2, r16	;/
.endif	; USE_INIT_ENV_GENERATORS

; LFO starts when note triggered:
.if USE_KBD_SYNC
	lds	r16, PATCH_SWITCH3
	sbrs	r16, SW_LFO_KBD_SYNC
	rjmp	MLP_NOTEON		; skip when LFO KBD SYNC = off
.endif	; USE_KBD_SYNC
	ldi	r16, 255		;\
	ldi	r17, 255		; > A = Amax
	ldi	r18, 127		;/
	sts	LFO_FRAC_L, r16		;\
	sts	LFO_FRAC_H, r17		; > store A
	sts	LFO_INTEGR, r18		;/
.if USE_SYNC_LFO2
	sts	LFO2_FRAC_L, r16	;\
	sts	LFO2_FRAC_H, r17	; > store A
	sts	LFO2_INTEGR, r18	;/
.endif	; USE_SYNC_LFO2
	ldi	r16, 1			;\
	sts	LFOPHASE, r16		;/ begin of falling
.if USE_SYNC_LFO2
	sts	LFO2PHASE, r16
.endif	; USE_SYNC_LFO2

MLP_NOTEON:
	;-------------
	;DCO A, DCO B:
	;-------------
	ldi	r25, 0			;\
	ldi	r22, 0			; > r23,r22:r25 = note# 0..127
	lds	r23, MIDINOTE		;/
	cpi	r23, 255		; note == 255?
	brne	MLP_NLIM2
	rjmp	MLP_VCOX		; yes:

;note# limited to 36..96:

MLP_NLIM1:
	subi	r23, 12			; shift one octave down, note -=12

MLP_NLIM2:
	cpi	r23, 97			; note >= 97
	brsh	MLP_NLIM1		; shift one octave down, until note <= 96
	rjmp	MLP_NLIM4

MLP_NLIM3:
	subi	r23, 244		; shift one octave up, note +=12

MLP_NLIM4:
	cpi	r23, 36			; note < 36
	brlo	MLP_NLIM3		; shift one octave up, until note >= 36

;transpose 1 octave down:
	subi	r23, 12			; note -= 12; Note range limited to 24..84


;portamento:
	lds	r25, NOTE_L		;\
	lds	r26, NOTE_H		; > r27,r26:r25 = nCurr
	lds	r27, NOTE_INTG		;/
	lds	r16, PORTACNT		;\
	tst	r16			; > jump when it's the first note
	brne	MLP_PORTAWR		;/  (PORTACNT != 0)
	lds	r16, PORTAMENTO
	rcall	ADCTORATE
	push	r22
	push	r23
	mov	r22, r18		;\ r23:r22 = portamento rate
	mov	r23, r19		;/ 65535..27
	ldi	r16, 0
	ldi	r17, 0
	lds	r18, DELTAT_L
	lds	r19, DELTAT_H
	ldi	r20, 3
	rcall	SHR32
	rcall	MUL32X16		; r18,r17:r16 = nDelta
	pop	r23
	pop	r22
	mov	r19, r16		;\
	mov	r20, r17		; > r21,r20:r19 = nDelta
	mov	r21, r18		;/
	lds	r25, NOTE_L		;\
	lds	r26, NOTE_H		; > r27,r26:r25 = nCurr
	lds	r27, NOTE_INTG		;/
	cp	r22, r26		;\ nEnd - nCurr
	cpc	r23, r27		;/ Cy = (nEnd < nCurr)
	brsh	MLP_PORTAADD

MLP_PORTAMIN:
	sub	r25, r19		;\
	sbc	r26, r20		; > nCurr -= nDelta
	sbc	r27, r21		;/
	cp	r22, r26		;\ nEnd - nCurr;
	cpc	r23, r27		;/ Cy = (nEnd < nCurr)
	brlo	MLP_PORTA1
	rjmp	MLP_PORTAEND

MLP_PORTAADD:
	add	r25, r19		;\
	adc	r26, r20		; > nCurr += nDelta
	adc	r27, r21		;/
	cp	r22, r26		;\ nEnd - nCurr;
	cpc	r23, r27		;/ Cy = (nEnd < nCurr)
	brsh	MLP_PORTA1

MLP_PORTAEND:
	ldi	r25, 0			;\
	mov	r26, r22		; > nCurr = nEnd
	mov	r27, r23		;/

MLP_PORTA1:
	mov	r22, r26
	mov	r23, r27

MLP_PORTAWR:
	sts	NOTE_L, r25
	sts	NOTE_H, r22
	sts	NOTE_INTG, r23

.if USE_TRANSPOSE_SWITCH
	lds	r16, PATCH_SWITCH3	; b5 = transpose: 0=down, 1=up
	sbrs	r16, SW_TRANSPOSE	;\		24..84
	subi	r23, 12			;/ n -= 12	12..72
.endif	; USE_TRANSPOSE_SWITCH

;pitch bender (-12..+12):
	lds	r16, MIDIPBEND_L	;\ r17,r16 = P.BEND
	lds	r17, MIDIPBEND_H	;/	-128,000..+127,996
	ldi	r18, 5			;\ r17,r16 = P.BEND/32
	rcall	ASR16			;/	-4,000..+3,999
	mov	r18, r16		;\ r19,r18 = P.BEND/32
	mov	r19, r17		;/	-4,000..+3,999
	add	r16, r18		;\ r17,r16 = 2/32*P.BEND
	adc	r17, r19		;/	-8,000..+7,999
	add	r16, r18		;\ r17,r16 = 3/32*P.BEND
	adc	r17, r19		;/	-12,000..+11,999
	add	r22, r16		;\
	adc	r23, r17		;/ add P.BEND to NOTE_INTG:NOTE_H

MLP_PBX:
;for "DCF KBD TRACK":
	sts	PITCH, r23		; n = 0..108


;LFO modulation:
	lds	r16, PATCH_SWITCH1	; Check LFO destination bit.
	sbrs	r16, SW_LFO_DEST	; DCF is 0, DCO is 1
	jmp	MLP_VCOLFOX		; exit when LFO=DCF
	lds	r16, LFOVALUE		; r16 = LFO         -128..+127
	lds	r17, LFOLEVEL		; r17 = LFO level      0..255

	lds	r18, PATCH_SWITCH1	; Is the LFO enabled?
	sbrs	r18, SW_LFO_ENABLE
	ldi	r17, 0			; Set LFO level to zero if switch is off

;nonlinear potentiometer function:
	mov	r18, r17		; r18 = LL (LfoLevel)
	lsr	r17			; r17 = LL/2
	cpi	r18, 128
	brlo	MLP_OM1			; skip if LL = 0..127
	subi	r17, 128		; r17 = 0,5*LL-128    -64..-1
	add	r17, r18		; r17 = 1,5*LL-128    +64..254

MLP_OM1:
	mulsu	r16, r17		; LFOVALUE*LFOLEVEL
	movw	r16, r0
	ldi	r18, 4			;\
	rcall	ASR16			;/ r17,r16 = LFO*mod / 16
	add	r22, r16		;\
	adc	r23, r17		;/ add LFO to note #

;limiting to 0..108
	tst	r23
	brpl	MLP_VCOLFO1
	ldi	r22, 0
	ldi	r23, 0
	rjmp	MLP_VCOLFOX

MLP_VCOLFO1:
	cpi	r23, 109
	brlo	MLP_VCOLFOX
	ldi	r22, 0
	ldi	r23, 108

MLP_VCOLFOX:
	push	r22			;\ note# = 0..108
	push	r23			;/ store for phase delta B

; determine the wavetable for osc A: note = 0..108

; Track which wavetable to use:
	mov	r25, r23		; Store a copy of the note number in r25
	subi	r25, 13			; 13..108
	brcc	WTA_NOUFL
	clr	r25
WTA_NOUFL:
	lsr	r25
	lsr	r25			; (108-12-1)/8 = 11
	lsr	r25			; 0..11
	sts	WAVETABLE_A, r25	; Save wavetable 0..15 for lookup when generating oscillator

;phase delta A:
;octave A:
	rcall	NOTERECALC		; r23,r22 = m12 (0,0..11,996), note
					; r20 = n12 (0..11), octave
	rcall	LOAD_DELTA		; r19:r18:r17:r16 = delta
	rcall	SHL32			; r19:r18:r17:r16 = delta*(2^exp)

;	Enabling these 3 lines will lock DCO A to 440Hz (?)
;	ldi	r17, 0xBF
;	ldi	r18, 0x9A
;	ldi	r19, 0x03

	; store delta
	sts	DELTAA_0, r17
	sts	DELTAA_1, r18
	sts	DELTAA_2, r19

;phase delta B:
	pop	r23			;\
	pop	r22			;/ n

;detune B:
	lds	r16, DETUNEB_FRAC	;\ r17,r16 = detuneB
	lds	r17, DETUNEB_INTG	;/ -128,000..+127,996
	ldi	r18, 4			;\ r17,r16 = detuneB / 16
	rcall	ASR16			;/ -8,0000..+7,9998
	add	r22, r16		;\
	adc	r23, r17		;/

;octave B:
	lds	r16, PATCH_SWITCH2	; b7 = octave B: 0=down, 1=up
	sbrc	r16, SW_OSCB_OCT
	subi	r23, 244		; n += 12

; determine the wavetable for osc B; r23: note = 0..108,
; Track which wavetable to use:
	mov	r25, r23		; Store a copy of the note number in r25
	subi	r25, 13			; 13..108
	brcc	WTB_NOUFL
	clr	r25
WTB_NOUFL:
	lsr	r25
	lsr	r25			; (108-12-1)/8 = 11
	lsr	r25			; 0..11
	sts	WAVETABLE_B, r25	; Save wavetable 0..15 for lookup when generating oscillator

	rcall	NOTERECALC		; r23,r22 = m12 (0,0..11,996), note
					; r20 = n12 (0..11), octave
	rcall	LOAD_DELTA		; r19:r18:r17:r16 = delta
	rcall	SHL32			; r19:r18:r17:r16 = delta*(2^exp)

	sts	DELTAB_0, r17
	sts	DELTAB_1, r18
	sts	DELTAB_2, r19

MLP_VCOX:

	;----
	;DCF:
	;----
	;LFO mod:
	ldi	r30, 0			;\
	ldi	r31, 0			;/ sum = 0

	lds	r16, PATCH_SWITCH1	; Check LFO destination bit.
	sbrc	r16, SW_LFO_DEST	; DCF is 0, DCO is 1
	jmp	MLP_DCF0		; exit when LFO=DCO
	lds	r16, LFOVALUE		; r16 = LFO	-128..+127
	lds	r17, LFOLEVEL		; r17 = DCF LFO MOD	0..255
	lds	r18, PATCH_SWITCH1	; Is the LFO enabled?
	sbrs	r18, SW_LFO_ENABLE
	ldi	r17, 0			; Set LFO level to zero if switch is off
	mulsu	r16, r17
	mov	r30, r1
	ldi	r31, 0
	rol	r1			; r1.7 --> Cy (sign)
	sbc	r31, r31		; sign extension to r31
					; r31:r30 = -255..+255
MLP_DCF0:

;ENV mod:
.if USE_ORIGINAL_VCFENVMOD
	lds	r16, ENV_INTEGR2	; Get the integer part of the filter envelope
	lds	r17, VELOCITY_ENVMOD	; Use MIDI velocity * envmod
	mul	r16, r17
	movw	r16, r0			; r17,r16 = FILTER ENV * ENVMOD
	rol	r16			; Cy = r16.7 (for rounding)
	adc	r30, r17
	adc	r31, ZERO
.else	; USE_ORIGINAL_VCFENVMOD
	lds	r16, VELOCITY_ENVMOD
	subi	r16, 0x80		; make signed -128..+127
	lds	r17, ENV_INTEGR2	; 0..255
	mulsu	r16, r17
	movw	r16, r0			; r17,r16 = ENVMOD * ENV
	ldi	r18, 0			; \
	sbc	r18, r18		; / sign extension
	rol	r16			; Cy = r16.7 (for rounding)
	adc	r30, r17
	adc	r31, r18
.endif	; USE_ORIGINAL_VCFENVMOD

;KBD TRACK:
.if USE_KBD_TRACK_SWICTH	; keyboard tracking
	lds	r16, PATCH_SWITCH3	;\ 1: KBD TRACK on
	sbrs	r16, SW_DCF_KBD_TRACK	;/ 0: KBD TRACK off
	rjmp	MLP_DCF3
.endif	; USE_KBD_TRACK_SWICTH
	lds	r16, PITCH		; r16 = n (12/octave)	0..96
	lsl	r16			; r16 = 2*n (24/octave)	0..192
	subi	r16, 96			; r16 = 2*(n-48) (24/octave)   -96..+96
	ldi	r17, 171		; 171/256 = 2/3

	mulsu	r16, r17
	movw	r16, r0
	ldi	r18, 0			;\
	sbrc	r17, 7			; > r18 = sign extension
	ldi	r18, 255		;/  of r17
	add	r30, r17
	adc	r31, r18

MLP_DCF3:
;CUTOFF:
	lds	r16, CUTOFF
	clr	r17
	add	r16, r30
	adc	r17, r31
	tst	r17			; limit result to 0..+255
	brpl	MLP_DCF1
	ldi	r16, 0
	rjmp	MLP_DCF2

MLP_DCF1:
	breq	MLP_DCF2
	ldi	r16, 255

MLP_DCF2:
	lsr	r16			; 0..127
	ldi	r30, low( TAB_VCF)	;\
	ldi	r31, high( TAB_VCF)	;/ Z = &Tab
	rcall	TAB_BYTE		; r0 = 1.. 255
	sts	LPF_I, r16		; Store Lowpass F value
	subi	r16, 10			; Offset HP knob value
	brcc	STORE_HPF
	ldi	r16, 0x00		; Limit HP to min of 0
STORE_HPF:
	sts	HPF_I, r16

; Limit resonance at low filter cutoff settings
	lds	r17, RESONANCE
	lds	r16, LPF_I
	cpi	r16, 16
	brlo	LIMIT_REZ		; Only limit resonance if LPF_I is 0..15
	mov	r16, r17
	rjmp	EXIT_LIMIT_REZ
LIMIT_REZ:
	ldi	r30, low( TAB_REZ)	;\
	ldi	r31, high( TAB_REZ)	;/ Z = &Tab
	rcall	TAB_BYTE		; r16 = 0..15	; r16 holds maximum allow resonance
	cp	r16, r17
	brlo	EXIT_LIMIT_REZ
	mov	r16, r17
EXIT_LIMIT_REZ:
	sts	SCALED_RESONANCE, r16	; Store scaled resonance

.if !USE_ORIGINAL_Q_CALC
;-------------------------------------------------------------------------------------------------------------------
; Scale Filter Q value to compensate for resonance loss
; Doing this here to get it out of the sample loop
;-------------------------------------------------------------------------------------------------------------------

	lds	r18, LPF_I		;load 'F' value
	ldi	r17, 0xff

	sub	r17, r18		; 1-F
	lsr	r17
	ldi	r19, Q_OFFSET
	add	r17, r19		; f = (1-F)/2+4

	sub	r16, r17		; Q-(1-f)
	brcc	REZ_OVERFLOW_CHECK	; if no overflow occured
	ldi	r16, 0x00		; 0x00 because of unsigned
REZ_OVERFLOW_CHECK:
	sts	SCALED_RESONANCE, r16
.endif	; !USE_ORIGINAL_Q_CALC

	;---------------
	;sound level:
	;---------------
.if USE_DCA_MOD_SWITCH
	lds	r17, PATCH_SWITCH3	;\ check DCA mode:
	sbrc	r17, SW_DCA_MODE	;/ 0 (gate), 1 (env)
	rjmp	MLP_VCAENV		; jump when mode==env
	lds	r16, GATE		;\
	ror	r16			;/ GATE --> Cy
	ldi	r16, 0			;\ r16 =   0 (when GATE == 0),
	sbc	r16, r16		;/ r16 = 255 (when GATE == 1)
	rjmp	MLP_VCAOK
.endif	; USE_DCA_MOD_SWITCH

MLP_VCAENV:
	lds	r16,ENV_INTEGR
	ldi	r30, low( TAB_VCA)	;\
	ldi	r31, high( TAB_VCA)	;/ Z = &Tab
	rcall	TAB_BYTE		; r0 = 2..255
MLP_VCAOK:
	sts	LEVEL, r16

	;-----------------------------
	;pseudo-random shift register:
	;-----------------------------
	;BIT = SHIFTREG.23 xor SHIFTREG.18
	;SHIFTREG = (SHIFTREG << 1) + BIT
	lds	r16, SHIFTREG_0
	lds	r17, SHIFTREG_1
	lds	r18, SHIFTREG_2
	bst	r18, 7			;\
	bld	r19, 0			;/ r19.0 = SHIFTREG.23
	bst	r18, 2			;\
	bld	r20, 0			;/ r20.0 = SHIFTREG.18
	eor	r19, r20		;r19.0 = BIT
	lsr	r19			; Cy = BIT
	rol	r16			;\
	rol	r17			; > r18:r17:r16 =
	rol	r18			;/  = (SHIFTREG << 1) + BIT
	sts	SHIFTREG_0, r16
	sts	SHIFTREG_1, r17
	sts	SHIFTREG_2, r18

	;------------------------
	;back to the main loop:
	;------------------------
	rjmp	MAINLOOP

;-----------------------------------------------------------------------------
;
;*** Bandlimited sawtooth wavetables (each table is 256 bytes long,  unsigned integer)

INV_SAW0:
	; base freqency: 25.96 Hz, discrets: 701, rms: 7.95, min: -0.87, max: 0.87

	.db	  128,   18,   17,   21,    22,   22,   22,   24
	.db	   25,   25,   26,   27,    28,   29,   30,   31
	.db	   32,   32,   33,   34,    35,   36,   37,   38
	.db	   39,   39,   40,   41,    42,   43,   44,   45

	.db	   45,   46,   47,   48,    49,   50,   51,   52
	.db	   52,   53,   54,   55,    56,   57,   58,   58
	.db	   59,   60,   61,   62,    63,   64,   65,   65
	.db	   66,   67,   68,   69,    70,   71,   71,   72

	.db	   73,   74,   75,   76,    77,   77,   78,   79
	.db	   80,   81,   82,   83,    83,   84,   85,   86
	.db	   87,   88,   89,   89,    90,   91,   92,   93
	.db	   94,   95,   96,   96,    97,   98,   99,  100

	.db	  101,  102,  102,  103,   104,  105,  106,  107
	.db	  108,  109,  109,  110,   111,  112,  113,  114
	.db	  115,  115,  116,  117,   118,  119,  120,  121
	.db	  122,  122,  123,  124,   125,  126,  127,  128

	.db	  128,  128,  129,  130,   131,  132,  133,  134
	.db	  134,  135,  136,  137,   138,  139,  140,  141
	.db	  141,  142,  143,  144,   145,  146,  147,  147
	.db	  148,  149,  150,  151,   152,  153,  154,  154

	.db	  155,  156,  157,  158,   159,  160,  160,  161
	.db	  162,  163,  164,  165,   166,  167,  167,  168
	.db	  169,  170,  171,  172,   173,  173,  174,  175
	.db	  176,  177,  178,  179,   179,  180,  181,  182

	.db	  183,  184,  185,  185,   186,  187,  188,  189
	.db	  190,  191,  191,  192,   193,  194,  195,  196
	.db	  197,  198,  198,  199,   200,  201,  202,  203
	.db	  204,  204,  205,  206,   207,  208,  209,  210

	.db	  211,  211,  212,  213,   214,  215,  216,  217
	.db	  217,  218,  219,  220,   221,  222,  223,  224
	.db	  224,  225,  226,  227,   228,  229,  230,  231
	.db	  231,  232,  234,  234,   234,  235,  239,  238


INV_SAW1:
	; base freqency: 41.20 Hz, discrets: 442, rms: 7.95, min: -0.88, max: 0.88

	.db	  128,   17,   16,   21,    23,   21,   22,   25
	.db	   25,   25,   26,   28,    28,   29,   30,   31
	.db	   31,   32,   34,   34,    35,   36,   37,   38
	.db	   38,   39,   40,   41,    42,   43,   44,   44

	.db	   45,   46,   47,   48,    49,   50,   51,   51
	.db	   52,   53,   54,   55,    56,   57,   57,   58
	.db	   59,   60,   61,   62,    63,   64,   64,   65
	.db	   66,   67,   68,   69,    70,   70,   71,   72

	.db	   73,   74,   75,   76,    76,   77,   78,   79
	.db	   80,   81,   82,   83,    83,   84,   85,   86
	.db	   87,   88,   89,   89,    90,   91,   92,   93
	.db	   94,   95,   96,   96,    97,   98,   99,  100

	.db	  101,  102,  102,  103,   104,  105,  106,  107
	.db	  108,  109,  109,  110,   111,  112,  113,  114
	.db	  115,  115,  116,  117,   118,  119,  120,  121
	.db	  122,  122,  123,  124,   125,  126,  127,  128

	.db	  128,  128,  129,  130,   131,  132,  133,  134
	.db	  134,  135,  136,  137,   138,  139,  140,  141
	.db	  141,  142,  143,  144,   145,  146,  147,  147
	.db	  148,  149,  150,  151,   152,  153,  154,  154

	.db	  155,  156,  157,  158,   159,  160,  160,  161
	.db	  162,  163,  164,  165,   166,  167,  167,  168
	.db	  169,  170,  171,  172,   173,  173,  174,  175
	.db	  176,  177,  178,  179,   180,  180,  181,  182

	.db	  183,  184,  185,  186,   186,  187,  188,  189
	.db	  190,  191,  192,  192,   193,  194,  195,  196
	.db	  197,  198,  199,  199,   200,  201,  202,  203
	.db	  204,  205,  205,  206,   207,  208,  209,  210

	.db	  211,  212,  212,  213,   214,  215,  216,  217
	.db	  218,  218,  219,  220,   221,  222,  222,  224
	.db	  225,  225,  226,  227,   228,  228,  230,  231
	.db	  231,  231,  234,  235,   233,  235,  240,  239


INV_SAW2:
	; base freqency: 65.41 Hz, discrets: 278, rms: 7.95, min: -0.85, max: 0.85

	.db	  128,   28,   22,   20,    20,   20,   21,   22
	.db	   24,   26,   27,   28,    29,   29,   30,   30
	.db	   31,   32,   33,   34,    35,   36,   37,   38
	.db	   39,   39,   40,   41,    41,   42,   43,   44

	.db	   45,   46,   47,   48,    49,   50,   50,   51
	.db	   52,   53,   54,   55,    56,   57,   58,   58
	.db	   59,   60,   61,   62,    62,   63,   64,   65
	.db	   66,   67,   68,   69,    70,   70,   71,   72

	.db	   73,   74,   75,   76,    77,   77,   78,   79
	.db	   80,   81,   82,   82,    83,   84,   85,   86
	.db	   87,   88,   89,   89,    90,   91,   92,   93
	.db	   94,   95,   96,   97,    97,   98,   99,  100

	.db	  101,  101,  102,  103,   104,  105,  106,  107
	.db	  108,  109,  109,  110,   111,  112,  113,  114
	.db	  115,  116,  116,  117,   118,  119,  120,  121
	.db	  121,  122,  123,  124,   125,  126,  127,  128

	.db	  128,  128,  129,  130,   131,  132,  133,  134
	.db	  135,  135,  136,  137,   138,  139,  140,  140
	.db	  141,  142,  143,  144,   145,  146,  147,  147
	.db	  148,  149,  150,  151,   152,  153,  154,  155

	.db	  155,  156,  157,  158,   159,  159,  160,  161
	.db	  162,  163,  164,  165,   166,  167,  167,  168
	.db	  169,  170,  171,  172,   173,  174,  174,  175
	.db	  176,  177,  178,  179,   179,  180,  181,  182

	.db	  183,  184,  185,  186,   186,  187,  188,  189
	.db	  190,  191,  192,  193,   194,  194,  195,  196
	.db	  197,  198,  198,  199,   200,  201,  202,  203
	.db	  204,  205,  206,  206,   207,  208,  209,  210

	.db	  211,  212,  213,  214,   215,  215,  216,  217
	.db	  217,  218,  219,  220,   221,  222,  223,  224
	.db	  225,  226,  226,  227,   227,  228,  229,  230
	.db	  232,  234,  235,  236,   236,  236,  234,  228


INV_SAW3:
	; base freqency: 103.83 Hz, discrets: 176, rms: 7.95, min: -0.92, max: 0.92

	.db	  128,   10,   15,   26,    21,   19,   25,   25
	.db	   23,   26,   28,   26,    28,   30,   29,   30
	.db	   33,   32,   33,   35,    35,   35,   37,   38
	.db	   38,   40,   41,   41,    42,   44,   44,   44

	.db	   46,   46,   47,   48,    49,   49,   51,   52
	.db	   52,   53,   55,   55,    56,   57,   58,   58
	.db	   60,   60,   61,   62,    63,   63,   65,   66
	.db	   66,   67,   68,   69,    70,   71,   71,   72

	.db	   73,   74,   75,   76,    77,   77,   78,   80
	.db	   80,   81,   82,   83,    83,   85,   85,   86
	.db	   87,   88,   89,   90,    91,   91,   92,   93
	.db	   94,   95,   96,   97,    97,   98,   99,  100

	.db	  101,  102,  102,  103,   105,  105,  106,  107
	.db	  108,  108,  110,  110,   111,  112,  113,  114
	.db	  115,  116,  116,  117,   118,  119,  120,  121
	.db	  122,  122,  123,  124,   125,  126,  127,  127

	.db	  128,  129,  129,  130,   131,  132,  133,  134
	.db	  134,  135,  136,  137,   138,  139,  140,  140
	.db	  141,  142,  143,  144,   145,  146,  146,  148
	.db	  148,  149,  150,  151,   151,  153,  154,  154

	.db	  155,  156,  157,  158,   159,  159,  160,  161
	.db	  162,  163,  164,  165,   165,  166,  167,  168
	.db	  169,  170,  171,  171,   173,  173,  174,  175
	.db	  176,  176,  178,  179,   179,  180,  181,  182

	.db	  183,  184,  185,  185,   186,  187,  188,  189
	.db	  190,  190,  191,  193,   193,  194,  195,  196
	.db	  196,  198,  198,  199,   200,  201,  201,  203
	.db	  204,  204,  205,  207,   207,  208,  209,  210

	.db	  210,  212,  212,  212,   214,  215,  215,  216
	.db	  218,  218,  219,  221,   221,  221,  223,  224
	.db	  223,  226,  227,  226,   228,  230,  228,  230
	.db	  233,  231,  231,  237,   235,  230,  241,  246


INV_SAW4:
	; base freqency: 164.81 Hz, discrets: 111, rms: 7.95, min: -1.00, max: 1.00

	.db	  128,    1,   27,   19,    21,   25,   20,   28
	.db	   22,   28,   25,   28,    29,   28,   32,   29
	.db	   33,   31,   34,   34,    35,   37,   36,   39
	.db	   38,   40,   40,   41,    43,   42,   45,   44

	.db	   46,   46,   48,   48,    49,   50,   50,   52
	.db	   52,   54,   54,   55,    56,   56,   58,   58
	.db	   60,   60,   61,   62,    63,   64,   64,   66
	.db	   66,   67,   68,   69,    70,   70,   72,   72

	.db	   74,   74,   75,   76,    76,   78,   78,   80
	.db	   80,   81,   82,   83,    84,   84,   86,   86
	.db	   87,   88,   89,   90,    90,   92,   92,   93
	.db	   94,   95,   96,   96,    98,   98,   99,  100

	.db	  101,  102,  102,  104,   104,  106,  106,  107
	.db	  108,  109,  110,  110,   112,  112,  113,  114
	.db	  115,  116,  116,  118,   118,  119,  120,  121
	.db	  122,  122,  124,  124,   125,  126,  127,  128

	.db	  128,  128,  129,  130,   131,  132,  132,  134
	.db	  134,  135,  136,  137,   138,  138,  140,  140
	.db	  141,  142,  143,  144,   144,  146,  146,  147
	.db	  148,  149,  150,  150,   152,  152,  154,  154

	.db	  155,  156,  157,  158,   158,  160,  160,  161
	.db	  162,  163,  164,  164,   166,  166,  167,  168
	.db	  169,  170,  170,  172,   172,  173,  174,  175
	.db	  176,  176,  178,  178,   180,  180,  181,  182

	.db	  182,  184,  184,  186,   186,  187,  188,  189
	.db	  190,  190,  192,  192,   193,  194,  195,  196
	.db	  196,  198,  198,  200,   200,  201,  202,  202
	.db	  204,  204,  206,  206,   207,  208,  208,  210

	.db	  210,  212,  211,  214,   213,  215,  216,  216
	.db	  218,  217,  220,  219,   221,  222,  222,  225
	.db	  223,  227,  224,  228,   227,  228,  231,  228
	.db	  234,  228,  236,  231,   235,  237,  229,  255


INV_SAW5:
	; base freqency: 261.62 Hz, discrets: 70, rms: 7.95, min: -1.00, max: 1.00

	.db	  128,   26,    1,   24,    30,   17,   19,   29
	.db	   27,   21,   26,   31,    27,   26,   32,   33
	.db	   30,   31,   36,   35,    33,   36,   39,   37
	.db	   37,   41,   41,   40,    42,   44,   44,   43

	.db	   46,   47,   46,   47,    50,   50,   50,   51
	.db	   53,   53,   53,   56,    57,   56,   57,   59
	.db	   60,   59,   61,   63,    62,   63,   65,   66
	.db	   66,   67,   69,   69,    69,   71,   72,   72

	.db	   73,   75,   75,   75,    77,   78,   78,   79
	.db	   80,   81,   81,   82,    84,   85,   85,   86
	.db	   88,   88,   88,   90,    91,   91,   92,   94
	.db	   94,   94,   96,   97,    97,   98,  100,  100

	.db	  100,  102,  103,  103,   104,  105,  106,  107
	.db	  107,  109,  110,  110,   111,  113,  113,  113
	.db	  115,  116,  116,  117,   119,  119,  119,  121
	.db	  122,  122,  123,  125,   125,  125,  127,  128

	.db	  128,  128,  129,  131,   131,  131,  133,  134
	.db	  134,  135,  137,  137,   137,  139,  140,  140
	.db	  141,  143,  143,  143,   145,  146,  146,  147
	.db	  149,  149,  150,  151,   152,  153,  153,  154

	.db	  156,  156,  156,  158,   159,  159,  160,  162
	.db	  162,  162,  164,  165,   165,  166,  168,  168
	.db	  168,  170,  171,  171,   172,  174,  175,  175
	.db	  176,  177,  178,  178,   179,  181,  181,  181

	.db	  183,  184,  184,  185,   187,  187,  187,  189
	.db	  190,  190,  191,  193,   194,  193,  195,  197
	.db	  196,  197,  199,  200,   199,  200,  203,  203
	.db	  203,  205,  206,  206,   206,  209,  210,  209

	.db	  210,  213,  212,  212,   214,  216,  215,  215
	.db	  219,  219,  217,  220,   223,  221,  220,  225
	.db	  226,  223,  224,  230,   229,  225,  230,  235
	.db	  229,  227,  237,  239,   226,  232,  255,  230


INV_SAW6:
	; base freqency: 415.30 Hz, discrets: 44, rms: 7.95, min: -1.00, max: 1.00

	.db	  128,   57,   11,    0,    13,   29,   33,   27
	.db	   19,   19,   25,   32,    33,   29,   26,   27
	.db	   32,   36,   36,   33,    32,   34,   38,   40
	.db	   40,   38,   38,   40,    43,   45,   44,   43

	.db	   43,   46,   49,   50,    49,   48,   49,   52
	.db	   54,   54,   53,   53,    55,   57,   59,   59
	.db	   58,   59,   60,   63,    64,   64,   63,   64
	.db	   66,   68,   69,   69,    69,   69,   71,   73

	.db	   74,   74,   74,   75,    77,   78,   79,   79
	.db	   79,   80,   82,   84,    84,   84,   84,   86
	.db	   88,   89,   89,   89,    90,   91,   93,   94
	.db	   94,   94,   95,   97,    98,   99,   99,   99

	.db	  100,  102,  103,  104,   104,  104,  106,  108
	.db	  108,  109,  109,  110,   111,  113,  113,  114
	.db	  114,  115,  117,  118,   119,  119,  119,  120
	.db	  122,  123,  124,  124,   124,  126,  127,  128

	.db	  128,  128,  129,  130,   132,  132,  132,  133
	.db	  134,  136,  137,  137,   137,  138,  139,  141
	.db	  142,  142,  143,  143,   145,  146,  147,  147
	.db	  148,  148,  150,  152,   152,  152,  153,  154

	.db	  156,  157,  157,  157,   158,  159,  161,  162
	.db	  162,  162,  163,  165,   166,  167,  167,  167
	.db	  168,  170,  172,  172,   172,  172,  174,  176
	.db	  177,  177,  177,  178,   179,  181,  182,  182

	.db	  182,  183,  185,  187,   187,  187,  187,  188
	.db	  190,  192,  193,  192,   192,  193,  196,  197
	.db	  198,  197,  197,  199,   201,  203,  203,  202
	.db	  202,  204,  207,  208,   207,  206,  207,  210

	.db	  213,  213,  212,  211,   213,  216,  218,  218
	.db	  216,  216,  218,  222,   224,  223,  220,  220
	.db	  224,  229,  230,  227,   223,  224,  231,  237
	.db	  237,  229,  223,  227,   243,  255,  245,  199


INV_SAW7:
	; base freqency: 659.25 Hz, discrets: 28, rms: 7.95, min: -0.99, max: 0.99

	.db	  128,   81,   41,   14,     2,    3,   12,   23
	.db	   32,   36,   34,   29,    24,   21,   22,   26
	.db	   32,   36,   38,   38,    36,   33,   32,   33
	.db	   36,   40,   43,   44,    44,   43,   42,   41

	.db	   42,   44,   47,   50,    51,   51,   51,   50
	.db	   49,   50,   52,   55,    57,   59,   59,   58
	.db	   58,   58,   59,   61,    63,   65,   66,   66
	.db	   66,   66,   66,   67,    69,   71,   73,   74

	.db	   74,   74,   74,   74,    75,   76,   78,   80
	.db	   81,   82,   82,   81,    82,   83,   84,   86
	.db	   88,   89,   90,   89,    89,   90,   91,   92
	.db	   94,   96,   97,   97,    97,   97,   98,   99

	.db	  100,  102,  104,  105,   105,  105,  105,  105
	.db	  106,  108,  110,  111,   112,  113,  113,  113
	.db	  113,  114,  116,  118,   119,  120,  121,  121
	.db	  121,  121,  122,  124,   125,  127,  128,  128

	.db	  128,  128,  128,  129,   131,  132,  134,  135
	.db	  135,  135,  135,  136,   137,  138,  140,  142
	.db	  143,  143,  143,  143,   144,  145,  146,  148
	.db	  150,  151,  151,  151,   151,  151,  152,  154

	.db	  156,  157,  158,  159,   159,  159,  159,  160
	.db	  162,  164,  165,  166,   167,  167,  166,  167
	.db	  168,  170,  172,  173,   174,  175,  174,  174
	.db	  175,  176,  178,  180,   181,  182,  182,  182

	.db	  182,  182,  183,  185,   187,  189,  190,  190
	.db	  190,  190,  190,  191,   193,  195,  197,  198
	.db	  198,  198,  197,  197,   199,  201,  204,  206
	.db	  207,  206,  205,  205,   205,  206,  209,  212

	.db	  214,  215,  214,  213,   212,  212,  213,  216
	.db	  220,  223,  224,  223,   220,  218,  218,  220
	.db	  224,  230,  234,  235,   232,  227,  222,  220
	.db	  224,  233,  244,  253,   254,  242,  215,  175


INV_SAW8:
	; base freqency: 1046.50 Hz, discrets: 18, rms: 7.95, min: -0.98, max: 0.98

	.db	  128,   97,   68,   43,    24,   11,    4,    3
	.db	    6,   12,   20,   28,    34,   38,   40,   39
	.db	   37,   34,   30,   28,    27,   27,   29,   33
	.db	   36,   40,   43,   46,    47,   47,   46,   44

	.db	   43,   42,   42,   43,    44,   47,   49,   52
	.db	   54,   56,   57,   57,    57,   56,   55,   55
	.db	   55,   56,   58,   60,    62,   64,   66,   68
	.db	   68,   69,   68,   68,    68,   68,   68,   69

	.db	   71,   72,   74,   76,    78,   79,   80,   80
	.db	   80,   80,   80,   80,    81,   82,   83,   85
	.db	   87,   89,   90,   91,    92,   92,   92,   92
	.db	   92,   92,   93,   94,    96,   97,   99,  101

	.db	  102,  103,  104,  104,   104,  104,  104,  105
	.db	  105,  107,  108,  110,   112,  113,  115,  116
	.db	  116,  116,  116,  116,   117,  117,  118,  119
	.db	  121,  122,  124,  126,   127,  128,  128,  128

	.db	  128,  128,  128,  128,   129,  130,  132,  134
	.db	  135,  137,  138,  139,   139,  140,  140,  140
	.db	  140,  140,  141,  143,   144,  146,  148,  149
	.db	  151,  151,  152,  152,   152,  152,  152,  153

	.db	  154,  155,  157,  159,   160,  162,  163,  164
	.db	  164,  164,  164,  164,   164,  165,  166,  167
	.db	  169,  171,  173,  174,   175,  176,  176,  176
	.db	  176,  176,  176,  177,   178,  180,  182,  184

	.db	  185,  187,  188,  188,   188,  188,  188,  187
	.db	  188,  188,  190,  192,   194,  196,  198,  200
	.db	  201,  201,  201,  200,   199,  199,  199,  200
	.db	  202,  204,  207,  209,   212,  213,  214,  214

	.db	  213,  212,  210,  209,   209,  210,  213,  216
	.db	  220,  223,  227,  229,   229,  228,  226,  222
	.db	  219,  217,  216,  218,   222,  228,  236,  244
	.db	  250,  253,  252,  245,   232,  213,  188,  159


INV_SAW9:
	; base freqency: 1661.21 Hz, discrets: 11, rms: 7.95, min: -0.97, max: 0.97

	.db	  128,  109,   90,   73,    56,   42,   30,   20
	.db	   12,    8,    5,    5,     6,    9,   13,   18
	.db	   24,   29,   34,   38,    42,   44,   46,   47
	.db	   46,   45,   44,   42,    41,   39,   38,   37

	.db	   36,   37,   38,   39,    41,   44,   47,   49
	.db	   52,   55,   57,   59,    60,   61,   61,   61
	.db	   61,   61,   60,   59,    59,   59,   59,   59
	.db	   60,   61,   63,   65,    67,   69,   71,   73

	.db	   75,   77,   78,   79,    79,   80,   80,   80
	.db	   80,   79,   79,   79,    79,   80,   80,   82
	.db	   83,   84,   86,   88,    90,   92,   94,   95
	.db	   96,   98,   98,   99,    99,   99,   99,   99

	.db	   99,   99,   99,  100,   100,  101,  103,  104
	.db	  106,  107,  109,  111,   113,  114,  116,  117
	.db	  117,  118,  118,  119,   119,  119,  119,  119
	.db	  119,  119,  120,  121,   122,  123,  125,  127

	.db	  128,  129,  131,  133,   134,  135,  136,  137
	.db	  137,  137,  137,  137,   137,  137,  138,  138
	.db	  139,  139,  140,  142,   143,  145,  147,  149
	.db	  150,  152,  153,  155,   156,  156,  157,  157

	.db	  157,  157,  157,  157,   157,  157,  158,  158
	.db	  160,  161,  162,  164,   166,  168,  170,  172
	.db	  173,  174,  176,  176,   177,  177,  177,  177
	.db	  176,  176,  176,  176,   177,  177,  178,  179

	.db	  181,  183,  185,  187,   189,  191,  193,  195
	.db	  196,  197,  197,  197,   197,  197,  196,  195
	.db	  195,  195,  195,  195,   196,  197,  199,  201
	.db	  204,  207,  209,  212,   215,  217,  218,  219

	.db	  220,  219,  218,  217,   215,  214,  212,  211
	.db	  210,  209,  210,  212,   214,  218,  222,  227
	.db	  232,  238,  243,  247,   250,  251,  251,  248
	.db	  244,  236,  226,  214,   200,  183,  166,  147


INV_SAW10:
	; base freqency: 2637.01 Hz, discrets: 7, rms: 7.95, min: -0.94, max: 0.94

	.db	  128,  116,  104,   91,    80,   69,   58,   49
	.db	   40,   32,   26,   20,    15,   12,   10,    8
	.db	    8,    8,    9,   11,    14,   17,   20,   24
	.db	   28,   32,   35,   39,    42,   46,   48,   51

	.db	   53,   54,   56,   56,    57,   57,   56,   56
	.db	   55,   54,   54,   53,    52,   51,   50,   50
	.db	   50,   50,   51,   51,    52,   53,   55,   57
	.db	   59,   61,   63,   65,    67,   69,   72,   74

	.db	   76,   77,   79,   80,    81,   82,   83,   83
	.db	   83,   83,   83,   83,    83,   83,   83,   82
	.db	   82,   82,   83,   83,    84,   84,   85,   86
	.db	   88,   89,   91,   92,    94,   96,   98,  100

	.db	  102,  103,  105,  107,   108,  109,  110,  111
	.db	  112,  112,  113,  113,   113,  113,  113,  113
	.db	  113,  113,  113,  114,   114,  114,  115,  116
	.db	  116,  118,  119,  120,   122,  123,  125,  127

	.db	  128,  129,  131,  133,   134,  136,  137,  138
	.db	  140,  140,  141,  142,   142,  142,  143,  143
	.db	  143,  143,  143,  143,   143,  143,  143,  144
	.db	  144,  145,  146,  147,   148,  149,  151,  153

	.db	  154,  156,  158,  160,   162,  164,  165,  167
	.db	  168,  170,  171,  172,   172,  173,  173,  174
	.db	  174,  174,  173,  173,   173,  173,  173,  173
	.db	  173,  173,  173,  174,   175,  176,  177,  179

	.db	  180,  182,  184,  187,   189,  191,  193,  195
	.db	  197,  199,  201,  203,   204,  205,  205,  206
	.db	  206,  206,  206,  205,   204,  203,  202,  202
	.db	  201,  200,  200,  199,   199,  200,  200,  202

	.db	  203,  205,  208,  210,   214,  217,  221,  224
	.db	  228,  232,  236,  239,   242,  245,  247,  248
	.db	  248,  248,  246,  244,   241,  236,  230,  224
	.db	  216,  207,  198,  187,   176,  165,  152,  140


INV_SAW11:
	; base freqency: 4185.98 Hz, discrets: 5, rms: 7.95, min: -0.92, max: 0.92

	.db	  128,  119,  110,  101,    93,   84,   76,   68
	.db	   60,   53,   47,   41,    35,   30,   26,   22
	.db	   19,   16,   14,   12,    11,   11,   11,   11
	.db	   12,   14,   16,   18,    20,   23,   26,   29

	.db	   32,   35,   38,   41,    44,   47,   50,   53
	.db	   55,   58,   60,   61,    63,   64,   66,   67
	.db	   67,   68,   68,   68,    68,   68,   68,   67
	.db	   67,   66,   66,   65,    65,   65,   64,   64

	.db	   64,   64,   64,   65,    65,   66,   67,   68
	.db	   69,   70,   72,   73,    75,   77,   79,   81
	.db	   83,   85,   87,   89,    91,   92,   94,   96
	.db	   98,   99,  101,  102,   103,  104,  105,  105

	.db	  106,  107,  107,  107,   107,  107,  108,  108
	.db	  107,  107,  107,  107,   107,  108,  108,  108
	.db	  108,  109,  109,  110,   111,  112,  113,  114
	.db	  115,  117,  118,  120,   121,  123,  125,  127

	.db	  128,  129,  131,  133,   135,  136,  138,  139
	.db	  141,  142,  143,  144,   145,  146,  147,  147
	.db	  148,  148,  148,  148,   149,  149,  149,  149
	.db	  149,  148,  148,  149,   149,  149,  149,  149

	.db	  150,  151,  151,  152,   153,  154,  155,  157
	.db	  158,  160,  162,  164,   165,  167,  169,  171
	.db	  173,  175,  177,  179,   181,  183,  184,  186
	.db	  187,  188,  189,  190,   191,  191,  192,  192

	.db	  192,  192,  192,  191,   191,  191,  190,  190
	.db	  189,  189,  188,  188,   188,  188,  188,  188
	.db	  189,  189,  190,  192,   193,  195,  196,  198
	.db	  201,  203,  206,  209,   212,  215,  218,  221

	.db	  224,  227,  230,  233,   236,  238,  240,  242
	.db	  244,  245,  245,  245,   245,  244,  242,  240
	.db	  237,  234,  230,  226,   221,  215,  209,  203
	.db	  196,  188,  180,  172,   163,  155,  146,  137



;-----------------------------------------------------------------------------
;
;*** Bandlimited square wavetables (each table is 256 bytes long, unsigned integer)
SQ_LIMIT0:
	; base freqency: 25.96 Hz, discrets: 350, rms: 12.65, min: -0.81, max: 0.81

	.db	  128,  230,  231,  229,   228,  229,  230,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  230,  229,   228,  229,  231,  230

	.db	  128,   26,   25,   27,    28,   27,   26,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   26,   27,    28,   27,   25,   26


SQ_LIMIT1:
	; base freqency: 41.20 Hz, discrets: 221, rms: 12.65, min: -0.82, max: 0.82

	.db	  128,  230,  232,  228,   228,  230,  230,  228
	.db	  229,  230,  229,  228,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  228,  229,  230
	.db	  229,  228,  230,  230,   228,  228,  232,  230

	.db	  128,   26,   24,   28,    28,   26,   26,   28
	.db	   27,   26,   27,   28,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   28,   27,   26
	.db	   27,   28,   26,   26,    28,   28,   24,   26


SQ_LIMIT2:
	; base freqency: 65.41 Hz, discrets: 139, rms: 12.65, min: -0.81, max: 0.81

	.db	  128,  221,  227,  229,   230,  231,  231,  230
	.db	  230,  229,  229,  228,   228,  229,  229,  229
	.db	  230,  230,  230,  230,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  230,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  230,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  230,  230,  230
	.db	  230,  229,  229,  229,   228,  228,  229,  229
	.db	  230,  230,  231,  231,   230,  229,  227,  221

	.db	  128,   35,   29,   27,    26,   25,   25,   26
	.db	   26,   27,   27,   28,    28,   27,   27,   27
	.db	   26,   26,   26,   26,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   26,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   26,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   26,   26,   26
	.db	   26,   27,   27,   27,    28,   28,   27,   27
	.db	   26,   26,   25,   25,    26,   27,   29,   35


SQ_LIMIT3:
	; base freqency: 103.83 Hz, discrets: 88, rms: 12.65, min: -0.85, max: 0.85

	.db	  128,  237,  234,  224,   229,  232,  227,  228
	.db	  231,  228,  228,  230,   229,  228,  230,  229
	.db	  228,  229,  230,  228,   229,  230,  228,  229
	.db	  230,  229,  229,  229,   229,  228,  229,  229

	.db	  228,  229,  229,  228,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229

	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  229,  229,  229
	.db	  229,  229,  229,  229,   229,  228,  229,  229

	.db	  228,  229,  229,  228,   229,  229,  229,  229
	.db	  230,  229,  228,  230,   229,  228,  230,  229
	.db	  228,  229,  230,  228,   229,  230,  228,  228
	.db	  231,  228,  227,  232,   229,  224,  234,  237

	.db	  128,   19,   22,   32,    27,   24,   29,   28
	.db	   25,   28,   28,   26,    27,   28,   26,   27
	.db	   28,   27,   26,   28,    27,   26,   28,   27
	.db	   26,   27,   27,   27,    27,   28,   27,   27

	.db	   28,   27,   27,   28,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27

	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   27,   27,   27
	.db	   27,   27,   27,   27,    27,   28,   27,   27

	.db	   28,   27,   27,   28,    27,   27,   27,   27
	.db	   26,   27,   28,   26,    27,   28,   26,   27
	.db	   28,   27,   26,   28,    27,   26,   28,   28
	.db	   25,   28,   29,   24,    27,   32,   22,   19


SQ_LIMIT4:
	; base freqency: 164.81 Hz, discrets: 55, rms: 12.65, min: -0.92, max: 0.92

	.db	  128,  245,  223,  230,   231,  226,  232,  225
	.db	  232,  227,  229,  229,   228,  231,  227,  230
	.db	  228,  229,  229,  228,   230,  228,  230,  228
	.db	  229,  229,  228,  230,   228,  230,  228,  229

	.db	  229,  229,  229,  228,   230,  228,  229,  229
	.db	  229,  229,  228,  230,   228,  229,  229,  229
	.db	  229,  228,  229,  228,   229,  229,  229,  229
	.db	  228,  229,  228,  229,   229,  229,  229,  228

	.db	  229,  228,  229,  229,   229,  229,  228,  229
	.db	  228,  229,  229,  229,   229,  228,  229,  228
	.db	  229,  229,  229,  229,   228,  230,  228,  229
	.db	  229,  229,  229,  228,   230,  228,  229,  229

	.db	  229,  229,  228,  230,   228,  230,  228,  229
	.db	  229,  228,  230,  228,   230,  228,  229,  229
	.db	  228,  230,  227,  231,   228,  229,  229,  227
	.db	  232,  225,  232,  226,   231,  230,  223,  245

	.db	  128,   11,   33,   26,    25,   30,   24,   31
	.db	   24,   29,   27,   27,    28,   25,   29,   26
	.db	   28,   27,   27,   28,    26,   28,   26,   28
	.db	   27,   27,   28,   26,    28,   26,   28,   27

	.db	   27,   27,   27,   28,    26,   28,   27,   27
	.db	   27,   27,   28,   26,    28,   27,   27,   27
	.db	   27,   28,   27,   28,    27,   27,   27,   27
	.db	   28,   27,   28,   27,    27,   27,   27,   28

	.db	   27,   28,   27,   27,    27,   27,   28,   27
	.db	   28,   27,   27,   27,    27,   28,   27,   28
	.db	   27,   27,   27,   27,    28,   26,   28,   27
	.db	   27,   27,   27,   28,    26,   28,   27,   27

	.db	   27,   27,   28,   26,    28,   26,   28,   27
	.db	   27,   28,   26,   28,    26,   28,   27,   27
	.db	   28,   26,   29,   25,    28,   27,   27,   29
	.db	   24,   31,   24,   30,    25,   26,   33,   11


SQ_LIMIT5:
	; base freqency: 261.62 Hz, discrets: 35, rms: 12.65, min: -0.93, max: 0.93

	.db	  128,  222,  246,  226,   221,  233,  233,  225
	.db	  227,  233,  230,  226,   229,  232,  228,  227
	.db	  231,  230,  227,  228,   231,  229,  227,  229
	.db	  231,  228,  228,  230,   230,  228,  229,  230

	.db	  229,  228,  229,  230,   228,  228,  230,  230
	.db	  228,  229,  230,  229,   228,  229,  230,  228
	.db	  228,  230,  229,  228,   229,  230,  229,  228
	.db	  229,  230,  228,  228,   230,  229,  228,  229

	.db	  230,  229,  228,  229,   230,  228,  228,  230
	.db	  229,  228,  229,  230,   229,  228,  229,  230
	.db	  228,  228,  230,  229,   228,  229,  230,  229
	.db	  228,  230,  230,  228,   228,  230,  229,  228

	.db	  229,  230,  229,  228,   230,  230,  228,  228
	.db	  231,  229,  227,  229,   231,  228,  227,  230
	.db	  231,  227,  228,  232,   229,  226,  230,  233
	.db	  227,  225,  233,  233,   221,  226,  246,  222

	.db	  128,   34,   10,   30,    35,   23,   23,   31
	.db	   29,   23,   26,   30,    27,   24,   28,   29
	.db	   25,   26,   29,   28,    25,   27,   29,   27
	.db	   25,   28,   28,   26,    26,   28,   27,   26

	.db	   27,   28,   27,   26,    28,   28,   26,   26
	.db	   28,   27,   26,   27,    28,   27,   26,   28
	.db	   28,   26,   27,   28,    27,   26,   27,   28
	.db	   27,   26,   28,   28,    26,   27,   28,   27

	.db	   26,   27,   28,   27,    26,   28,   28,   26
	.db	   27,   28,   27,   26,    27,   28,   27,   26
	.db	   28,   28,   26,   27,    28,   27,   26,   27
	.db	   28,   26,   26,   28,    28,   26,   27,   28

	.db	   27,   26,   27,   28,    26,   26,   28,   28
	.db	   25,   27,   29,   27,    25,   28,   29,   26
	.db	   25,   29,   28,   24,    27,   30,   26,   23
	.db	   29,   31,   23,   23,    35,   30,   10,   34


SQ_LIMIT6:
	; base freqency: 415.30 Hz, discrets: 22, rms: 12.65, min: -0.94, max: 0.94

	.db	  128,  193,  236,  247,   237,  223,  219,  226
	.db	  234,  236,  231,  225,   224,  228,  233,  233
	.db	  229,  226,  226,  229,   232,  232,  229,  227
	.db	  227,  230,  232,  231,   228,  227,  228,  230

	.db	  231,  230,  228,  227,   228,  230,  231,  230
	.db	  228,  227,  229,  230,   231,  229,  228,  228
	.db	  229,  231,  230,  229,   228,  228,  229,  231
	.db	  230,  229,  228,  228,   230,  231,  230,  228

	.db	  228,  228,  230,  231,   230,  228,  228,  229
	.db	  230,  231,  229,  228,   228,  229,  230,  231
	.db	  229,  228,  228,  229,   231,  230,  229,  227
	.db	  228,  230,  231,  230,   228,  227,  228,  230

	.db	  231,  230,  228,  227,   228,  231,  232,  230
	.db	  227,  227,  229,  232,   232,  229,  226,  226
	.db	  229,  233,  233,  228,   224,  225,  231,  236
	.db	  234,  226,  219,  223,   237,  247,  236,  193

	.db	  128,   63,   20,    9,    19,   33,   37,   30
	.db	   22,   20,   25,   31,    32,   28,   23,   23
	.db	   27,   30,   30,   27,    24,   24,   27,   29
	.db	   29,   26,   24,   25,    28,   29,   28,   26

	.db	   25,   26,   28,   29,    28,   26,   25,   26
	.db	   28,   29,   27,   26,    25,   27,   28,   28
	.db	   27,   25,   26,   27,    28,   28,   27,   25
	.db	   26,   27,   28,   28,    26,   25,   26,   28

	.db	   28,   28,   26,   25,    26,   28,   28,   27
	.db	   26,   25,   27,   28,    28,   27,   26,   25
	.db	   27,   28,   28,   27,    25,   26,   27,   29
	.db	   28,   26,   25,   26,    28,   29,   28,   26

	.db	   25,   26,   28,   29,    28,   25,   24,   26
	.db	   29,   29,   27,   24,    24,   27,   30,   30
	.db	   27,   23,   23,   28,    32,   31,   25,   20
	.db	   22,   30,   37,   33,    19,    9,   20,   63


SQ_LIMIT7:
	; base freqency: 659.25 Hz, discrets: 14, rms: 12.65, min: -0.93, max: 0.93

	.db	  128,  171,  208,  233,   246,  247,  240,  231
	.db	  223,  219,  221,  226,   232,  235,  236,  234
	.db	  230,  226,  224,  225,   227,  231,  233,  234
	.db	  233,  230,  227,  226,   226,  228,  230,  232

	.db	  233,  232,  230,  228,   227,  227,  228,  230
	.db	  231,  232,  232,  230,   228,  227,  227,  228
	.db	  229,  231,  232,  232,   230,  229,  227,  227
	.db	  228,  229,  231,  232,   232,  231,  229,  228

	.db	  227,  228,  229,  231,   232,  232,  231,  229
	.db	  228,  227,  227,  229,   230,  232,  232,  231
	.db	  229,  228,  227,  227,   228,  230,  232,  232
	.db	  231,  230,  228,  227,   227,  228,  230,  232

	.db	  233,  232,  230,  228,   226,  226,  227,  230
	.db	  233,  234,  233,  231,   227,  225,  224,  226
	.db	  230,  234,  236,  235,   232,  226,  221,  219
	.db	  223,  231,  240,  247,   246,  233,  208,  171

	.db	  128,   85,   48,   23,    10,    9,   16,   25
	.db	   33,   37,   35,   30,    24,   21,   20,   22
	.db	   26,   30,   32,   31,    29,   25,   23,   22
	.db	   23,   26,   29,   30,    30,   28,   26,   24

	.db	   23,   24,   26,   28,    29,   29,   28,   26
	.db	   25,   24,   24,   26,    28,   29,   29,   28
	.db	   27,   25,   24,   24,    26,   27,   29,   29
	.db	   28,   27,   25,   24,    24,   25,   27,   28

	.db	   29,   28,   27,   25,    24,   24,   25,   27
	.db	   28,   29,   29,   27,    26,   24,   24,   25
	.db	   27,   28,   29,   29,    28,   26,   24,   24
	.db	   25,   26,   28,   29,    29,   28,   26,   24

	.db	   23,   24,   26,   28,    30,   30,   29,   26
	.db	   23,   22,   23,   25,    29,   31,   32,   30
	.db	   26,   22,   20,   21,    24,   30,   35,   37
	.db	   33,   25,   16,    9,    10,   23,   48,   85


SQ_LIMIT8:
	; base freqency: 1046.50 Hz, discrets: 9, rms: 12.65, min: -0.94, max: 0.94

	.db	  128,  156,  183,  206,   224,  238,  246,  248
	.db	  247,  242,  236,  230,   225,  221,  220,  220
	.db	  223,  226,  230,  233,   236,  237,  237,  235
	.db	  233,  230,  227,  225,   224,  224,  226,  227

	.db	  230,  232,  233,  234,   234,  234,  232,  230
	.db	  228,  227,  226,  226,   226,  228,  229,  231
	.db	  233,  233,  234,  233,   232,  230,  229,  227
	.db	  226,  226,  227,  228,   229,  231,  232,  233

	.db	  233,  233,  232,  231,   229,  228,  227,  226
	.db	  226,  227,  229,  230,   232,  233,  234,  233
	.db	  233,  231,  229,  228,   226,  226,  226,  227
	.db	  228,  230,  232,  234,   234,  234,  233,  232

	.db	  230,  227,  226,  224,   224,  225,  227,  230
	.db	  233,  235,  237,  237,   236,  233,  230,  226
	.db	  223,  220,  220,  221,   225,  230,  236,  242
	.db	  247,  248,  246,  238,   224,  206,  183,  156

	.db	  128,  100,   73,   50,    32,   18,   10,    8
	.db	    9,   14,   20,   26,    31,   35,   36,   36
	.db	   33,   30,   26,   23,    20,   19,   19,   21
	.db	   23,   26,   29,   31,    32,   32,   30,   29

	.db	   26,   24,   23,   22,    22,   22,   24,   26
	.db	   28,   29,   30,   30,    30,   28,   27,   25
	.db	   23,   23,   22,   23,    24,   26,   27,   29
	.db	   30,   30,   29,   28,    27,   25,   24,   23

	.db	   23,   23,   24,   25,    27,   28,   29,   30
	.db	   30,   29,   27,   26,    24,   23,   22,   23
	.db	   23,   25,   27,   28,    30,   30,   30,   29
	.db	   28,   26,   24,   22,    22,   22,   23,   24

	.db	   26,   29,   30,   32,    32,   31,   29,   26
	.db	   23,   21,   19,   19,    20,   23,   26,   30
	.db	   33,   36,   36,   35,    31,   26,   20,   14
	.db	    9,    8,   10,   18,    32,   50,   73,  100


SQ_LIMIT9:
	; base freqency: 1661.21 Hz, discrets: 5, rms: 12.65, min: -0.95, max: 0.95

	.db	  128,  144,  159,  175,   189,  202,  214,  224
	.db	  232,  239,  244,  247,   249,  250,  249,  247
	.db	  244,  241,  237,  234,   230,  227,  225,  222
	.db	  221,  220,  220,  221,   222,  223,  225,  228

	.db	  230,  232,  234,  236,   237,  238,  239,  239
	.db	  238,  237,  236,  234,   233,  231,  229,  227
	.db	  226,  225,  224,  224,   224,  225,  226,  227
	.db	  228,  230,  231,  233,   234,  236,  237,  237

	.db	  237,  237,  237,  236,   234,  233,  231,  230
	.db	  228,  227,  226,  225,   224,  224,  224,  225
	.db	  226,  227,  229,  231,   233,  234,  236,  237
	.db	  238,  239,  239,  238,   237,  236,  234,  232

	.db	  230,  228,  225,  223,   222,  221,  220,  220
	.db	  221,  222,  225,  227,   230,  234,  237,  241
	.db	  244,  247,  249,  250,   249,  247,  244,  239
	.db	  232,  224,  214,  202,   189,  175,  159,  144

	.db	  128,  112,   97,   81,    67,   54,   42,   32
	.db	   24,   17,   12,    9,     7,    6,    7,    9
	.db	   12,   15,   19,   22,    26,   29,   31,   34
	.db	   35,   36,   36,   35,    34,   33,   31,   28

	.db	   26,   24,   22,   20,    19,   18,   17,   17
	.db	   18,   19,   20,   22,    23,   25,   27,   29
	.db	   30,   31,   32,   32,    32,   31,   30,   29
	.db	   28,   26,   25,   23,    22,   20,   19,   19

	.db	   19,   19,   19,   20,    22,   23,   25,   26
	.db	   28,   29,   30,   31,    32,   32,   32,   31
	.db	   30,   29,   27,   25,    23,   22,   20,   19
	.db	   18,   17,   17,   18,    19,   20,   22,   24

	.db	   26,   28,   31,   33,    34,   35,   36,   36
	.db	   35,   34,   31,   29,    26,   22,   19,   15
	.db	   12,    9,    7,    6,     7,    9,   12,   17
	.db	   24,   32,   42,   54,    67,   81,   97,  112


SQ_LIMIT10:
	; base freqency: 2637.01 Hz, discrets: 3, rms: 12.65, min: -0.97, max: 0.97

	.db	  128,  137,  147,  157,   166,  175,  184,  192
	.db	  200,  208,  215,  221,   227,  232,  237,  241
	.db	  244,  247,  249,  251,   252,  252,  252,  251
	.db	  250,  249,  248,  246,   244,  241,  239,  237

	.db	  234,  232,  230,  228,   226,  224,  223,  222
	.db	  221,  220,  220,  220,   220,  221,  221,  222
	.db	  224,  225,  226,  228,   230,  231,  233,  235
	.db	  236,  238,  239,  240,   241,  242,  243,  243

	.db	  243,  243,  243,  242,   241,  240,  239,  238
	.db	  236,  235,  233,  231,   230,  228,  226,  225
	.db	  224,  222,  221,  221,   220,  220,  220,  220
	.db	  221,  222,  223,  224,   226,  228,  230,  232

	.db	  234,  237,  239,  241,   244,  246,  248,  249
	.db	  250,  251,  252,  252,   252,  251,  249,  247
	.db	  244,  241,  237,  232,   227,  221,  215,  208
	.db	  200,  192,  184,  175,   166,  157,  147,  137

	.db	  128,  119,  109,   99,    90,   81,   72,   64
	.db	   56,   48,   41,   35,    29,   24,   19,   15
	.db	   12,    9,    7,    5,     4,    4,    4,    5
	.db	    6,    7,    8,   10,    12,   15,   17,   19

	.db	   22,   24,   26,   28,    30,   32,   33,   34
	.db	   35,   36,   36,   36,    36,   35,   35,   34
	.db	   32,   31,   30,   28,    26,   25,   23,   21
	.db	   20,   18,   17,   16,    15,   14,   13,   13

	.db	   13,   13,   13,   14,    15,   16,   17,   18
	.db	   20,   21,   23,   25,    26,   28,   30,   31
	.db	   32,   34,   35,   35,    36,   36,   36,   36
	.db	   35,   34,   33,   32,    30,   28,   26,   24

	.db	   22,   19,   17,   15,    12,   10,    8,    7
	.db	    6,    5,    4,    4,     4,    5,    7,    9
	.db	   12,   15,   19,   24,    29,   35,   41,   48
	.db	   56,   64,   72,   81,    90,   99,  109,  119


SQ_LIMIT11:
	; base freqency: 4185.98 Hz, discrets: 2, rms: 12.65, min: -1.00, max: 1.00

	.db	  128,  134,  141,  147,   154,  160,  167,  173
	.db	  179,  185,  191,  196,   202,  207,  212,  217
	.db	  221,  225,  229,  233,   237,  240,  242,  245
	.db	  247,  249,  251,  252,   254,  254,  255,  255

	.db	  255,  255,  255,  255,   254,  253,  252,  251
	.db	  249,  248,  246,  245,   243,  241,  239,  237
	.db	  236,  234,  232,  230,   229,  227,  226,  224
	.db	  223,  222,  221,  220,   219,  219,  218,  218

	.db	  218,  218,  218,  219,   219,  220,  221,  222
	.db	  223,  224,  226,  227,   229,  230,  232,  234
	.db	  236,  237,  239,  241,   243,  245,  246,  248
	.db	  249,  251,  252,  253,   254,  255,  255,  255

	.db	  255,  255,  255,  254,   254,  252,  251,  249
	.db	  247,  245,  242,  240,   237,  233,  229,  225
	.db	  221,  217,  212,  207,   202,  196,  191,  185
	.db	  179,  173,  167,  160,   154,  147,  141,  134

	.db	  128,  122,  115,  109,   102,   96,   89,   83
	.db	   77,   71,   65,   60,    54,   49,   44,   39
	.db	   35,   31,   27,   23,    19,   16,   14,   11
	.db	    9,    7,    5,    4,     2,    2,    1,    1

	.db	    0,    1,    1,    1,     2,    3,    4,    5
	.db	    7,    8,   10,   11,    13,   15,   17,   19
	.db	   20,   22,   24,   26,    27,   29,   30,   32
	.db	   33,   34,   35,   36,    37,   37,   38,   38

	.db	   38,   38,   38,   37,    37,   36,   35,   34
	.db	   33,   32,   30,   29,    27,   26,   24,   22
	.db	   20,   19,   17,   15,    13,   11,   10,    8
	.db	    7,    5,    4,    3,     2,    1,    1,    1

	.db	    0,    1,    1,    2,     2,    4,    5,    7
	.db	    9,   11,   14,   16,    19,   23,   27,   31
	.db	   35,   39,   44,   49,    54,   60,   65,   71
	.db	   77,   83,   89,   96,   102,  109,  115,  122



;-----------------------------------------------------------------------------
;
;*** Bandlimited triangle wavetables (each table is 256 bytes long, unsigned integer)
TRI_LIMIT0:
	; base freqency: 25.96 Hz, discrets: 350, rms: 9.24, min: -1.00, max: 1.00

	.db	  128,  126,  124,  122,   120,  118,  116,  114
	.db	  112,  110,  108,  106,   104,  102,  100,   98
	.db	   96,   94,   92,   90,    88,   86,   84,   82
	.db	   80,   78,   76,   74,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    56,   54,   52,   50
	.db	   48,   46,   44,   42,    40,   38,   36,   34
	.db	   32,   30,   28,   26,    24,   22,   20,   18
	.db	   16,   14,   12,   10,     8,    6,    4,    2

	.db	    0,    2,    4,    6,     8,   10,   12,   14
	.db	   16,   18,   20,   22,    24,   26,   28,   30
	.db	   32,   34,   36,   38,    40,   42,   44,   46
	.db	   48,   50,   52,   54,    56,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   74,   76,   78
	.db	   80,   82,   84,   86,    88,   90,   92,   94
	.db	   96,   98,  100,  102,   104,  106,  108,  110
	.db	  112,  114,  116,  118,   120,  122,  124,  126

	.db	  128,  130,  132,  134,   136,  138,  140,  142
	.db	  144,  146,  148,  150,   152,  154,  156,  158
	.db	  160,  162,  164,  166,   168,  170,  172,  174
	.db	  176,  178,  180,  182,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   200,  202,  204,  206
	.db	  208,  210,  212,  214,   216,  218,  220,  222
	.db	  224,  226,  228,  230,   232,  234,  236,  238
	.db	  240,  242,  244,  246,   248,  250,  252,  254

	.db	  255,  254,  252,  250,   248,  246,  244,  242
	.db	  240,  238,  236,  234,   232,  230,  228,  226
	.db	  224,  222,  220,  218,   216,  214,  212,  210
	.db	  208,  206,  204,  202,   200,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  182,  180,  178
	.db	  176,  174,  172,  170,   168,  166,  164,  162
	.db	  160,  158,  156,  154,   152,  150,  148,  146
	.db	  144,  142,  140,  138,   136,  134,  132,  130


TRI_LIMIT1:
	; base freqency: 41.20 Hz, discrets: 221, rms: 9.24, min: -1.00, max: 1.00

	.db	  128,  126,  124,  122,   120,  118,  116,  114
	.db	  112,  110,  108,  106,   104,  102,  100,   98
	.db	   96,   94,   92,   90,    88,   86,   84,   82
	.db	   80,   78,   76,   74,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    56,   54,   52,   50
	.db	   48,   46,   44,   42,    40,   38,   36,   34
	.db	   32,   30,   28,   26,    24,   22,   20,   18
	.db	   16,   14,   12,   10,     8,    6,    4,    2

	.db	    1,    2,    4,    6,     8,   10,   12,   14
	.db	   16,   18,   20,   22,    24,   26,   28,   30
	.db	   32,   34,   36,   38,    40,   42,   44,   46
	.db	   48,   50,   52,   54,    56,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   74,   76,   78
	.db	   80,   82,   84,   86,    88,   90,   92,   94
	.db	   96,   98,  100,  102,   104,  106,  108,  110
	.db	  112,  114,  116,  118,   120,  122,  124,  126

	.db	  128,  130,  132,  134,   136,  138,  140,  142
	.db	  144,  146,  148,  150,   152,  154,  156,  158
	.db	  160,  162,  164,  166,   168,  170,  172,  174
	.db	  176,  178,  180,  182,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   200,  202,  204,  206
	.db	  208,  210,  212,  214,   216,  218,  220,  222
	.db	  224,  226,  228,  230,   232,  234,  236,  238
	.db	  240,  242,  244,  246,   248,  250,  252,  254

	.db	  255,  254,  252,  250,   248,  246,  244,  242
	.db	  240,  238,  236,  234,   232,  230,  228,  226
	.db	  224,  222,  220,  218,   216,  214,  212,  210
	.db	  208,  206,  204,  202,   200,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  182,  180,  178
	.db	  176,  174,  172,  170,   168,  166,  164,  162
	.db	  160,  158,  156,  154,   152,  150,  148,  146
	.db	  144,  142,  140,  138,   136,  134,  132,  130


TRI_LIMIT2:
	; base freqency: 65.41 Hz, discrets: 139, rms: 9.24, min: -1.00, max: 1.00

	.db	  128,  126,  124,  122,   120,  118,  116,  114
	.db	  112,  110,  108,  106,   104,  102,  100,   98
	.db	   96,   94,   92,   90,    88,   86,   84,   82
	.db	   80,   78,   76,   74,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    56,   54,   52,   50
	.db	   48,   46,   44,   42,    40,   38,   36,   34
	.db	   32,   30,   28,   26,    24,   22,   20,   18
	.db	   16,   14,   12,   10,     8,    6,    4,    2

	.db	    1,    2,    4,    6,     8,   10,   12,   14
	.db	   16,   18,   20,   22,    24,   26,   28,   30
	.db	   32,   34,   36,   38,    40,   42,   44,   46
	.db	   48,   50,   52,   54,    56,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   74,   76,   78
	.db	   80,   82,   84,   86,    88,   90,   92,   94
	.db	   96,   98,  100,  102,   104,  106,  108,  110
	.db	  112,  114,  116,  118,   120,  122,  124,  126

	.db	  128,  130,  132,  134,   136,  138,  140,  142
	.db	  144,  146,  148,  150,   152,  154,  156,  158
	.db	  160,  162,  164,  166,   168,  170,  172,  174
	.db	  176,  178,  180,  182,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   200,  202,  204,  206
	.db	  208,  210,  212,  214,   216,  218,  220,  222
	.db	  224,  226,  228,  230,   232,  234,  236,  238
	.db	  240,  242,  244,  246,   248,  250,  252,  254

	.db	  255,  254,  252,  250,   248,  246,  244,  242
	.db	  240,  238,  236,  234,   232,  230,  228,  226
	.db	  224,  222,  220,  218,   216,  214,  212,  210
	.db	  208,  206,  204,  202,   200,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  182,  180,  178
	.db	  176,  174,  172,  170,   168,  166,  164,  162
	.db	  160,  158,  156,  154,   152,  150,  148,  146
	.db	  144,  142,  140,  138,   136,  134,  132,  130


TRI_LIMIT3:
	; base freqency: 103.83 Hz, discrets: 88, rms: 9.24, min: -1.00, max: 1.00

	.db	  128,  126,  124,  122,   120,  118,  116,  114
	.db	  112,  110,  108,  106,   104,  102,  100,   98
	.db	   96,   94,   92,   90,    88,   86,   84,   82
	.db	   80,   78,   76,   74,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    56,   54,   52,   50
	.db	   48,   46,   44,   42,    40,   38,   36,   34
	.db	   32,   30,   28,   26,    24,   22,   20,   18
	.db	   16,   14,   12,   10,     8,    6,    4,    2

	.db	    1,    2,    4,    6,     8,   10,   12,   14
	.db	   16,   18,   20,   22,    24,   26,   28,   30
	.db	   32,   34,   36,   38,    40,   42,   44,   46
	.db	   48,   50,   52,   54,    56,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   74,   76,   78
	.db	   80,   82,   84,   86,    88,   90,   92,   94
	.db	   96,   98,  100,  102,   104,  106,  108,  110
	.db	  112,  114,  116,  118,   120,  122,  124,  126

	.db	  128,  130,  132,  134,   136,  138,  140,  142
	.db	  144,  146,  148,  150,   152,  154,  156,  158
	.db	  160,  162,  164,  166,   168,  170,  172,  174
	.db	  176,  178,  180,  182,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   200,  202,  204,  206
	.db	  208,  210,  212,  214,   216,  218,  220,  222
	.db	  224,  226,  228,  230,   232,  234,  236,  238
	.db	  240,  242,  244,  246,   248,  250,  252,  254

	.db	  255,  254,  252,  250,   248,  246,  244,  242
	.db	  240,  238,  236,  234,   232,  230,  228,  226
	.db	  224,  222,  220,  218,   216,  214,  212,  210
	.db	  208,  206,  204,  202,   200,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  182,  180,  178
	.db	  176,  174,  172,  170,   168,  166,  164,  162
	.db	  160,  158,  156,  154,   152,  150,  148,  146
	.db	  144,  142,  140,  138,   136,  134,  132,  130


TRI_LIMIT4:
	; base freqency: 164.81 Hz, discrets: 55, rms: 9.24, min: -1.00, max: 1.00

	.db	  128,  126,  125,  122,   120,  118,  116,  114
	.db	  112,  110,  108,  106,   104,  102,  100,   98
	.db	   96,   94,   92,   90,    88,   86,   84,   82
	.db	   80,   78,   76,   74,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    56,   54,   52,   50
	.db	   48,   46,   44,   42,    40,   38,   36,   34
	.db	   32,   30,   28,   26,    24,   22,   20,   18
	.db	   16,   14,   12,   10,     8,    6,    4,    2

	.db	    1,    2,    4,    6,     8,   10,   12,   14
	.db	   16,   18,   20,   22,    24,   26,   28,   30
	.db	   32,   34,   36,   38,    40,   42,   44,   46
	.db	   48,   50,   52,   54,    56,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   74,   76,   78
	.db	   80,   82,   84,   86,    88,   90,   92,   94
	.db	   96,   98,  100,  102,   104,  106,  108,  110
	.db	  112,  114,  116,  118,   120,  122,  125,  126

	.db	  128,  130,  131,  134,   136,  138,  140,  142
	.db	  144,  146,  148,  150,   152,  154,  156,  158
	.db	  160,  162,  164,  166,   168,  170,  172,  174
	.db	  176,  178,  180,  182,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   200,  202,  204,  206
	.db	  208,  210,  212,  214,   216,  218,  220,  222
	.db	  224,  226,  228,  230,   232,  234,  236,  238
	.db	  240,  242,  244,  246,   248,  250,  252,  254

	.db	  255,  254,  252,  250,   248,  246,  244,  242
	.db	  240,  238,  236,  234,   232,  230,  228,  226
	.db	  224,  222,  220,  218,   216,  214,  212,  210
	.db	  208,  206,  204,  202,   200,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  182,  180,  178
	.db	  176,  174,  172,  170,   168,  166,  164,  162
	.db	  160,  158,  156,  154,   152,  150,  148,  146
	.db	  144,  142,  140,  138,   136,  134,  131,  130


TRI_LIMIT5:
	; base freqency: 261.62 Hz, discrets: 35, rms: 9.24, min: -0.99, max: 0.99

	.db	  128,  126,  125,  123,   120,  118,  117,  114
	.db	  112,  110,  108,  106,   104,  102,  100,   98
	.db	   96,   94,   92,   90,    88,   86,   84,   82
	.db	   80,   78,   76,   74,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    56,   54,   52,   50
	.db	   48,   46,   44,   42,    40,   38,   36,   34
	.db	   32,   30,   28,   26,    24,   22,   20,   18
	.db	   16,   14,   12,   10,     8,    7,    4,    2

	.db	    1,    2,    4,    7,     8,   10,   12,   14
	.db	   16,   18,   20,   22,    24,   26,   28,   30
	.db	   32,   34,   36,   38,    40,   42,   44,   46
	.db	   48,   50,   52,   54,    56,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   74,   76,   78
	.db	   80,   82,   84,   86,    88,   90,   92,   94
	.db	   96,   98,  100,  102,   104,  106,  108,  110
	.db	  112,  114,  117,  118,   120,  123,  125,  126

	.db	  128,  130,  131,  133,   136,  138,  139,  142
	.db	  144,  146,  148,  150,   152,  154,  156,  158
	.db	  160,  162,  164,  166,   168,  170,  172,  174
	.db	  176,  178,  180,  182,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   200,  202,  204,  206
	.db	  208,  210,  212,  214,   216,  218,  220,  222
	.db	  224,  226,  228,  230,   232,  234,  236,  238
	.db	  240,  242,  244,  246,   248,  249,  252,  254

	.db	  255,  254,  252,  249,   248,  246,  244,  242
	.db	  240,  238,  236,  234,   232,  230,  228,  226
	.db	  224,  222,  220,  218,   216,  214,  212,  210
	.db	  208,  206,  204,  202,   200,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  182,  180,  178
	.db	  176,  174,  172,  170,   168,  166,  164,  162
	.db	  160,  158,  156,  154,   152,  150,  148,  146
	.db	  144,  142,  139,  138,   136,  133,  131,  130


TRI_LIMIT6:
	; base freqency: 415.30 Hz, discrets: 22, rms: 9.24, min: -0.99, max: 0.99

	.db	  128,  127,  125,  122,   120,  118,  116,  115
	.db	  113,  110,  108,  106,   104,  103,  100,   98
	.db	   96,   94,   92,   91,    88,   86,   84,   82
	.db	   80,   78,   76,   74,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    56,   54,   52,   50
	.db	   48,   46,   44,   42,    40,   38,   36,   34
	.db	   33,   30,   28,   26,    24,   23,   21,   18
	.db	   16,   14,   12,   11,     9,    6,    4,    2

	.db	    2,    2,    4,    6,     9,   11,   12,   14
	.db	   16,   18,   21,   23,    24,   26,   28,   30
	.db	   33,   34,   36,   38,    40,   42,   44,   46
	.db	   48,   50,   52,   54,    56,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   74,   76,   78
	.db	   80,   82,   84,   86,    88,   91,   92,   94
	.db	   96,   98,  100,  103,   104,  106,  108,  110
	.db	  113,  115,  116,  118,   120,  122,  125,  127

	.db	  128,  129,  131,  134,   136,  138,  140,  141
	.db	  143,  146,  148,  150,   152,  153,  156,  158
	.db	  160,  162,  164,  165,   168,  170,  172,  174
	.db	  176,  178,  180,  182,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   200,  202,  204,  206
	.db	  208,  210,  212,  214,   216,  218,  220,  222
	.db	  223,  226,  228,  230,   232,  233,  235,  238
	.db	  240,  242,  244,  245,   247,  250,  252,  254

	.db	  254,  254,  252,  250,   247,  245,  244,  242
	.db	  240,  238,  235,  233,   232,  230,  228,  226
	.db	  223,  222,  220,  218,   216,  214,  212,  210
	.db	  208,  206,  204,  202,   200,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  182,  180,  178
	.db	  176,  174,  172,  170,   168,  165,  164,  162
	.db	  160,  158,  156,  153,   152,  150,  148,  146
	.db	  143,  141,  140,  138,   136,  134,  131,  129


TRI_LIMIT7:
	; base freqency: 659.25 Hz, discrets: 14, rms: 9.24, min: -0.99, max: 0.99

	.db	  128,  127,  125,  123,   121,  118,  116,  114
	.db	  112,  110,  109,  107,   105,  103,  100,   98
	.db	   96,   94,   92,   91,    89,   87,   85,   82
	.db	   80,   78,   76,   74,    73,   71,   69,   67

	.db	   64,   62,   60,   58,    56,   54,   53,   51
	.db	   49,   46,   44,   42,    40,   38,   36,   35
	.db	   33,   31,   28,   26,    24,   22,   20,   19
	.db	   17,   15,   13,   10,     8,    6,    4,    3

	.db	    2,    3,    4,    6,     8,   10,   13,   15
	.db	   17,   19,   20,   22,    24,   26,   28,   31
	.db	   33,   35,   36,   38,    40,   42,   44,   46
	.db	   49,   51,   53,   54,    56,   58,   60,   62

	.db	   64,   67,   69,   71,    73,   74,   76,   78
	.db	   80,   82,   85,   87,    89,   91,   92,   94
	.db	   96,   98,  100,  103,   105,  107,  109,  110
	.db	  112,  114,  116,  118,   121,  123,  125,  127

	.db	  128,  129,  131,  133,   135,  138,  140,  142
	.db	  144,  146,  147,  149,   151,  153,  156,  158
	.db	  160,  162,  164,  165,   167,  169,  171,  174
	.db	  176,  178,  180,  182,   183,  185,  187,  189

	.db	  192,  194,  196,  198,   200,  202,  203,  205
	.db	  207,  210,  212,  214,   216,  218,  220,  221
	.db	  223,  225,  228,  230,   232,  234,  236,  237
	.db	  239,  241,  243,  246,   248,  250,  252,  253

	.db	  254,  253,  252,  250,   248,  246,  243,  241
	.db	  239,  237,  236,  234,   232,  230,  228,  225
	.db	  223,  221,  220,  218,   216,  214,  212,  210
	.db	  207,  205,  203,  202,   200,  198,  196,  194

	.db	  192,  189,  187,  185,   183,  182,  180,  178
	.db	  176,  174,  171,  169,   167,  165,  164,  162
	.db	  160,  158,  156,  153,   151,  149,  147,  146
	.db	  144,  142,  140,  138,   135,  133,  131,  129


TRI_LIMIT8:
	; base freqency: 1046.50 Hz, discrets: 9, rms: 9.24, min: -0.98, max: 0.98

	.db	  128,  126,  124,  122,   120,  118,  116,  114
	.db	  113,  111,  109,  107,   105,  103,  100,   98
	.db	   96,   94,   92,   90,    88,   86,   85,   83
	.db	   81,   79,   77,   75,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    57,   55,   53,   51
	.db	   49,   47,   44,   42,    40,   38,   36,   34
	.db	   32,   30,   29,   27,    25,   23,   21,   19
	.db	   16,   14,   12,    9,     7,    6,    4,    4

	.db	    3,    4,    4,    6,     7,    9,   12,   14
	.db	   16,   19,   21,   23,    25,   27,   29,   30
	.db	   32,   34,   36,   38,    40,   42,   44,   47
	.db	   49,   51,   53,   55,    57,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   75,   77,   79
	.db	   81,   83,   85,   86,    88,   90,   92,   94
	.db	   96,   98,  100,  103,   105,  107,  109,  111
	.db	  113,  114,  116,  118,   120,  122,  124,  126

	.db	  128,  130,  132,  134,   136,  138,  140,  142
	.db	  143,  145,  147,  149,   151,  153,  156,  158
	.db	  160,  162,  164,  166,   168,  170,  171,  173
	.db	  175,  177,  179,  181,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   199,  201,  203,  205
	.db	  207,  209,  212,  214,   216,  218,  220,  222
	.db	  224,  226,  227,  229,   231,  233,  235,  237
	.db	  240,  242,  244,  247,   249,  250,  252,  252

	.db	  253,  252,  252,  250,   249,  247,  244,  242
	.db	  240,  237,  235,  233,   231,  229,  227,  226
	.db	  224,  222,  220,  218,   216,  214,  212,  209
	.db	  207,  205,  203,  201,   199,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  181,  179,  177
	.db	  175,  173,  171,  170,   168,  166,  164,  162
	.db	  160,  158,  156,  153,   151,  149,  147,  145
	.db	  143,  142,  140,  138,   136,  134,  132,  130


TRI_LIMIT9:
	; base freqency: 1661.21 Hz, discrets: 5, rms: 9.24, min: -0.96, max: 0.96

	.db	  128,  126,  124,  122,   120,  118,  116,  114
	.db	  112,  110,  108,  106,   104,  103,  101,   99
	.db	   97,   95,   93,   91,    89,   87,   85,   83
	.db	   81,   78,   76,   74,    72,   70,   68,   66

	.db	   64,   62,   60,   58,    56,   54,   53,   51
	.db	   49,   47,   45,   43,    41,   39,   37,   35
	.db	   33,   31,   28,   26,    24,   21,   19,   17
	.db	   15,   13,   11,    9,     8,    7,    6,    6

	.db	    6,    6,    6,    7,     8,    9,   11,   13
	.db	   15,   17,   19,   21,    24,   26,   28,   31
	.db	   33,   35,   37,   39,    41,   43,   45,   47
	.db	   49,   51,   53,   54,    56,   58,   60,   62

	.db	   64,   66,   68,   70,    72,   74,   76,   78
	.db	   81,   83,   85,   87,    89,   91,   93,   95
	.db	   97,   99,  101,  103,   104,  106,  108,  110
	.db	  112,  114,  116,  118,   120,  122,  124,  126

	.db	  128,  130,  132,  134,   136,  138,  140,  142
	.db	  144,  146,  148,  150,   152,  153,  155,  157
	.db	  159,  161,  163,  165,   167,  169,  171,  173
	.db	  175,  178,  180,  182,   184,  186,  188,  190

	.db	  192,  194,  196,  198,   200,  202,  203,  205
	.db	  207,  209,  211,  213,   215,  217,  219,  221
	.db	  223,  225,  228,  230,   232,  235,  237,  239
	.db	  241,  243,  245,  247,   248,  249,  250,  250

	.db	  250,  250,  250,  249,   248,  247,  245,  243
	.db	  241,  239,  237,  235,   232,  230,  228,  225
	.db	  223,  221,  219,  217,   215,  213,  211,  209
	.db	  207,  205,  203,  202,   200,  198,  196,  194

	.db	  192,  190,  188,  186,   184,  182,  180,  178
	.db	  175,  173,  171,  169,   167,  165,  163,  161
	.db	  159,  157,  155,  153,   152,  150,  148,  146
	.db	  144,  142,  140,  138,   136,  134,  132,  130


TRI_LIMIT10:
	; base freqency: 2637.01 Hz, discrets: 3, rms: 9.24, min: -0.93, max: 0.93

	.db	  128,  126,  124,  122,   120,  118,  115,  113
	.db	  111,  109,  107,  105,   103,  101,   99,   97
	.db	   96,   94,   92,   90,    88,   87,   85,   83
	.db	   81,   80,   78,   76,    74,   72,   70,   68

	.db	   66,   64,   62,   60,    58,   55,   53,   51
	.db	   48,   46,   44,   41,    39,   37,   34,   32
	.db	   30,   28,   25,   23,    21,   20,   18,   16
	.db	   15,   13,   12,   11,    10,   10,    9,    9

	.db	    9,    9,    9,   10,    10,   11,   12,   13
	.db	   15,   16,   18,   20,    21,   23,   25,   28
	.db	   30,   32,   34,   37,    39,   41,   44,   46
	.db	   48,   51,   53,   55,    58,   60,   62,   64

	.db	   66,   68,   70,   72,    74,   76,   78,   80
	.db	   81,   83,   85,   87,    88,   90,   92,   94
	.db	   96,   97,   99,  101,   103,  105,  107,  109
	.db	  111,  113,  115,  118,   120,  122,  124,  126

	.db	  128,  130,  132,  134,   136,  138,  141,  143
	.db	  145,  147,  149,  151,   153,  155,  157,  159
	.db	  160,  162,  164,  166,   168,  169,  171,  173
	.db	  175,  176,  178,  180,   182,  184,  186,  188

	.db	  190,  192,  194,  196,   198,  201,  203,  205
	.db	  208,  210,  212,  215,   217,  219,  222,  224
	.db	  226,  228,  231,  233,   235,  236,  238,  240
	.db	  241,  243,  244,  245,   246,  246,  247,  247

	.db	  247,  247,  247,  246,   246,  245,  244,  243
	.db	  241,  240,  238,  236,   235,  233,  231,  228
	.db	  226,  224,  222,  219,   217,  215,  212,  210
	.db	  208,  205,  203,  201,   198,  196,  194,  192

	.db	  190,  188,  186,  184,   182,  180,  178,  176
	.db	  175,  173,  171,  169,   168,  166,  164,  162
	.db	  160,  159,  157,  155,   153,  151,  149,  147
	.db	  145,  143,  141,  138,   136,  134,  132,  130


TRI_LIMIT11:
	; base freqency: 4185.98 Hz, discrets: 2, rms: 9.24, min: -0.90, max: 0.90

	.db	  128,  127,  125,  123,   122,  120,  118,  116
	.db	  115,  113,  111,  109,   107,  105,  103,  101
	.db	   99,   97,   95,   93,    91,   89,   87,   84
	.db	   82,   80,   77,   75,    73,   70,   68,   66

	.db	   63,   61,   58,   56,    54,   51,   49,   47
	.db	   44,   42,   40,   38,    36,   34,   32,   30
	.db	   28,   26,   25,   23,    22,   20,   19,   18
	.db	   17,   16,   15,   15,    14,   14,   13,   13

	.db	   13,   13,   13,   14,    14,   15,   15,   16
	.db	   17,   18,   19,   20,    22,   23,   25,   26
	.db	   28,   30,   32,   34,    36,   38,   40,   42
	.db	   44,   47,   49,   51,    54,   56,   58,   61

	.db	   63,   66,   68,   70,    73,   75,   77,   80
	.db	   82,   84,   87,   89,    91,   93,   95,   97
	.db	   99,  101,  103,  105,   107,  109,  111,  113
	.db	  115,  116,  118,  120,   122,  123,  125,  127

	.db	  128,  129,  131,  133,   134,  136,  138,  140
	.db	  141,  143,  145,  147,   149,  151,  153,  155
	.db	  157,  159,  161,  163,   165,  167,  169,  172
	.db	  174,  176,  179,  181,   183,  186,  188,  190

	.db	  193,  195,  198,  200,   202,  205,  207,  209
	.db	  212,  214,  216,  218,   220,  222,  224,  226
	.db	  228,  230,  231,  233,   234,  236,  237,  238
	.db	  239,  240,  241,  241,   242,  242,  243,  243

	.db	  243,  243,  243,  242,   242,  241,  241,  240
	.db	  239,  238,  237,  236,   234,  233,  231,  230
	.db	  228,  226,  224,  222,   220,  218,  216,  214
	.db	  212,  209,  207,  205,   202,  200,  198,  195

	.db	  193,  190,  188,  186,   183,  181,  179,  176
	.db	  174,  172,  169,  167,   165,  163,  161,  159
	.db	  157,  155,  153,  151,   149,  147,  145,  143
	.db	  141,  140,  138,  136,   134,  133,  131,  129

;-----------------------------------------------------------------------------
;
;-----------------------------------------------------------------------------
.if USE_UART

debug_entry:
	push	r17
	push	r16

	call	usart_init_default_baudrate	; set baudrate to 38400
	pop	r16
	pop	r17
	ret

; -------------
; reinitialize UART for midi
debug_exit:
	push	r16
debug_byte_midi:
	sbis	UCSRA, UDRE		; before change the baudrate wait for transmitter ready
	rjmp	debug_byte_midi

	ldi	r16, high((cpu_frequency / (baud_rate * 16)) - 1)
	out	UBRRH, r16
	ldi	r16, low((cpu_frequency / (baud_rate * 16)) - 1)
	out	UBRRL, r16

; enable receiver and receiver interrupt
	ldi	r16, (1<<RXCIE)|(1<<RXEN);\
	out	UCR, r16		;/ RXCIE=1, RXEN=1
	pop	r16
	ret

; -------------
debug_byte:
	push	r16

	call	usart_write_byte
	call	usart_write_crlf

	pop	r16
	ret
;-----------------------------------------------------------------------------
;
;-----------------------------------------------------------------------------

startup_message:
	.db	0x0d, 0x0a, 0x0d, 0x0a
	.db	"aweMeeblip-SE V2.04 (c) AWe 2012", 0x0d, 0x0a
	.db	"   build ", __DATE__, " ", __TIME__, " ", 0x0d, 0x0a

.if MEEBLIP_SE_ORIGINAL_CODE
  .if MEEBLIP_SE_OPIMIZED_CODE
	.db	"* using optimized meeblib code", 0x0d, 0x0a
  .else	; MEEBLIP_SE_OPIMIZED_CODE
	.db	"* using meeblip original code", 0x0d, 0x0a
  .endif	; MEEBLIP_SE_OPIMIZED_CODE
.endif	; MEEBLIP_SE_ORIGINAL_CODE

.if SKIP_FILTER
	.db	"* filter section disabled", 0x0d, 0x0a
.endif	; SKIP_FILTER

.if SKIP_AMP
	.db	"* amplifier section disabled", 0x0d, 0x0a
.endif	; SKIP_AMP

;.if MEEBLIP_V1_27_HW			; 0: use my cloned&modified pcb version 1.27
;	.db	"* ", 0x0d, 0x0a
;.else	; MEEBLIP_V1_27_HW
;	.db	"* ", 0x0d, 0x0a
;.endif	; MEEBLIP_V1_27_HW

.if KEEP_BUGS
	.db	"* keep bugs for meeblip v2.04 compability", 0x0d, 0x0a
.endif	; KEEP_BUGS

.if USE_ORIGINAL_DCO_FM				; 0: use switch for transpose
	.db	"* using original dco fm ", 0x0d, 0x0a
.else	; USE_ORIGINAL_DCO_FM
	.db	"* dco fm section removed", 0x0d, 0x0a
.endif	; USE_ORIGINAL_DCO_FM

.if USE_ORIGINAL_MIXER			; 0: use alternative mixer
	.db	"* using original mixer", 0x0d, 0x0a
.else	; USE_ORIGINAL_MIXER
	.db	"* using modified mixer", 0x0d, 0x0a
.endif	; USE_ORIGINAL_MIXER

.if USE_ORIGINAL_NOISEGEN
	.db	"* using original noise generator", 0x0d, 0x0a
.else	; USE_ORIGINAL_NOISEGEN
	.db	"* using modified noise generator", 0x0d, 0x0a
.endif	; USE_ORIGINAL_NOISEGEN

.if USE_ORIGINAL_SWITCH_MAP
	.db	"* using original switch mapping ", 0x0d, 0x0a
.else	; USE_ORIGINAL_SWITCH_MAP
	.db	"* using modified switch mapping ", 0x0d, 0x0a
.endif	; USE_ORIGINAL_SWITCH_MAP

.if USE_ORIGINAL_VCFENVMOD
	.db	"* using orignal VCFENVMOD ", 0x0d, 0x0a
.else	; USE_ORIGINAL_VCFENVMOD
	.db	"* using modified VCFENVMOD", 0x0d, 0x0a
.endif	; USE_ORIGINAL_VCFENVMOD

.if !USE_ORIGINAL_RAW_OSC				; 0: use switch for Ringmodulator enable, set SW_ANTI_ALIAS to 1
	.db	"* remove raw waveform generators", 0x0d, 0x0a
.endif	; !USE_ORIGINAL_RAW_OSC

.if USE_RINGMODULATOR
	.db	"* using ring modulator", 0x0d, 0x0a
.endif	; USE_RINGMODULATOR

	.db	0, 0	; end of string

;-----------------------------------------------------------------------------
;
;-----------------------------------------------------------------------------

.MACRO print_string	;  <variable>
	ldi	zl, low( @0*2);	; load z pointer
	ldi	zh, high( @0*2);
	call	usart_string
.ENDMACRO

.MACRO print_value	;  <variable>
	lds	r16, @0
	call	usart_write_byte
.ENDMACRO

.MACRO print_variable	;  <variable>
	print_string	STR_@0
	print_value	@0
.ENDMACRO

; -------------
usart_print_sound_parameter:
; lower knobs
	print_string	STR_LINE
	print_string	STR_LOWER

	print_variable	RESONANCE
	print_variable	CUTOFF
	print_variable	LFOFREQ
	print_variable	PANEL_LFOLEVEL
	print_variable	VCFENVMOD
	print_variable	PORTAMENTO
	print_variable	PULSE_KNOB
	print_variable	OSC_DETUNE

; upper knobs
	print_string	STR_LINE
	print_string	STR_UPPER

	print_variable	KNOB_DCF_DECAY
	print_variable	KNOB_DCF_ATTACK
	print_variable	KNOB_AMP_DECAY
	print_variable	KNOB_AMP_ATTACK

; midi cc parameter
	print_string	STR_LINE
	print_string	STR_MIDI_CC

.if USE_EXTENDED_MIDI_CONTROL
	print_variable	PWMDEPTH
	print_variable	MIDI_KNOB_2
	print_variable	MIDI_KNOB_3
	print_variable	MIDI_KNOB_4
	print_variable	LFO2FREQ
	print_variable	FMDEPTH
	print_variable	MIXER_BALANCE
.if USE_MASTER_VOLUME
	print_variable	MASTER_VOLUME
.endif	; USE_MASTER_VOLUME
	call	usart_write_crlf
.endif	; USE_EXTENDED_MIDI_CONTROL
	print_variable	ATTACKTIME
	print_variable	DECAYTIME
	print_variable	SUSTAINLEVEL
	print_variable	RELEASETIME

	call	usart_write_crlf

	print_variable	ATTACKTIME2
	print_variable	DECAYTIME2
	print_variable	SUSTAINLEVEL2
	print_variable	RELEASETIME2

; switches
	print_string	STR_LINE
	print_string	STR_MIDI_CC

	print_variable	SW1
	print_variable	SW2
.if USE_EXTENDED_MIDI_CONTROL
	print_variable	SW3
	print_variable	SW4
.endif	; USE_EXTENDED_MIDI_CONTROL
	call	usart_write_crlf

; patch switches
	print_variable	PATCH_SWITCH1
	print_variable	PATCH_SWITCH2
.if USE_EXTENDED_MIDI_CONTROL
	print_variable	PATCH_SWITCH3
	print_variable	PATCH_SWITCH4
.endif	; USE_EXTENDED_MIDI_CONTROL

; varibales
	print_string	STR_LINE
	print_string	STR_MIDI_CC

	print_variable	DCOA_LEVEL
	print_variable	DCOB_LEVEL
	print_variable	LPF_I
	print_variable	SCALED_RESONANCE
.if USE_MASTER_VOLUME
	print_variable	VOLUME
.endif	; USE_MASTER_VOLUME
	print_string	STR_LINE
	call	usart_write_crlf
	ret

; -------------
; the strings

STR_RESONANCE:		.db	0x0D, 0x0A, "RESONANCE      ", 0
STR_CUTOFF:		.db	0x0D, 0x0A, "CUTOFF         ", 0
STR_LFOFREQ:		.db	0x0D, 0x0A, "LFOFREQ        ", 0
STR_PANEL_LFOLEVEL:	.db	0x0D, 0x0A, "LFOLEVEL       ", 0
STR_VCFENVMOD:		.db	0x0D, 0x0A, "VCFENVMOD      ", 0
STR_PORTAMENTO:		.db	0x0D, 0x0A, "PORTAMENTO     ", 0
STR_PULSE_KNOB:		.db	0x0D, 0x0A, "PULSE_KNOB     ", 0
STR_OSC_DETUNE:		.db	0x0D, 0x0A, "OSC_DETUNE     ", 0
STR_KNOB_DCF_DECAY:	.db	0x0D, 0x0A, "DCF_DECAY      ", 0
STR_KNOB_DCF_ATTACK:	.db	0x0D, 0x0A, "DCF_ATTACK     ", 0
STR_KNOB_AMP_DECAY:	.db	0x0D, 0x0A, "AMP_DECAY      ", 0
STR_KNOB_AMP_ATTACK:	.db	0x0D, 0x0A, "AMP_ATTACK     ", 0

.if USE_EXTENDED_MIDI_CONTROL
STR_PWMDEPTH:		.db	0x0D, 0x0A, "PWMDEPTH       ", 0
STR_MIDI_KNOB_2: 	.db	0x0D, 0x0A, "MIDI_KNOB_2    ", 0
STR_MIDI_KNOB_3: 	.db	0x0D, 0x0A, "MIDI_KNOB_3    ", 0
STR_MIDI_KNOB_4: 	.db	0x0D, 0x0A, "MIDI_KNOB_4    ", 0
STR_FMDEPTH:		.db	0x0D, 0x0A, "FMDEPTH        ", 0
STR_LFO2FREQ:		.db	0x0D, 0x0A, "LFO2FREQ       ", 0
STR_MIXER_BALANCE:	.db	0x0D, 0x0A, "MIXER_BALANCE  ", 0
STR_MASTER_VOLUME:	.db	0x0D, 0x0A, "MASTER_VOLUME  ", 0
.endif	; USE_EXTENDED_MIDI_CONTROL
STR_ATTACKTIME:		.db	0x0D, 0x0A, "ATTACKTIME     ", 0
STR_DECAYTIME:		.db	0x0D, 0x0A, "DECAYTIME      ", 0
STR_SUSTAINLEVEL:	.db	0x0D, 0x0A, "SUSTAINLEVEL   ", 0
STR_RELEASETIME:	.db	0x0D, 0x0A, "RELEASETIME    ", 0
STR_ATTACKTIME2:	.db	0x0D, 0x0A, "ATTACKTIME2    ", 0
STR_DECAYTIME2:		.db	0x0D, 0x0A, "DECAYTIME2     ", 0
STR_SUSTAINLEVEL2:	.db	0x0D, 0x0A, "SUSTAINLEVEL2  ", 0
STR_RELEASETIME2:	.db	0x0D, 0x0A, "RELEASETIME2   ", 0

STR_SW1:		.db	0x0D, 0x0A, "SW1            ", 0
STR_SW2:		.db	0x0D, 0x0A, "SW2            ", 0
.if USE_EXTENDED_MIDI_CONTROL
STR_SW3:		.db	0x0D, 0x0A, "SW3            ", 0
STR_SW4:		.db	0x0D, 0x0A, "SW4            ", 0
.endif	; USE_EXTENDED_MIDI_CONTROL
STR_PATCH_SWITCH1:	.db	0x0D, 0x0A, "PATCH_SWITCH1  ", 0
STR_PATCH_SWITCH2:	.db	0x0D, 0x0A, "PATCH_SWITCH2  ", 0
.if USE_EXTENDED_MIDI_CONTROL
STR_PATCH_SWITCH3:	.db	0x0D, 0x0A, "PATCH_SWITCH3  ", 0
STR_PATCH_SWITCH4:	.db	0x0D, 0x0A, "PATCH_SWITCH4  ", 0
.endif	; USE_EXTENDED_MIDI_CONTROL

STR_DCOA_LEVEL:		.db	0x0D, 0x0A, "DCOA_LEVEL     ", 0
STR_DCOB_LEVEL:		.db	0x0D, 0x0A, "DCOB_LEVEL     ", 0
STR_SCALED_RESONANCE:	.db	0x0D, 0x0A, "SCLD_RESONANCE ", 0
STR_LPF_I:		.db	0x0D, 0x0A, "LPF_I          ", 0
STR_VOLUME:		.db	0x0D, 0x0A, "VOLUME         ", 0

STR_LINE:		.db	0x0D, 0x0A, "-------------------", 0
STR_UPPER:		.db	0x0D, 0x0A, "--- upper knobs ---", 0
STR_LOWER:		.db	0x0D, 0x0A, "--- lower knobs ---", 0
STR_MIDI_CC:		.db	0x0D, 0x0A, "--- midi control --", 0

.endif	; MEEBLIP_SE_ORIGINAL_CODE

;-----------------------------------------------------------------------------
	.EXIT
;-----------------------------------------------------------------------------
