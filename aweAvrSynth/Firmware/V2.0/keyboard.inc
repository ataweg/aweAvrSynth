;-------------------------------------------------------------------------------------------------------------------
;
; keyboard.inc
; switch mapping for avrSynth or meeblip
; Axel Werner
;
; requires asm2 assembler, asm leads to some undefined variable errors
;
;Changelog
;
; 2011.10.28	AWe	change keyboard layout, swap the 4 lower left switches with the 4 upper right switches
; 2011.10.22	AWe	switch layout changed to support new DCOs and ringmodulator
; 2011.09.07	AWe	modified coding with help of a macro and defines
;
;-------------------------------------------------------------------------------------------------------------------

#define VERSION_2011_09_07	0
#define VERSION_2011_10_23	1

#define USE_ORIGINAL_MAPPING	0
#define USE_ORIGMAP_NEW_ORDER	1

;-------------------------------------------------------------------------------------------------------------------
;
; MODEFLAGS1
;	; b0 = DCO DIST: 0=off, 1=on
;	; b1 = wave A: 0=saw, 1=squ
;	; b2 = wave B: 0=saw, 1=squ
;	; b3 = osc B: 0=off, 1=on
;	; b4 = DCA mode: 0=gate, 1=env
;	; b5 = transpose: 0=down, 1=up
;	; b6 = (noise)
;	; b7 = octave B: 0=down, 1=up
; MODEFLAGS2
;	; b0 = LFO MODE: 0=DCF, 1=DCO
;	; b1 = LFO WAVE: 0=tri, 1=squ
;	; b2 = DCF KBD TRACK: 0=off, 1=on
;	; b3 = (dcf mode hp/lp)
;	; b4 = (knob shift)
;	; b5 = MODWHEEL Enable
;	; b6 = LFO KBD SYNC: 0=off, 1=on
;	; b7 = LFO: 0=norm, 1=rand
;
; MODEFLAGS3
;	; b0 = 0=Mixer, 1= Ringmodulator
;	; b1 = PulsWidth Modulation 0=off(Panel), 1=LFO2
;	; b2 = ENV0: 0=ADSR, 1= AD/ASR        (not implemtd yet)
;	; b3 = ENV0: 0=AD, 1=ASR              (not implemtd yet)
;	; b4 = ENV1: 0=ADSR, 1= AR            (not implemtd yet)
;	; b5 = 
;	; b6 =
;	; b7 =
; MODEFLAGS4
;	; b0 = OSC A wave: 0=normal squ or SAW, 1=extended pwm or tri
;	; b1 = OSC A wave: 0=normal, 1=sin
;	; b2 = OSC B wave: 0=normal SQU or SAW, 1=extended pwm or tri
;	; b3 = OSC B wave: 0=normal, 1=sin
;	; b4 =
;	; b5 =
;	; b6 = LFO2 WAVE: 0=tri, 1=squ
;	; b7 = LFO2: 0=norm, 1=rand
;-------------------------------------------------------------------------------------------------------------------

; ---------------------------------------------------------------------
; map switch to modeflag
; ---------------------------------------------------------------------

#define map_sw2mf	macro_sw2mf

#define MF_LFO_RANDOM		REG_MF2, 7
#define MF_LFO_KBD_SYNC		REG_MF2, 6
#define MF_MOD_WHEEL		REG_MF2, 5
#define MF_KNOB_SHIFT		REG_MF2, 4
#define MF_DCF_MODE		REG_MF2, 3
#define MF_DCF_KBD_TRACK	REG_MF2, 2
#define MF_LFO_WAVE		REG_MF2, 1
#define MF_LFO_MODE		REG_MF2, 0

#define MF_OSC_B_OCTAVE		REG_MF1, 7
#define MF_OSC_A_NOISE		REG_MF1, 6
#define MF_TRANSPONSE		REG_MF1, 5
#define MF_DCA_GATE		REG_MF1, 4
#define MF_OSC_B_ENABLE		REG_MF1, 3
#define MF_OSC_B_WAVE		REG_MF1, 2
#define MF_OSC_A_WAVE		REG_MF1, 1
#define MF_DISTROTION		REG_MF1, 0

#define MF_RINGMOD		REG_MF3, 0
#define MF_PWMMOD		REG_MF3, 1

#define MF_OSC_A_EXTEND		REG_MF4, 0
#define MF_OSC_A_SIN		REG_MF4, 1
#define MF_OSC_B_EXTEND		REG_MF4, 2
#define MF_OSC_B_SIN		REG_MF4, 3

; ---------------------------------------------------------------------
;
; ---------------------------------------------------------------------

.MACRO macro_sw2mf	; reg, bit, switch, sw_num
;	bst	switch, sw_num
;	bld	reg, bit

	bst	@2, @3
	bld	@0, @1
.ENDMACRO

.def	SW1 	= R16	; SWITCH1  right side
.def	SW2 	= R17	; SWITCH2  left side
.def	REG_MF1 = R18	; MODEFLAGS1
.def	REG_MF2 = R19	; MODEFLAGS2
.def	REG_MF3 = R20	; MODEFLAGS3
.def	REG_MF4 = R21	; MODEFLAGS4

; ---------------------------------------------------------------------
;
; ---------------------------------------------------------------------

 #if USE_ORIGINAL_MAPPING
 ; ------------------
 ; original mapping
 ;
  # if USE_ORIGMAP_NEW_ORDER
  ; ------------------
  ; new code order,
; upper switch
	map_sw2mf	MF_OSC_A_WAVE   , SW2, 7	; Osc A wave saw/square      | PD7.PB3. SW1
	map_sw2mf	MF_OSC_B_ENABLE , SW2, 6	; Osc B on/off               | PD6.PB3. SW2
	map_sw2mf	MF_MOD_WHEEL    , SW2, 5	; Modwheel disable/enable    | PD5.PB3. SW3
	map_sw2mf	MF_TRANSPONSE   , SW2, 4	; Transpose down/up          | PD4.PB3. SW4
	map_sw2mf	MF_DCF_KBD_TRACK, SW1, 7	; DCF key track 0=off, 1=on  | PD7.PB1. SW9
	map_sw2mf	MF_DCF_MODE     , SW1, 6	; DCF mode 0=LP, 1=HP        | PD6.PB1. SW10
	map_sw2mf	MF_LFO_MODE     , SW1, 5	; LFO Mode 0=DCF, 1 = DCO    | PD5.PB1. SW11
	map_sw2mf	MF_LFO_KBD_SYNC , SW1, 4	; LFO keyboard sync off/on   | PD4.PB1. SW12
; lower switch
	map_sw2mf	MF_OSC_B_WAVE   , SW2, 3	; Osc B wave saw/square      | PD7.PB2. SW5
	map_sw2mf	MF_OSC_B_OCTAVE , SW2, 2	; Octave B down/up           | PD6.PB2. SW6
	map_sw2mf	MF_OSC_A_NOISE  , SW2, 1	; Osc A Noise                | PD5.PB2. SW7
	map_sw2mf	MF_DCA_GATE     , SW2, 0	; DCA gate/env               | PD4.PB2. SW8
	map_sw2mf	MF_DISTROTION   , SW1, 3	; DCO Distortion off/on      | PD7.PB0. SW13
	map_sw2mf	MF_KNOB_SHIFT   , SW1, 2	; Control knob shift         | PD6.PB0. SW14
	map_sw2mf	MF_LFO_WAVE     , SW1, 1	; LFO Wave tri/squ           | PD5.PB0. SW15
	map_sw2mf	MF_LFO_RANDOM   , SW1, 0	; LFO normal/random          | PD4.PB0. SW16
  #else
  ; ------------------
  ; original code order, sorted by switch order
  ;
	map_sw2mf	MF_LFO_RANDOM    , SW1, 0	; LFO normal/random          | PD4.PB0. SW16  LFO_RANDOM
	map_sw2mf	MF_LFO_WAVE      , SW1, 1	; LFO Wave tri/squ           | PD5.PB0. SW15  LFO_WAVE
	map_sw2mf	MF_KNOB_SHIFT    , SW1, 2	; Control knob shift         | PD6.PB0. SW14  KNOB_SHIFT
	map_sw2mf	MF_DISTROTION    , SW1, 3	; DCO Distortion off/on      | PD7.PB0. SW13  DISTROTION
	map_sw2mf	MF_LFO_KBD_SYNC  , SW1, 4	; LFO keyboard sync off/on   | PD4.PB1. SW12  LFO_KBD _SYNC
	map_sw2mf	MF_LFO_MODE      , SW1, 5	; LFO Mode 0=DCF, 1 = DCO    | PD5.PB1. SW11  LFO_SYNC
	map_sw2mf	MF_DCF_MODE      , SW1, 6	; DCF mode 0=LP, 1=HP        | PD6.PB1. SW10  LFO_DEST
	map_sw2mf	MF_DCF_KBD_TRACK , SW1, 7	; DCF key track 0=off, 1=on  | PD7.PB1. SW9   DCF_MODE

	map_sw2mf	MF_DCA_GATE      , SW2, 0	; DCA gate/env               | PD4.PB2. SW8   DCA_GATE
	map_sw2mf	MF_OSC_A_NOISE   , SW2, 1	; Osc A Noise                | PD5.PB2. SW7   OSC_A_NOISE
	map_sw2mf	MF_OSC_B_OCTAVE  , SW2, 2	; Octave B down/up           | PD6.PB2. SW6   OSC_B_OCTAVE
	map_sw2mf	MF_OSC_B_WAVE    , SW2, 3	; Osc B wave saw/square      | PD7.PB2. SW5   OSC_B_WAVE
	map_sw2mf	MF_TRANSPONSE    , SW2, 4	; Transpose down/up          | PD4.PB3. SW4   TRANSPONSE
	map_sw2mf	MF_MOD_WHEEL     , SW2, 5	; Modwheel disable/enable    | PD5.PB3. SW3   MOD_WHEEL
	map_sw2mf	MF_OSC_B_ENABLE  , SW2, 6	; Osc B on/off               | PD6.PB3. SW2   OSC_B_ENABLE
	map_sw2mf	MF_OSC_A_WAVE    , SW2, 7	; Osc A wave saw/square      | PD7.PB3. SW1   OSC_A_WAVE
   #endif
 #else
   #if VERSION_2011_09_07
 ; ------------------
 ; my new mapping

; upper switch
	map_sw2mf	MF_OSC_A_WAVE   , SW2, 7	; Osc A wave saw/square      | PD7.PB3.	SW1
	map_sw2mf	MF_MOD_WHEEL    , SW2, 6	; Modwheel disable/enable    | PD6.PB3.	SW2
	map_sw2mf	MF_OSC_A_NOISE  , SW2, 5	; Osc A Noise                | PD5.PB3.	SW3
	map_sw2mf	MF_DCF_KBD_TRACK, SW2, 4	; DCF key track 0=off, 1=on  | PD4.PB3.	SW4
	map_sw2mf	MF_LFO_MODE     , SW1, 7	; LFO Mode 0=DCF, 1 = DCO    | PD7.PB1.	SW9
	map_sw2mf	MF_LFO_KBD_SYNC , SW1, 6	; LFO keyboard sync off/on   | PD6.PB1.	SW10
	map_sw2mf	MF_DISTROTION   , SW1, 5	; DCO Distortion off/on      | PD5.PB1.	SW11
	map_sw2mf	MF_DCF_MODE     , SW1, 4	; DCF mode 0=LP, 1=HP        | PD4.PB1.	SW12
; lower switch
	map_sw2mf	MF_OSC_B_WAVE   , SW2, 3	; Osc B wave saw/square      | PD7.PB2.	SW5
	map_sw2mf	MF_OSC_B_ENABLE , SW2, 2	; Osc B on/off               | PD6.PB2.	SW6
	map_sw2mf	MF_OSC_B_OCTAVE , SW2, 1	; Octave B down/up           | PD5.PB2.	SW7
	map_sw2mf	MF_DCA_GATE     , SW2, 0	; DCA gate/env               | PD4.PB2.	SW8
	map_sw2mf	MF_LFO_WAVE     , SW1, 3	; LFO Wave tri/squ           | PD7.PB0.	SW13
	map_sw2mf	MF_LFO_RANDOM   , SW1, 2	; LFO normal/random          | PD6.PB0.	SW14
	map_sw2mf	MF_TRANSPONSE   , SW1, 1	; Transpose down/up          | PD5.PB0.	SW15
	map_sw2mf	MF_KNOB_SHIFT   , SW1, 0	; Control knob shift         | PD4.PB0.	SW16

   #elif VERSION_2011_10_23
; ------------------
; changes
;  add switches
;  MF_PWMMOD 		replace MF_MOD_WHEEL
;  MF_RINGMOD		replace MF_DCF_KBD_TRACK
;  MF_OSC_A_SIN		replace MF_LFO_KBD_SYNC
;  MF_OSC_A_EXTEND	replace MF_DISTROTION
;
;  MF_OSC_B_SIN		replace MF_LFO_RANDOM
;  MF_OSC_B_EXTEND	replace MF_TRANSPONSE
;  MF_TRANSPONSE	replace MF_DCA_GATE
; ------------------
;  OSC A wave: 0=normal squ or SAW, 1=extended pwm or tri
;  OSC A wave: 0=normal, 1=sin
;  OSC B wave: 0=normal SQU or SAW, 1=extended pwm or tri
;  OSC B wave: 0=normal, 1=sin

; upper switch
	map_sw2mf	MF_OSC_A_WAVE   , SW2, 7	; Osc A wave saw/square      | PD7.PB3.	SW1
	map_sw2mf	MF_PWMMOD	, SW2, 6	;                            | PD6.PB3.	SW2
	map_sw2mf	MF_OSC_A_NOISE  , SW2, 5	; Osc A Noise                | PD5.PB3.	SW3
	map_sw2mf	MF_RINGMOD      , SW2, 4	;                            | PD4.PB3.	SW4
	map_sw2mf	MF_LFO_MODE     , SW1, 7	; LFO Mode 0=DCF, 1 = DCO    | PD7.PB1.	SW9
	map_sw2mf	MF_OSC_A_SIN    , SW1, 6	;                            | PD6.PB1.	SW10
	map_sw2mf	MF_OSC_A_EXTEND , SW1, 5	;                            | PD5.PB1.	SW11
	map_sw2mf	MF_DCF_MODE     , SW1, 4	; DCF mode 0=LP, 1=HP        | PD4.PB1.	SW12
; lower switch
	map_sw2mf	MF_OSC_B_WAVE   , SW2, 3	; Osc B wave saw/square      | PD7.PB2.	SW5
	map_sw2mf	MF_OSC_B_ENABLE , SW2, 2	; Osc B on/off               | PD6.PB2.	SW6
	map_sw2mf	MF_OSC_B_OCTAVE , SW2, 1	; Octave B down/up           | PD5.PB2.	SW7
	map_sw2mf	MF_TRANSPONSE   , SW2, 0	; Transpose down/up          | PD4.PB2.	SW8
	map_sw2mf	MF_LFO_WAVE     , SW1, 3	; LFO Wave tri/squ           | PD7.PB0.	SW13
	map_sw2mf	MF_OSC_B_SIN    , SW1, 2	;                            | PD6.PB0.	SW14
	map_sw2mf	MF_OSC_B_EXTEND , SW1, 1	;                            | PD5.PB0.	SW15
	map_sw2mf	MF_KNOB_SHIFT   , SW1, 0	; Control knob shift         | PD4.PB0.	SW16
   #else
   #error "No valid keyboard mapping defined"
   #endif
#endif
