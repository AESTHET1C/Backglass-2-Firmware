;*************************************************************************
;* File Name         : main.asm
;* Title             : Backglass 2 Coprocessor Firmware
;* Created           : 2017-04-14
;* Author            : Alex Tavares <tavaresa13@gmail.com>
;* Target MCU        : ATmega48PB
;*
;* DESCRIPTION
;* Firmware for the audio/visual coprocessor on the Backglass 2 pinball
;* controller board. Accepts commands via I2C and controls speaker and
;* 4-digit 7-segment display outputs.
;* Speaker is controlled via 4 independent channels, each with frequency,
;* duty cycle, and volume parameters.
;* I2C transmissions are in the following scheme:
;*
;* Note 0, Note 1, Note 2, Note 3,
;* Voice 0, Voice 1, Voice 2, Voice 3,
;* Digit 0, Digit 1, Digit 2, Digit 3
;*
;* where transmissions must be 4, 8, or 12 data packets in length.
;* All other transmissions are discarded.
;*
;* Voice bytes are in the following scheme:
;*
;* VVVVVVDD
;*
;* V = Volume, D = Duty cycle
;*
;* Note that D = 0 is 12.5% duty cycle, while D = 3 is 50% duty cycle.
;*
;*************************************************************************

.INCLUDE <m48PBdef.inc>
.INCLUDE "timing_table.inc"

; Constants
.DEF ZERO = r0
.DEF FOUR = r1
.DEF EIGHT = r2
.DEF TWELVE = r3
; TWI condition codes
.DEF START_C = r4
.DEF DATA_C = r5
.DEF STOP_C = r6
; Firmware state variables
.DEF META_BANK_SELECT = r7     ; Either 0x00 or 0xFF
.DEF DURATION_BANK_SELECT = r8 ; Either 0x00 or 0xFF
.DEF TWI_QUEUE_POINTER = r9    ; 0 - 13
.DEF CURRENT_DIGIT = r10       ; 0x11, 0x22, 0x44, 0x88


.EQU TWI_SLAVE_ADDRESS = 1
.EQU MAX_AUDIO_CYCLES = 252


.MACRO ACK
ldi  ZL, 0b11000100
sts  TWCR, ZL
.ENDMACRO


; Interrupt vectors
.CSEG
.ORG 0x0000
rjmp INIT ; RESET
rjmp DUMMY_ISR ; INT0
rjmp DUMMY_ISR ; INT1
rjmp DUMMY_ISR ; PCINT0
rjmp DUMMY_ISR ; PCINT1
rjmp DUMMY_ISR ; PCINT2
rjmp DUMMY_ISR ; WDT
rjmp DUMMY_ISR ; TIMER2 COMPA
rjmp DUMMY_ISR ; TIMER2 COMPB
rjmp DUMMY_ISR ; TIMER2 OVR
rjmp DUMMY_ISR ; TIMER1 CAPT
rjmp DUMMY_ISR ; TIMER1 COMPA
rjmp DUMMY_ISR ; TIMER1 COMPB
rjmp TOV1_ISR ; TIMER1 OVF
rjmp DUMMY_ISR ; TIMER0 COMPA
rjmp DUMMY_ISR ; TIMER0 COMPB
rjmp DUMMY_ISR ; TIMER0 OVF
rjmp DUMMY_ISR ; SPI, STC
rjmp DUMMY_ISR ; USART, RX
rjmp DUMMY_ISR ; USART, UDRE
rjmp DUMMY_ISR ; USART, TX
rjmp DUMMY_ISR ; ADC
rjmp DUMMY_ISR ; EE READY
rjmp DUMMY_ISR ; ANALOG COMP
rjmp DUMMY_ISR ; TWI
rjmp DUMMY_ISR ; SPM READY
rjmp DUMMY_ISR ; USART, START


; Initialization of device and variables
INIT:

	; Shut down unused MCU module clocks
	ldi  r16, 0b0110001 ; Timer2, Timer0, USART0, ADC
	sts  PRR, r16
	
	; Initialize permanent registers
	; Do not modify these elsewhere in program
	clr  ZERO
	ldi  r16, 4
	mov  FOUR, r16
	ldi  r16, 8
	mov  EIGHT, r16
	ldi  r16, 12
	mov  TWELVE, r16
	ldi  r16, 0x60
	mov  START_C, r16
	ldi  r16, 0x80
	mov  DATA_C, r16
	ldi  r16, 0xA0
	mov  STOP_C, r16
	ldi  YH, high(SRAM_Low)               ; Y is always used to access lower half of SRAM

	; Set initial audio variables
	ldi  r16, 1                           ; Temporary use
	clr  META_BANK_SELECT                 ; Set initial active banks to "0"
	clr  DURATION_BANK_SELECT
	sts  (VolumeArray + 0), ZERO          ; Zero volume array, bank 0 (active bank)
	sts  (VolumeArray + 1), ZERO
	sts  (VolumeArray + 2), ZERO
	sts  (VolumeArray + 3), ZERO
	sts  (VoiceArray + 0), ZERO           ; Set all voices to "0", bank 0 (active bank)
	sts  (VoiceArray + 1), ZERO
	sts  (VoiceArray + 2), ZERO
	sts  (VoiceArray + 3), ZERO
	sts  (DurationArray + 0), r16         ; Set all note durations to "1" (for bank 0 [active bank])
	sts  (DurationArray + 1), ZERO
	sts  (DurationArray + 2), r16
	sts  (DurationArray + 3), ZERO
	sts  (DurationArray + 4), r16
	sts  (DurationArray + 5), ZERO
	sts  (DurationArray + 6), r16
	sts  (DurationArray + 7), ZERO
	sts  (DurationArray + 8), r16
	sts  (DurationArray + 9), ZERO
	sts  (DurationArray + 10), r16
	sts  (DurationArray + 11), ZERO
	sts  (DurationArray + 12), r16
	sts  (DurationArray + 13), ZERO
	sts  (DurationArray + 14), r16
	sts  (DurationArray + 15), ZERO
	sts  (ChannelPhaseArray + 0), ZERO    ; Set all channel phases to "0"
	sts  (ChannelPhaseArray + 1), ZERO
	sts  (ChannelPhaseArray + 2), ZERO
	sts  (ChannelPhaseArray + 3), ZERO
	sts  (RemainingArray + 0), r16        ; Set all notes to update on first interrupt
	sts  (RemainingArray + 1), ZERO
	sts  (RemainingArray + 2), r16
	sts  (RemainingArray + 3), ZERO
	sts  (RemainingArray + 4), r16
	sts  (RemainingArray + 5), ZERO
	sts  (RemainingArray + 6), r16
	sts  (RemainingArray + 7), ZERO
	sts  (RemainingArray + 8), r16
	sts  (RemainingArray + 9), ZERO
	sts  (RemainingArray + 10), r16
	sts  (RemainingArray + 11), ZERO
	sts  (RemainingArray + 12), r16
	sts  (RemainingArray + 13), ZERO
	sts  (RemainingArray + 14), r16
	sts  (RemainingArray + 15), ZERO
	 
	; Configure stack
	ldi  r16, high(RAMEND)
	out  SPH, r16
	ldi  r16, low(RAMEND)
	out  SPL, r16

	; Configure TIMER1
	in   r16, DDRB                  ; Set pin to output
	ldi  r17, 0b00000010
	or   r16, r17
	out  DDRB, r16
	ldi  r16, 0b10000010            ; Select fast non-inverting PWM mode
	sts  TCCR1A, r16
	ldi  r16, 0b00011000            ; Select ICR1 as TOP, do not yet enable clock
	sts  TCCR1B, r16
	ldi  r16, 0                     ; Set TOP value
	sts  ICR1H, r16
	ldi  r16, MAX_AUDIO_CYCLES
	sts  ICR1L, r16
	ldi  r16, 0b00000001            ; Enable overflow interrupt
	sts  TIMSK1, r16
	ldi  r16, 0b00011001            ; Enable clock (/1 prescaler)
	sts  TCCR1B, r16

	; Configure TWI
	clr  TWI_QUEUE_POINTER          ; Initialize byte select pointer
	ldi  r16, 0b00110000            ; Set TWI pullups
	in   r17, PORTC
	or   r17, r16
	out  PORTC, r17
	ldi  r16, TWI_SLAVE_ADDRESS     ; Set slave address
	lsl  r16
	sts  TWAR, r16
	ldi  r16, 0b01000100            ; Enable module
	sts  TWCR, r16

	;Set initial display variables
	ldi  r16, 0x11                  ; Set initial digit display to "0"
	mov  CURRENT_DIGIT, r16
	sts  Digit0, ZERO               ; Clear all digits
	sts  Digit1, ZERO
	sts  Digit2, ZERO
	sts  Digit3, ZERO
	ldi  r16, 0b11110000            ; Initialize output pins
	in   r17, PORTC
	and  r17, r16
	out  PORTC, r17
	out  PORTD, ZERO
	ldi  r16, 0b00001111
	out  DDRC, r16
	ldi  r16, 0b11111111
	out  DDRD, r16

	;Enable global interrupts
	sei

; Main code
; Handles TWI reading and state
TWI_LOOP:

	lds  r11, TWCR
	sbrs r11, 7
	rjmp TWI_LOOP

TWI_CHECK_CODE:
	lds  r11, TWSR
	cp   r11, START_C
	breq TWI_START
	cp   r11, DATA_C
	breq TWI_DATA_JUMP
	cp   r11, STOP_C
	breq TWI_STOP_JUMP
	ACK
	rjmp TWI_LOOP

	TWI_DATA_JUMP:
		rjmp TWI_DATA

	TWI_STOP_JUMP:
		rjmp TWI_STOP

	TWI_START:
		ACK
		clr  TWI_QUEUE_POINTER
		rjmp TWI_LOOP

	TWI_DATA:
		lds  r11, TWDR                ; Store received data to queue, prepare for next byte
		ACK
		ldi  ZH, high(TWIBufferArray)
		mov  ZL, TWI_QUEUE_POINTER
		st   Z, r11
		mov  r11, TWELVE              ; Increment queue pointer if transmission is still valid
		inc  r11
		cp   TWI_QUEUE_POINTER, r11
		brsh TWI_DATA_SKIP_INC_POINTER
		inc  TWI_QUEUE_POINTER
		TWI_DATA_SKIP_INC_POINTER:
		cp   TWI_QUEUE_POINTER, EIGHT      ; Update buffers if eigth byte received
		brne TWI_LOOP

		; Update meta buffer bank
		lds  r11, (TWIBufferArray + 4)     ; Get meta bytes from queue
		lds  r12, (TWIBufferArray + 5)
		lds  r13, (TWIBufferArray + 6)
		lds  r14, (TWIBufferArray + 7)
		mov  r15, META_BANK_SELECT         ; Store volumes to buffer bank
		com  r15
		and  r15, FOUR
		ldi  ZH, high(VolumeArray)
		ldi  ZL, low(VolumeArray)
		add  ZL, r15
		mov  r15, r11
		lsr  r15
		lsr  r15
		st   Z+, r15
		mov  r15, r12
		lsr  r15
		lsr  r15
		st   Z+, r15
		mov  r15, r13
		lsr  r15
		lsr  r15
		st   Z+, r15
		mov  r15, r14
		lsr  r15
		lsr  r15
		st   Z, r15
		ldi  ZL, 0b00000011              ; Store voices to buffer bank
		and  r11, ZL
		and  r12, ZL
		and  r13, ZL
		and  r14, ZL
		mov  r15, META_BANK_SELECT
		com  r15
		and  r15, FOUR
		ldi  ZL, low(VoiceArray)
		add  ZL, r15
		st   Z+, r11
		st   Z+, r12
		st   Z+, r13
		st   Z, r14

		; Update duration buffer bank (using buffer meta bank)
		mov  r15, META_BANK_SELECT
		com  r15
		and  r15, FOUR
		ldi  ZH, high(SRAM_Low)

			; Skip loading channel 0's voice, as it's already in r11
		add  r11, FOUR
		lsl  r11
		ldi  ZL, (low(TWIBufferArray) + 0)
		ld   ZL, Z
		lsl  ZL
		lsl  ZL
		adc  r11, ZERO
		mov  ZH, r11
		lpm  r11, Z+
		lpm  r12, Z+
		lpm  r13, Z+
		lpm  r14, Z
		mov  ZL, DURATION_BANK_SELECT
		com  ZL
		andi ZL, 16
		ldi  ZH, (low(DurationArray) + 0)
		add  ZL, ZH
		ldi  ZH, high(SRAM_Low)
		st   Z+, r11
		st   Z+, r12
		st   Z+, r13
		st   Z, r14

		ldi  ZL, (low(VoiceArray) + 1)
		add  ZL, r15
		ld   r11, Z
		add  r11, FOUR
		lsl  r11
		ldi  ZL, (low(TWIBufferArray) + 1)
		ld   ZL, Z
		lsl  ZL
		lsl  ZL
		adc  r11, ZERO
		mov  ZH, r11
		lpm  r11, Z+
		lpm  r12, Z+
		lpm  r13, Z+
		lpm  r14, Z
		mov  ZL, DURATION_BANK_SELECT
		com  ZL
		andi ZL, 16
		ldi  ZH, (low(DurationArray) + 4)
		add  ZL, ZH
		ldi  ZH, high(SRAM_Low)
		st   Z+, r11
		st   Z+, r12
		st   Z+, r13
		st   Z, r14

		ldi  ZL, (low(VoiceArray) + 2)
		add  ZL, r15
		ld   r11, Z
		add  r11, FOUR
		lsl  r11
		ldi  ZL, (low(TWIBufferArray) + 2)
		ld   ZL, Z
		lsl  ZL
		lsl  ZL
		adc  r11, ZERO
		mov  ZH, r11
		lpm  r11, Z+
		lpm  r12, Z+
		lpm  r13, Z+
		lpm  r14, Z
		mov  ZL, DURATION_BANK_SELECT
		com  ZL
		andi ZL, 16
		ldi  ZH, (low(DurationArray) + 8)
		add  ZL, ZH
		ldi  ZH, high(SRAM_Low)
		st   Z+, r11
		st   Z+, r12
		st   Z+, r13
		st   Z, r14

		ldi  ZL, (low(VoiceArray) + 3)
		add  ZL, r15
		ld   r11, Z
		add  r11, FOUR
		lsl  r11
		ldi  ZL, (low(TWIBufferArray) + 3)
		ld   ZL, Z
		lsl  ZL
		lsl  ZL
		adc  r11, ZERO
		mov  ZH, r11
		lpm  r11, Z+
		lpm  r12, Z+
		lpm  r13, Z+
		lpm  r14, Z
		mov  ZL, DURATION_BANK_SELECT
		com  ZL
		andi ZL, 16
		ldi  ZH, (low(DurationArray) + 12)
		add  ZL, ZH
		ldi  ZH, high(SRAM_Low)
		st   Z+, r11
		st   Z+, r12
		st   Z+, r13
		st   Z, r14

		rjmp TWI_LOOP

	TWI_STOP:
		ACK
		cp   TWI_QUEUE_POINTER, FOUR
		breq TWI_STOP_4
		cp   TWI_QUEUE_POINTER, EIGHT
		breq TWI_STOP_8
		cp   TWI_QUEUE_POINTER, TWELVE
		breq TWI_STOP_12
		rjmp TWI_LOOP

		TWI_STOP_12:

			; Update display
			lds  r11, (TWIBufferArray + 8)
			lds  r12, (TWIBufferArray + 9)
			lds  r13, (TWIBufferArray + 10)
			lds  r14, (TWIBufferArray + 11)
			sts  Digit0, r11
			sts  Digit1, r12
			sts  Digit2, r13
			sts  Digit3, r14

			; rjmp purposely omitted

		TWI_STOP_8:
			
			; Swap duration bank
			com  DURATION_BANK_SELECT
			
			; Swap meta bank
			com  META_BANK_SELECT

			rjmp TWI_LOOP

		TWI_STOP_4:

			; Update duration buffer bank (using active meta bank)
			mov  r15, META_BANK_SELECT
			and  r15, FOUR
			ldi  ZH, high(SRAM_Low)

			ldi  ZL, (low(VoiceArray) + 0)
			add  ZL, r15
			ld   r11, Z
			add  r11, FOUR
			lsl  r11
			ldi  ZL, (low(TWIBufferArray) + 0)
			ld   ZL, Z
			lsl  ZL
			adc  r11, ZERO
			mov  ZH, r11
			lpm  r11, Z+
			lpm  r12, Z+
			lpm  r13, Z+
			lpm  r14, Z
			mov  ZL, DURATION_BANK_SELECT
			com  ZL
			andi ZL, 16
			ldi  ZH, (low(DurationArray) + 0)
			add  ZL, ZH
			ldi  ZH, high(SRAM_Low)
			st   Z+, r11
			st   Z+, r12
			st   Z+, r13
			st   Z, r14

			ldi  ZL, (low(VoiceArray) + 1)
			add  ZL, r15
			ld   r11, Z
			add  r11, FOUR
			lsl  r11
			ldi  ZL, (low(TWIBufferArray) + 1)
			ld   ZL, Z
			lsl  ZL
			adc  r11, ZERO
			mov  ZH, r11
			lpm  r11, Z+
			lpm  r12, Z+
			lpm  r13, Z+
			lpm  r14, Z
			mov  ZL, DURATION_BANK_SELECT
			com  ZL
			andi ZL, 16
			ldi  ZH, (low(DurationArray) + 4)
			add  ZL, ZH
			ldi  ZH, high(SRAM_Low)
			st   Z+, r11
			st   Z+, r12
			st   Z+, r13
			st   Z, r14

			ldi  ZL, (low(VoiceArray) + 2)
			add  ZL, r15
			ld   r11, Z
			add  r11, FOUR
			lsl  r11
			ldi  ZL, (low(TWIBufferArray) + 2)
			ld   ZL, Z
			lsl  ZL
			adc  r11, ZERO
			mov  ZH, r11
			lpm  r11, Z+
			lpm  r12, Z+
			lpm  r13, Z+
			lpm  r14, Z
			mov  ZL, DURATION_BANK_SELECT
			com  ZL
			andi ZL, 16
			ldi  ZH, (low(DurationArray) + 8)
			add  ZL, ZH
			ldi  ZH, high(SRAM_Low)
			st   Z+, r11
			st   Z+, r12
			st   Z+, r13
			st   Z, r14

			ldi  ZL, (low(VoiceArray) + 3)
			add  ZL, r15
			ld   r11, Z
			add  r11, FOUR
			lsl  r11
			ldi  ZL, (low(TWIBufferArray) + 3)
			ld   ZL, Z
			lsl  ZL
			adc  r11, ZERO
			mov  ZH, r11
			lpm  r11, Z+
			lpm  r12, Z+
			lpm  r13, Z+
			lpm  r14, Z
			mov  ZL, DURATION_BANK_SELECT
			com  ZL
			andi ZL, 16
			ldi  ZH, (low(DurationArray) + 12)
			add  ZL, ZH
			ldi  ZH, high(SRAM_Low)
			st   Z+, r11
			st   Z+, r12
			st   Z+, r13
			st   Z, r14

			; Swap duration bank
			com  DURATION_BANK_SELECT

			rjmp TWI_LOOP


TOV1_ISR:

	; Save status flags to stack
	in   r16, SREG
	push r16

	; Handle audio output (99 - 167 cycles)
	TOV1_ISR_AUDIO:
	lds  r16, (ChannelPhaseArray + 0) ; r16 = Channel 0 phase
	lds  r17, (ChannelPhaseArray + 1) ; r17 = Channel 1 phase
	lds  r18, (ChannelPhaseArray + 2) ; r18 = Channel 2 phase
	lds  r19, (ChannelPhaseArray + 3) ; r19 = Channel 3 phase
	mov  YL, META_BANK_SELECT
	andi YL, 4
	adiw YL, low(VolumeArray)
	ld   r20, Y+ ; r20 = Channel 0 volume
	ld   r21, Y+ ; r21 = Channel 1 volume
	ld   r22, Y+ ; r22 = Channel 2 volume
	ld   r23, Y  ; r23 = Channel 3 volume

	TOV1_ISR_CHANNEL0:
	lds  r24, (RemainingArray + 0)  ; Get remaining cycles of channel
	lds  r25, (RemainingArray + 1)
	sbiw r24, 1                     ; Update remaining cycles of channel
	brne TOV1_ISR_CHANNEL1
	com  r16                        ; Update phase of channel
	mov  YL, DURATION_BANK_SELECT   ; Get new phase duration of channel from SRAM array
	andi YL, 16
	ldi  r24, (low(DurationArray) + 0)
	add  YL, r24
	mov  r24, r16
	andi r24, 0x02
	add  YL, r24
	ld   r24, Y+
	ld   r25, Y
	sbiw r24, 1                     ; Test if new duration is 1
	brne TOV1_ISR_CHANNEL0_ACTIVE
	clr  r20                        ; Disable channel if true
	TOV1_ISR_CHANNEL0_ACTIVE:
	adiw r24, 1

	TOV1_ISR_CHANNEL1:
	sts  (RemainingArray + 0), r24  ; Save remaining cycles of previous channel to SRAM
	sts  (RemainingArray + 1), r25
	lds  r24, (RemainingArray + 2)
	lds  r25, (RemainingArray + 3)
	sbiw r24, 1
	brne TOV1_ISR_CHANNEL2
	com  r17
	mov  YL, DURATION_BANK_SELECT
	andi YL, 16
	ldi  r24, (low(DurationArray) + 4)
	add  YL, r24
	mov  r24, r17
	andi r24, 0x02
	add  YL, r24
	ld   r24, Y+
	ld   r25, Y
	sbiw r24, 1
	brne TOV1_ISR_CHANNEL1_ACTIVE
	clr  r21
	TOV1_ISR_CHANNEL1_ACTIVE:
	adiw r24, 1

	TOV1_ISR_CHANNEL2:
	sts  (RemainingArray + 2), r24
	sts  (RemainingArray + 3), r25
	lds  r24, (RemainingArray + 4)
	lds  r25, (RemainingArray + 5)
	sbiw r24, 1
	brne TOV1_ISR_CHANNEL3
	com  r18
	mov  YL, DURATION_BANK_SELECT
	andi YL, 16
	ldi  r24, (low(DurationArray) + 8)
	add  YL, r24
	mov  r24, r18
	andi r24, 0x02
	add  YL, r24
	ld   r24, Y+
	ld   r25, Y
	sbiw r24, 1
	brne TOV1_ISR_CHANNEL2_ACTIVE
	clr  r22
	TOV1_ISR_CHANNEL2_ACTIVE:
	adiw r24, 1

	TOV1_ISR_CHANNEL3:
	sts  (RemainingArray + 4), r24
	sts  (RemainingArray + 5), r25
	lds  r24, (RemainingArray + 6)
	lds  r25, (RemainingArray + 7)
	sbiw r24, 1
	brne TOV1_ISR_AUDIO_OUTPUT
	com  r19
	mov  YL, DURATION_BANK_SELECT
	andi YL, 16
	ldi  r24, (low(DurationArray) + 12)
	add  YL, r24
	mov  r24, r19
	andi r24, 0x02
	add  YL, r24
	ld   r24, Y+
	ld   r25, Y
	sbiw r24, 1
	brne TOV1_ISR_CHANNEL3_ACTIVE
	clr  r23
	TOV1_ISR_CHANNEL3_ACTIVE:
	adiw r24, 1

	TOV1_ISR_AUDIO_OUTPUT:
	sts  (RemainingArray + 6), r24
	sts  (RemainingArray + 7), r25
	and  r20, r16                   ; Calculate audio output level
	and  r21, r17
	and  r22, r18
	and  r23, r19
	add  r20, r21
	add  r20, r22
	add  r20, r23
	sts  OCR1AH, ZERO               ; Set audio output
	sts  OCR1AL, r20

	sts  (ChannelPhaseArray + 0), r16 ; Save phase states to SRAM
	sts  (ChannelPhaseArray + 1), r17
	sts  (ChannelPhaseArray + 2), r18
	sts  (ChannelPhaseArray + 3), r19

	; Handle 7-seg display
	TOV1_ISR_DISPLAY:
	lsl  CURRENT_DIGIT
	adc  CURRENT_DIGIT, ZERO
	ldi  XH, 0x02
	mov  XL, CURRENT_DIGIT
	ld   r16, X
	mov  r17, CURRENT_DIGIT
	andi r17, 0x0F
	in   r18, PORTC
	andi r18, 0xF0
	or   r18, r17
	out  PORTD, ZERO
	out  PORTC, r18
	out  PORTD, r16

	; Revert status flags and return
	pop  r16
	out  SREG, r16
	reti


; Handler for erroneous interrupts
; If this is called, something's gone horribly wrong
DUMMY_ISR:
	rjmp INIT ; Try to restart
	          ; If the interrupt source is continouous, admit defeat


; Reserve SRAM variables
.DSEG
.ORG 0x0100
SRAM_Low:
TWIBufferArray:    .BYTE 12
                   .BYTE 2         ; Bit buckets
VoiceArray:        .BYTE (4 + 4)   ; Bank selected via META_BANK_SELECT
VolumeArray:       .BYTE (4 + 4)   ; Bank selected via META_BANK_SELECT
DurationArray:     .BYTE (16 + 16) ; Bank selected via DURATION_BANK_SELECT
ChannelPhaseArray: .BYTE 4
RemainingArray:    .BYTE 8


; Reserve 7-seg digit output array
.ORG 0x0211
Digit0:            .BYTE 1
.ORG 0x0222
Digit1:            .BYTE 1
.ORG 0x0244
Digit2:            .BYTE 1
.ORG 0x0288
Digit3:            .BYTE 1


;Reserve stack
.ORG 0x0289
.BYTE 119
