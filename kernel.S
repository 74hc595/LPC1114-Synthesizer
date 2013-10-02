@ vim:noet:sw=8:ts=8:ai:syn=armasm

@ Execute the SysTick handler from RAM for super speed
.data
_SysTick_Handler:
	mov	r3, #0			@ sample accumulator
	adr	r0, osc0		@ point to first oscillator

osc_update_base:
@ Update all oscillators
@ Self-modifying code is used to change the waveform, volume, and duty cycle
.rept 4
	ldmia	r0, {r0, r1, r2}
	add	r2, r1
	stmia	r0!, {r2}
	mov	r1, #0			@ volume
@ ----- begin modifiable code
	asr	r2, #23
	sub	r2, #0			@ duty cycle
	lsl	r1, #16
	eor	r2, r1
@ ----- end modifiable code
	add	r3, r2
.endr

@ When sawtooth wave is selected, the "modifiable code" sections
@ will be replaced with the following instructions:
@	asr	r2, #15
@	mul	r2, r1
@	nop
@	nop

dacout:
	ldmia	r0, {r0, r1, r2}
	lsr	r3, #15			@ shift down to 12 bits for DAC
	add	r3, r1			@ convert 2's complement to offset
	orr	r3, r2			@ set /GA and /SHDN bits
	strh	r3, [r0]		@ send to DAC

	bx	lr


.align 4
@ oscillator states
@ 3 words per oscillator (phase pointer, frequency, phase)
@ the phase pointer allows r0 to point to the phase word after an ldm
oscillators:
osc0:	.word	osc0ph
osc0fr:	.word	0
osc0ph:	.word	0
osc1:	.word	osc1ph
osc1fr:	.word	0
osc1ph:	.word	0
osc2:	.word	osc2ph
osc2fr:	.word	0
osc2ph:	.word	0
osc3:	.word	osc3ph
osc3fr:	.word	0
osc3ph:	.word	0

@ constants used for DAC output
output_port:
	.word	0x40040008		@ SPI data register
offset:
	.word	0x00000800		@ convert 2's complement to offset
dac_bits:
	.word	0x00003000		@ /GA and /SHDN bits
test:
	.word	0x000007ff

.global oscillators
.global osc_update_base

@ make sure the thumb bit is set in the vector table
.global SysTick_Handler
.thumb_set SysTick_Handler, _SysTick_Handler
