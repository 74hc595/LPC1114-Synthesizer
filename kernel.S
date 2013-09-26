.data

/* will be aligned on a 256-byte boundary */
.global waveform
waveform:
  .rept 128
  .byte 0x00
  .endr
  .rept 128
  .byte 0xFF
  .endr


/* Execute the SysTick handler from RAM for super speed */
_SysTick_Handler:
  adr   r0, waveform_ptr_addr /* prepare to load 3 words */
  ldm   r0, {r0, r1, r2}      /* r0=&waveform_ptr, r1=waveform_ptr, r2=output port */
  ldrb  r3, [r1]              /* get table entry */
  lsl   r3, #4                /* shift value for DAC */
  add   r3, r12               /* set DAC control bits */
  strh  r3, [r2]              /* send to DAC */
  add   r1, #4                /* increment pointer */
  strb  r1, [r0]              /* only store lo byte, high bits aren't touched */
  bx    lr


.align 4
waveform_ptr_addr:
  .word waveform_ptr

.global waveform_ptr
waveform_ptr:
  .word 0

output_port:
  .word 0x40040008  /* SPI data register */

/* make sure the thumb bit is set in the vector table */
.global SysTick_Handler
.thumb_set SysTick_Handler, _SysTick_Handler
