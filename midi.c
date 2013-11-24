/**
 * LPC1114 Synthesizer
 * Matt Sarnoff (msarnoff.org)
 * November 24, 2013
 *
 * MIDI input.
 */

#include "sound.h"
#include "hardware.h"

extern _Bool chord_pgm_active;
extern _Bool pitch_pgm_active;
extern void add_note_to_input(int8_t note);

static volatile uint8_t midibuf[3];
static volatile uint8_t midibytesleft = 0;

static inline void handle_midi_command(void)
{
  switch (midibuf[0]) {
    case 0x80:  /* note off */
      note_off(midibuf[1]);
      break;
    case 0x90:  /* note on */
      if (!chord_pgm_active && !pitch_pgm_active) {
        note_on(midibuf[1]);
      } else {
        add_note_to_input(midibuf[1]);
      }
      break;
    case 0xE0:  /* pitch bend */
    {
      /* convert 14-bit value to 10-bit semitone amount
       * (right shift by 3 bits to get a range of +/- 2 semitones */
      uint16_t raw14bit = ((uint16_t)midibuf[2] << 7) | midibuf[0];
      int16_t semitones = (((int16_t)raw14bit) - 8192) >> 3;
      set_pitch_bend(semitones);
      break;
    }
    default:
      break;
  }
}


void UART_IRQHandler(void)
{
  /* get the received byte and clear the interrupt */
  uint8_t byte = UART_U0RBR;

  /* command byte */
  if (byte >= 0x80) {
    midibuf[0] = byte;
    switch (byte) {
      case 0x80:  /* note off */
      case 0x90:  /* note on */
      case 0xE0:  /* pitch bend */
        midibytesleft = 2;
        break;
      case 0xC0:  /* program change */
        midibytesleft = 1;
        break;
      case 0xFE:  /* ignore active sense */
        break;
      case 0xFC:  /* stop */
      case 0xFF:  /* reset */
      default:
        midibytesleft = 0;
    }
  }
  /* data byte */
  else {
    if (midibytesleft > 0) {
      midibuf[3-midibytesleft] = byte;
      midibytesleft--;
    }
    if (midibytesleft <= 0) {
      handle_midi_command();
    }
  }
}

