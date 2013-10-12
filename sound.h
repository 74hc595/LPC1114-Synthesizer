/**
 * Sound parameter control functions.
 */


#ifndef _SOUND_H_
#define _SOUND_H_

#include <stdint.h>

#define NUM_OSCILLATORS 4

typedef enum {
  GLIDE_OFF,
  GLIDE_FAST,
  GLIDE_MEDIUM,
  GLIDE_SLOW,
  NUM_GLIDE_PRESETS
} glide_t;

void sound_init(void);

void sound_set_duty_cycle(uint8_t val, uint8_t oscmask);
void sound_set_sawtooth(uint8_t oscmask);
void sound_set_lofi_sawtooth(uint8_t oscmask);

void sound_set_detune(uint8_t mode, uint8_t val);

void sound_set_oscillator_tuning(int8_t note_offsets[NUM_OSCILLATORS]);

void note_on(uint8_t notenum);
void note_off(uint8_t notenum);

/* Sets pitch bend in fractional semitones (signed) */
void set_pitch_bend(int16_t semitones);

/* Sets glide rate to a preset. */
void set_glide(glide_t glide);

void set_attack(uint16_t val);
void set_release(uint16_t val);

#endif
