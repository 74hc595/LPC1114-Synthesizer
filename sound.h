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

typedef enum {
  LFO_TRIANGLE,
  LFO_SAWTOOTH,
  LFO_SQUARE,
  LFO_RANDOM,
  NUM_LFO_SHAPES
} lfo_shape_t;

typedef enum {
  SUSTAIN_OFF,
  SUSTAIN_ON,
  SUSTAIN_REPEAT
} sustain_mode_t;

enum {
  MOD_SRC_ENV = 1 << 0,
  MOD_SRC_LFO = 1 << 1
};

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
void set_glide_preset(glide_t glide);

/* Sets attack rate.
 * 0 is instantaneous; 128 is 1 second; 255 is 10 seconds */
void set_attack(uint8_t val);

/* Sets release rate. (same as attack) */
void set_release(uint8_t val);

/* Sets sustain mode. */
void set_sustain_mode(sustain_mode_t mode);

/* Sets number of echoes. */
void set_echoes(uint8_t val);
uint8_t get_echoes(void);

/* Sets filter cutoff to the frequency of the specified MIDI note. */
void set_filter_cutoff(uint8_t val);

/* Sets filter resonance to the given value between 0 ("infinite" resonance)
 * and 0x20000 ("no" resonance). */
void set_filter_resonance(uint32_t val);

/* Sets cutoff frequency modulation amount, in fractional semitones.
 * Modulation amount may be positive or negative. */
void set_filter_cutoff_mod_amount(int16_t semitones);

/* Sets LFO rate. The scale is exponential. */
void set_lfo_rate(uint8_t val);

/* Sets LFO shape. */
void set_lfo_shape(lfo_shape_t shape);
lfo_shape_t get_lfo_shape(void);

/* Sets modulation sources for cutoff frequency.
 * sources should be a combination of MOD_SRC_* constants. */
void set_filter_cutoff_mod_sources(uint8_t sources);

/* Sets modulation sources for pitch.
 * sources should be a combination of MOD_SRC_* constants. */
void set_pitch_mod_sources(uint8_t sources);

#endif
