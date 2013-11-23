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

typedef enum {
  FILTER_OFF,
  FILTER_LOWPASS,
  FILTER_HIGHPASS,
  FILTER_BANDPASS,
  NUM_FILTER_MODES
} filter_mode_t;

enum {
  MOD_SRC_ENV = 1 << 0,
  MOD_SRC_LFO = 1 << 1
};

void sound_init(void);


void note_on(uint8_t notenum);
void note_off(uint8_t notenum);

/* Sets the oscillator waveforms.
 * A 0 bit in waveformbits sets the corresponding oscillator to
 * output a pulse wave with the specified pulse width.
 * (0=50%, 255=0%)
 * A 1 bit in waveformbits sets the corresponding oscillator to
 * output a sawtooth wave. */
void set_oscillator_waveforms(uint8_t waveformbits, uint8_t pulse_width);

#if 0
/* Sets the oscillators indicated by 1-bits in oscmask to
 * output pulse waves at the specified duty cycle. (0=50%, 255=0%) */
void set_duty_cycle(uint8_t val, uint8_t oscmask);

/* Sets the oscillators indicated by 1-bits in oscmask to
 * output sawtooth waves. */
void set_sawtooth(uint8_t oscmask);
#endif

/* Sets oscillator fine tuning. mode indicates which oscillators
 * are in sync, val specifies the detune amount. */
void set_detune(uint8_t mode, uint8_t val);

/* Sets coarse tuning (in fractional semitones) for each oscillator. */
void set_oscillator_tuning(int8_t note_offsets[NUM_OSCILLATORS]);

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

/* Sets filter cutoff to the frequency of the specified MIDI note
 * in fractional semitones. */
void set_filter_cutoff(int32_t semitones);

/* Sets filter resonance to the given value between 0 ("infinite" resonance)
 * and 0x20000 ("no" resonance). */
void set_filter_resonance(uint32_t val);

/* Sets cutoff frequency modulation amount, in fractional semitones.
 * Modulation amount may be positive or negative. */
void set_filter_cutoff_mod_amount(int16_t semitones);

/* Sets filter mode. */
void set_filter_mode(filter_mode_t mode);
filter_mode_t get_filter_mode(void);

/* Whether filter cutoff frequency tracks the keyboard. */
void set_keyboard_tracking(_Bool track);
_Bool get_keyboard_tracking(void);

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

/* Sets modulation sources for pulse width.
 * sources should be a combination of MOD_SRC_* constants. */
void set_pulse_width_mod_sources(uint8_t sources);

/* Sets pitch modulation amont, in fractional semitones.
 * Modulation amount may be positive or negative. */
void set_pitch_mod_amount(int16_t semitones);

/* If true, pressing a key while another is held down will
 * not retrigger the envelope generators and LFO. */
void set_legato(_Bool val);
_Bool get_legato(void);

/* Sets modulation envelope attack rate.
 * 0 is instantaneous; 255 is 1 second */
void set_mod_attack(uint8_t val);

/* Sets modulation envelope release rate. (same as attack) */
void set_mod_release(uint8_t val);

/* Sets pulse width modulation amount. */
void set_pulse_width_mod_amount(int8_t amount);

#endif
