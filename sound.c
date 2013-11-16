#include "sound.h"
#include "hardware.h"
#include "tables.h"
#include <stdbool.h>

extern volatile uint32_t filter_cutoff;
extern volatile uint32_t filter_q;
extern volatile uint16_t filter_mode_control;
extern volatile uint16_t filter_bypass_control;
extern volatile uint16_t volume_control;
extern volatile uint16_t silence;

/**
 * Oscillator state.
 */
typedef struct
{
  uint32_t *phaseptr;
  uint32_t freq;
  uint32_t phase;
} oscillator_state_t;


/**
 * This structure mirrors the oscillator-updating code in the SysTick handler
 * and allows it to be modified in a structured manner.
 * The __pad members should not be touched!
 */
typedef struct
{
  uint16_t __pad1[5];
  union {
    uint16_t waveform_code[3];
    struct {
      uint16_t __pad3;
      uint8_t duty;
      uint8_t __pad4;
      uint16_t __pad5[1];
    };
  };
  uint16_t __pad6;
} oscillator_control_t;


typedef enum {
  ENV_OFF,
  ENV_ATTACK,
  ENV_SUSTAIN,
  ENV_RELEASE,
  ENV_REPEAT
} env_stage_t;

/**
 * The synth uses last-note priority; when multiple keys are held down,
 * the one that sounds is always the most recent one pressed.
 * We keep of the last 4 notes pressed using a 4-byte queue
 * that can be manipulated in constant time using shift and mask operations.
 */
typedef struct {
  uint8_t count;
  union {
    uint32_t bits;
    uint8_t bytes[4];
  };
} byte_queue_t;

/**
 * Pitches are represented as 7.9 fixed-point numbers, with the integer
 * part representing a MIDI note number (equal temperament) and the lower
 * 9 bits representing a fraction of a half step.
 * A lookup table is used to convert note numbers to frequencies, but linear
 * interpolation is used for intermediate pitches.
 * Thus the full pitch range is a piecewise linear approximation of an exponential.
 */

/* The current pitch value, including glide but not including pitch bend. */
static uint16_t current_pitch = 0;

/* The current pitch value, including pitch bend and modulation. */
static uint16_t current_pitch_post_mod = 0;

/* The "destination" pitch; the pitch that the glide is ascending/descending toward.
 * If glide is off, this will be identical to current_pitch. */
static uint16_t dest_pitch = 0;

/* Glide rate. If 0, glide is off and pitch changes are instantaneous. */
static int16_t glide_rate = 0;

/* Pitch bend amount. Signed. */
static int16_t pitch_bend = 0;

/* Additional coarse tuning offsets applied to individual oscillators. */
static int16_t tuning_amounts[NUM_OSCILLATORS] = {0};

/* Additional fine tuning amounts applied to individual oscillators. */
static int16_t detune_amounts[NUM_OSCILLATORS] = {0};

/* 4-element queue used to keep track of which keys are being held down.
 * Each element is a MIDI note number. */
static byte_queue_t playing_notes = {0};

/* Indicates that the oscillator frequencies need to be recomputed.
 * If true, frequencies will be updated on the next tick of the low-frequency timer. */
static volatile _Bool freq_needs_update = false;
static volatile _Bool filter_needs_update = false;

/* Envelope attack time constant. */
static uint16_t attack = 0;

/* Envelope release time constant. */
static uint16_t release = 0;

/* Envelope value. Only the upper byte is used for amplitude. */
static uint16_t envelope = 0;

/* Sustain mode: off, on, or repeat. */
static sustain_mode_t sustain_mode = SUSTAIN_OFF;

/* Number of "echoes," i.e. times the release phase repeats after the
 * normal release ends. */
static uint8_t echoes = 0;

/* Number of echoes currently left. */
static uint8_t echoes_left = 0;

/* Envelope stage. */
static env_stage_t envelope_stage = ENV_OFF;

/* Filter parameters prior to correction */
static int32_t cutoff_pitch = 0; /* fixed point note number */
static uint32_t uncorrected_q = 0;

/* Cached filter mode constant; the filter mode is actually determined
 * by the instruction pointed to by filter_mode_control */
static filter_mode_t filter_mode = FILTER_LOWPASS;

/* Whether the filter cutoff frequency tracks the keyboard. */
static _Bool keyboard_tracking = true;

/* Cutoff modulation strength, in fractional semitones. */
static int16_t cutoff_mod_amount = 0;

/* Pitch modulation strength, in fractional semitones. */
static int16_t pitch_mod_amount = 0;

/* If true, the envelope and LFO won't be retriggered if a note is pressed
 * while another is held down. */
static _Bool legato = false;

/* LFO state */
static uint16_t lfo_phase = 0;
static uint16_t lfo_freq = 0;
static uint16_t lfo_value = 0;
static lfo_shape_t lfo_shape = 0;

/* Modulation envelope state */
static uint16_t mod_attack = 0;
static uint16_t mod_release = 0;
static int32_t mod_envelope = 0;
static env_stage_t mod_envelope_stage = ENV_OFF;

/* Moulation sources */
static _Bool lfo_affects_cutoff = false;
static _Bool lfo_affects_pitch = false;
static _Bool env_affects_cutoff = false;
static _Bool env_affects_pitch = false;

extern volatile oscillator_state_t oscillators[4];
extern volatile oscillator_control_t osc_update_base[4];


#define TEST_FREQ 2000000

/**
 * Generates a pseudorandom 8-bit value.
 */ 
static inline uint8_t rand8(void)
{
  static int32_t state = 1;
  state = (1103515245*state + 12345) & 0x7FFFFFFF;
  return (state & 0xFF);
}


/**
 * Modifies the waveform generation code to generate a pulse wave for the
 * specified voice.
 */
static inline void oscillator_set_pulse(int oscnum, uint8_t duty)
{
  osc_update_base[oscnum].waveform_code[0] = 0x15d2;      /* asr r2, #23 */
  osc_update_base[oscnum].waveform_code[1] = 0x3a00|duty; /* sub r2, #<duty> */ 
  osc_update_base[oscnum].waveform_code[2] = 0x404a;      /* eor r2, r1 */
}


/**
 * Modifies the waveform generation code to generate a sawtooth wave for the
 * specified voice.
 */
static inline void oscillator_set_sawtooth(int oscnum)
{
  osc_update_base[oscnum].waveform_code[0] = 0x1092; /* asr r2, #2 */
  osc_update_base[oscnum].waveform_code[1] = 0x46c0; /* nop */
  osc_update_base[oscnum].waveform_code[2] = 0x46c0; /* nop */
}


void sound_init(void)
{
  lfo_shape = LFO_TRIANGLE;
  cutoff_pitch = NUM_CUTOFF_ENTRIES << 9;
  uncorrected_q = 0x20000;
  filter_needs_update = true;
  pitch_mod_amount = 12 << 9;
  mod_attack = 0xFFFF;
  mod_release = 0x1000;
}


void set_duty_cycle(uint8_t val, uint8_t oscmask)
{
  int i;
  for (i = 0; i < NUM_OSCILLATORS; i++) {
    if (oscmask & (1 << i)) {
      /* is the oscillator already in pulse mode? */
      if (osc_update_base[i].waveform_code[0] == 0x15d2) {
        osc_update_base[i].duty = val;
      }
      /* if not, switch to pulse mode */
      else {
        oscillator_set_pulse(i, val);
      }
    }
  }
}


void set_sawtooth(uint8_t oscmask)
{
  int i;
  for (i = 0; i < NUM_OSCILLATORS; i++) {
    if (oscmask & (1 << i)) {
        oscillator_set_sawtooth(i);
    }
  }
}


void update_frequencies()
{
  int i;
  current_pitch_post_mod = current_pitch + pitch_bend;

  if (lfo_affects_pitch) {
    int32_t mod_amount = lfo_value * pitch_mod_amount;
    current_pitch_post_mod += (mod_amount >> 16);
  }

  if (env_affects_pitch) {
    int32_t mod_amount = mod_envelope * pitch_mod_amount;
    current_pitch_post_mod += (mod_amount >> 16);
  }

  for (i = 0; i < NUM_OSCILLATORS; i++) {
    uint16_t note = current_pitch_post_mod + tuning_amounts[i] + detune_amounts[i];
    uint8_t basenote = note >> 9;
    uint32_t fracnote = note & ((1 << 9)-1);
    uint32_t basefreq = notetable[basenote];
    uint32_t delta = (notetable[basenote+1]-basefreq) >> 9;
    oscillators[i].freq = basefreq + fracnote*delta;
  }  
}  


void set_detune(uint8_t mode, uint8_t val)
{
  switch (mode) {
    case 0:
      detune_amounts[0] = 0;
      detune_amounts[1] = 0;
      detune_amounts[2] = val;
      detune_amounts[3] = val;
      break;
    case 1:
      detune_amounts[0] = 0;
      detune_amounts[1] = 0;
      detune_amounts[2] = val;
      detune_amounts[3] = -val;
      break;
    case 2:
      detune_amounts[0] = 0;
      detune_amounts[1] = val*2;
      detune_amounts[2] = val;
      detune_amounts[3] = -(val >> 1);
      break;
    case 3:
      detune_amounts[0] = 0;
      detune_amounts[1] = -val*2;
      detune_amounts[2] = val*3;
      detune_amounts[3] = val;
      break;

  }
  freq_needs_update = true;
}


void set_oscillator_tuning(int8_t note_offsets[NUM_OSCILLATORS])
{
  int i;
  for (i = 0; i < NUM_OSCILLATORS; i++) {
    tuning_amounts[i] = note_offsets[i] << 9;
  }
  freq_needs_update = true;
}


uint32_t freq_for_note(uint16_t note)
{
  uint8_t basenote = note >> 9;
  uint32_t fracnote = note & ((1 << 9)-1);
  uint32_t basefreq = notetable[basenote];
  uint32_t delta = (notetable[basenote+1]-basefreq) >> 9;
  return basefreq + fracnote*delta;
}


void note_on(uint8_t notenum)
{
  /* add note to the queue; kick out the oldest note if the queue is full */
  if (playing_notes.count == 4) {
    playing_notes.bits >>= 8;
    playing_notes.count--;
  }
  playing_notes.bits |= (uint32_t)notenum << (playing_notes.count*8);
  playing_notes.count++;

  dest_pitch = notenum << 9;
  if (!glide_rate) {
    current_pitch = dest_pitch;
  }

  /* if this is the first note being played, reset glide */
  if (playing_notes.count == 1 || sustain_mode == SUSTAIN_OFF) {
    current_pitch = dest_pitch;
    envelope_stage = ENV_ATTACK;
  }

  /* if this is the first note being played, or legato is off,
   * retrigger the envelopes and lfo */
  if (!legato || playing_notes.count == 1 || sustain_mode == SUSTAIN_OFF) {
    envelope = 0;
    lfo_phase = 0;
    mod_envelope = 0;
    mod_envelope_stage = ENV_ATTACK;
  }

  freq_needs_update = true;
}


void note_off(uint8_t notenum)
{
  /* remove the note from the queue
   * assumes that the note won't appear twice, which would only happen if
   * the MIDI device misbehaved and sent multiple note-on messages for the
   * same note without any note-offs */
  int i;
  for (i = 0; i < playing_notes.count; i++) {
    if (playing_notes.bytes[i] == notenum) {
      uint8_t shift = i*8;
      uint32_t shiftedbits = playing_notes.bits;
      playing_notes.bits &= ((uint32_t)1 << shift)-1;
      shiftedbits = (shiftedbits >> (shift+8)) << shift;
      playing_notes.bits |= shiftedbits;
      playing_notes.count--;
      break;
    }
  }

  /* if no more keys are held down, stop playing */
  if (playing_notes.count == 0) {
    if (envelope_stage != ENV_OFF) {
      envelope_stage = ENV_RELEASE;
      mod_envelope_stage = ENV_RELEASE;
    }
  }
  /* otherwise, change pitch to the note at the end of the queue */
  else {
    dest_pitch = playing_notes.bytes[playing_notes.count-1] << 9;
    if (!glide_rate) {
      current_pitch = dest_pitch;
    }
    freq_needs_update = true;
  }
}


void set_pitch_bend(int16_t semitones)
{
  pitch_bend = semitones;
  freq_needs_update = true;
}


void set_glide_preset(glide_t glide)
{
  switch (glide) {
    case GLIDE_OFF:
    default:
      glide_rate = 0;
      current_pitch = dest_pitch;
      break;
    case GLIDE_FAST:
      glide_rate = 300;
      break;
    case GLIDE_MEDIUM:
      glide_rate = 150;
      break;
    case GLIDE_SLOW:
      glide_rate = 50;
      break;
  }
}


void set_attack(uint8_t val)
{
  attack = envtable[val];
}


void set_release(uint8_t val)
{
  release = envtable[val];
}


void set_sustain_mode(sustain_mode_t mode)
{
  sustain_mode = mode;
}


void set_echoes(uint8_t val)
{
  echoes = echoes_left = val;
}


uint8_t get_echoes(void)
{
  return echoes;
}


void set_filter_cutoff(uint8_t val)
{
  cutoff_pitch = val << 9;
//  uncorrected_cutoff = (val < NUM_CUTOFF_ENTRIES) ? cutofftable[val] : 0xFFFF;
  filter_needs_update = true;
}


void set_filter_resonance(uint32_t val)
{
  uncorrected_q = val;
  filter_needs_update = true;
}


void set_filter_cutoff_mod_amount(int16_t semitones)
{
  cutoff_mod_amount = semitones;
  filter_needs_update = true;
}


void set_filter_mode(filter_mode_t mode)
{
  /* Mode changing is accomplished by modifying the instruction that
   * moves one of the state-variable filter's outputs into r3. */
  filter_mode = mode;
  switch (mode) {
    case FILTER_OFF:
      /* This is fragile!! Turning off the filter injects a branch
       * instruction with a relative offset. If the filter code in kernel.S
       * is changed, this constant *must* be updated appropriately! */
      filter_bypass_control = 0xe023; /* b filter_bypass */
      return;
    default:
      break;
    case FILTER_LOWPASS:
      filter_mode_control = 0x1c0b; /* mov r3, r1 */
      break;
    case FILTER_HIGHPASS:
      filter_mode_control = 0x1c1b; /* mov r3, r3 */
      break;
    case FILTER_BANDPASS:
      filter_mode_control = 0x1c03; /* add r3, r1 */
      break;
    /* notch would be 0x185b (add r3, r1) */
  }
  filter_bypass_control = 0x46c0; /* nop */
}


filter_mode_t get_filter_mode(void)
{
  return filter_mode;
}


void set_keyboard_tracking(_Bool track)
{
  keyboard_tracking = track;
  filter_needs_update = true;
}


_Bool get_keyboard_tracking(void)
{
  return keyboard_tracking;
}


void set_lfo_rate(uint8_t val)
{
  lfo_freq = lfofreqtable[val];
}


void set_lfo_shape(lfo_shape_t shape)
{
  lfo_shape = shape;
}


lfo_shape_t get_lfo_shape(void)
{
  return lfo_shape;
}


void set_pitch_mod_amount(int16_t semitones)
{
  pitch_mod_amount = semitones;
  freq_needs_update = true;
}


void set_filter_cutoff_mod_sources(uint8_t sources)
{
  lfo_affects_cutoff = ((sources & MOD_SRC_LFO) != 0);
  env_affects_cutoff = ((sources & MOD_SRC_ENV) != 0);
  if (!lfo_affects_cutoff && !lfo_affects_pitch) {
    lfo_value = 0;
  }
  filter_needs_update = true;
}


void set_pitch_mod_sources(uint8_t sources)
{
  lfo_affects_pitch = ((sources & MOD_SRC_LFO) != 0);
  env_affects_pitch = ((sources & MOD_SRC_ENV) != 0);
  if (!lfo_affects_cutoff && !lfo_affects_pitch) {
    lfo_value = 0;
  }
  freq_needs_update = true;
}


void set_legato(_Bool val)
{
  legato = val;
}


_Bool get_legato(void)
{
  return legato;
}


/**
 * Update envelope, LFO, and glide.
 */
void TIMER32_0_IRQHandler(void)
{
  /* Clear the interrupt flag */
  TMR_TMR32B0IR = TMR_TMR32B0IR_MR0;

  /* Update glide */
  if (glide_rate) {
    /* Gliding up? */
    if (current_pitch < dest_pitch) {
      current_pitch += glide_rate;
      if (current_pitch > dest_pitch) {
        current_pitch = dest_pitch;
      }
      freq_needs_update = true;
    }
    /* Gliding down? */
    else if (current_pitch > dest_pitch) {
      current_pitch -= glide_rate;
      if (current_pitch < dest_pitch) {
        current_pitch = dest_pitch;
      }
      freq_needs_update = true;
    }
  }

  /* Update envelope
   * Attack and release have exponential responses
   * When echo or repeat is enabled, start the retrigger well before the envelope
   * reaches zero to quicken the gaps between triggers */
  int32_t delta = 0;
  uint16_t silent_threshold = (echoes_left || sustain_mode == SUSTAIN_REPEAT) ? 0x1000 : 0x00FF;
  switch (envelope_stage) {
    case ENV_OFF:
      envelope = 0;
      break;
    case ENV_ATTACK:
      echoes_left = echoes;
      delta = ((0xFFFF-envelope)*attack) >> 15;
      envelope += delta;
      /* phase ends once rise completes */
      if (delta == 0 || envelope >= 0xFF00) {
        envelope = 0xFFFF;
        switch (sustain_mode) {
          case SUSTAIN_ON: envelope_stage = ENV_SUSTAIN; break;
          case SUSTAIN_OFF: envelope_stage = ENV_RELEASE; break;
          case SUSTAIN_REPEAT: envelope_stage = ENV_REPEAT; break;
        }
      }
      break;
    case ENV_SUSTAIN:
      envelope = 0xFFFF;
      break;
    case ENV_RELEASE:
      delta = (envelope*release) >> 15;
      envelope -= delta;
      /* phase ends once fall completes */
      if (delta == 0 || envelope <= silent_threshold) {
        if (!echoes_left) {
          envelope = 0;
          envelope_stage = ENV_OFF;
        } else {
          echoes_left--;
          envelope = 0xFFFF;
        }
      }
      break;
    case ENV_REPEAT:
      delta = (envelope*release) >> 15;
      envelope -= delta;
      if (delta == 0 || envelope <= silent_threshold) {
        envelope = 0;
        envelope_stage = ENV_ATTACK;
      }
      break;
  }

  uint8_t vol = envelope >> (8+echoes-echoes_left);
  volume_control = 0x2000|vol;
  silence = (vol>0) ? 0x46c0 : 0xe04c;

  /* Update LFO */
  if (lfo_freq > 0) {
    uint32_t phase_before = lfo_phase;
    lfo_phase += lfo_freq;

    switch (lfo_shape) {
      case LFO_TRIANGLE:
        lfo_value = (lfo_phase << 1) ^ -(lfo_phase >> 15);
        break;
      case LFO_SAWTOOTH:
        lfo_value = lfo_phase;
        break;
      case LFO_SQUARE:
        lfo_value = -(lfo_phase >> 15);
        break;
      case LFO_RANDOM:
      default:
        /* Generate a new random value when the phase accumulator wraps */
        if ((phase_before & (1<<15)) != (lfo_phase & (1<<15))) {
          lfo_value = rand8() << 8;
        }
        break;
    }

    if (lfo_affects_cutoff) {
      filter_needs_update = true;
    }
    if (lfo_affects_pitch) {
      freq_needs_update = true;
    }
  }
  /* When frequency is set to 0, reset the LFO state */
  else if (lfo_value != 0) {
    lfo_value = 0;
    lfo_phase = 0;
    if (lfo_affects_cutoff) {
      filter_needs_update = true;
    }
    if (lfo_affects_pitch) {
      freq_needs_update = true;
    }
  }

  /* Update modulation envelope */
  if (env_affects_cutoff || env_affects_pitch) {
    switch (mod_envelope_stage) {
      case ENV_OFF:
        mod_envelope = 0;
        break;
      case ENV_ATTACK:
        mod_envelope += mod_attack;
        if (mod_envelope >= 0xFFFF) {
          mod_envelope = 0xFFFF;
          mod_envelope_stage = ENV_RELEASE;
        }
        break;
      case ENV_RELEASE:
        mod_envelope -= mod_release;
        if (mod_envelope <= 0) {
          mod_envelope = 0;
          mod_envelope_stage = ENV_OFF;
        }
        break;
      default:
        break;
    }

    if (mod_envelope_stage != ENV_OFF) {
      if (env_affects_cutoff) {
        filter_needs_update = true;
      }
      if (env_affects_pitch) {
        freq_needs_update = true;
      }
    }
  } else {
    mod_envelope_stage = ENV_OFF;
    mod_envelope = 0;
  }

  /* Update oscillator frequencies */
  if (freq_needs_update) {
    freq_needs_update = false;
    update_frequencies();
    if (keyboard_tracking) {
      filter_needs_update = true;
    }
  }

  /* Update filter parameters */
  if (filter_needs_update) {
    filter_needs_update = false;

    /* Convert cutoff pitch from a note number to a parameter value.
     * Add keyboard tracking; middle C (note 60) is used as the base note. */
    int32_t cutoff = cutoff_pitch;
    if (keyboard_tracking) {
      cutoff += current_pitch_post_mod - (60 << 9);
    }

    /* Add LFO modulation */
    if (lfo_affects_cutoff) {
      int32_t mod_amount = lfo_value * cutoff_mod_amount;
      cutoff += (mod_amount >> 16);
    }

    /* Add envelope modulation */
    if (env_affects_cutoff) {
      int32_t mod_amount = mod_envelope * cutoff_mod_amount;
      cutoff += (mod_amount >> 16);
    }

    uint32_t fc = 0;
    if (cutoff < 0) {
      fc = 0;
    } else if (cutoff >= ((NUM_CUTOFF_ENTRIES-1) << 9)) {
      fc = 0xFFFF;
    } else {
      /* Interpolate between entries in the cutoff table */
      uint8_t basenote = cutoff >> 9;
      uint32_t fracnote = cutoff & ((1 << 9)-1);
      uint16_t basefreq = cutofftable[basenote];
      uint16_t delta = (cutofftable[basenote+1]-basefreq) >> 9;
      fc = basefreq + fracnote*delta;
    }
    
    /* Apply correction to the filter parameters to prevent instability
     * http://courses.cs.washington.edu/courses/cse490s/11au/Readings/Digital_Sound_Generation_2.pdf
     * Q = min(Qc, 2-Fc)
     * F = Fc*(1.75 - 0.75*Q*Fc)
     * (note: the document suggests F=Fc*(1.85-0.85*Q*Fc), but multiplying by 0.75 is easier) */
    uint32_t two_minus_fc = 0x20000-fc;
    filter_q = (uncorrected_q < two_minus_fc) ? uncorrected_q : two_minus_fc;

    uint32_t q_fc = (uncorrected_q>>1)*(fc>>1); /* multiply Q and Fc without overflowing */
    q_fc -= q_fc >> 2;  /* multiply by 0.75 (subtract 1/4) */
    q_fc >>= 14;
    q_fc = 0x1c000 - q_fc; /* subtract from 1.75 */
    filter_cutoff = ((fc>>1)*(q_fc>>1)) >> 14;
  }
}
