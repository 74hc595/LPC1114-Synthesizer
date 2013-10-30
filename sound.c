#include "sound.h"
#include "hardware.h"
#include <stdbool.h>

extern uint32_t notetable[128];
extern uint16_t envtable[256];

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

typedef enum {
  SUSTAIN_ON,
  SUSTAIN_OFF,
  SUSTAIN_REPEAT
} sustain_mode_t;

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

/* Envelope attack time constant. */
static uint16_t attack = 0;

/* Envelope release time constant. */
static uint16_t release = 0;

/* Envelope value. Only the upper byte is used for amplitude. */
static uint16_t envelope = 0;

/* Sustain mode: off, on, or repeat. */
static sustain_mode_t sustain_mode = SUSTAIN_ON;

/* Number of "echoes," i.e. times the release phase repeats after the
 * normal release ends. */
static uint8_t echoes = 0;

/* Number of echoes currently left. */
static uint8_t echoes_left = 0;

/* Envelope stage. */
static env_stage_t envelope_stage = ENV_OFF;

extern volatile oscillator_state_t oscillators[4];
extern volatile oscillator_control_t osc_update_base[4];


#define TEST_FREQ 2000000

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
  osc_update_base[oscnum].waveform_code[0] = 0x11d2; /* asr r2, #15 */
  osc_update_base[oscnum].waveform_code[1] = 0x46c0; /* nop */
  osc_update_base[oscnum].waveform_code[2] = 0x46c0; /* nop */
}


void sound_init(void)
{
}


void sound_set_duty_cycle(uint8_t val, uint8_t oscmask)
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


void sound_set_sawtooth(uint8_t oscmask)
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
  uint16_t rootpitch = current_pitch + pitch_bend;
  for (i = 0; i < NUM_OSCILLATORS; i++) {
    uint16_t note = rootpitch + tuning_amounts[i] + detune_amounts[i];
    uint8_t basenote = note >> 9;
    uint32_t fracnote = note & ((1 << 9)-1);
    uint32_t basefreq = notetable[basenote];
    uint32_t delta = (notetable[basenote+1]-basefreq) >> 9;
    oscillators[i].freq = basefreq + fracnote*delta;
  }  
}  


void sound_set_detune(uint8_t mode, uint8_t val)
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


void sound_set_oscillator_tuning(int8_t note_offsets[NUM_OSCILLATORS])
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

  /* if this is the first note being played, reset the oscillator phases and glide */
  if (playing_notes.count == 1 || (sustain_mode == SUSTAIN_OFF)) {
    int i;
    current_pitch = dest_pitch;
    if (envelope_stage == ENV_OFF) {
      for (i = 0; i < NUM_OSCILLATORS; i++) {
        oscillators[i].phase = 0;
      }
    }
    envelope_stage = ENV_ATTACK;
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


void set_glide(glide_t glide)
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
  TMR_TMR16B1MR0 = 255-vol;

  /* Update oscillator frequencies */
  if (freq_needs_update) {
    freq_needs_update = false;
    update_frequencies();
  }
}
