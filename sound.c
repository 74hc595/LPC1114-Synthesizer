#include "sound.h"
#include "hardware.h"
#include <stdbool.h>

extern uint32_t notetable[128];

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
  uint16_t __pad1[3];
  uint8_t volume;
  uint8_t __pad2;
  union {
    uint16_t waveform_code[4];
    struct {
      uint16_t __pad3;
      uint8_t duty;
      uint8_t __pad4;
      uint16_t __pad5[2];
    };
  };
  uint16_t __pad6;
} oscillator_control_t;


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
 * Thus, the full pitch range is a piecewise linear representation of an exponential.
 */

/* The current pitch value, including glide but not including pitch bend. */
static uint16_t current_pitch = 0;

/* The "destination" pitch; the pitch that the glide is ascending/descending toward.
 * If glide is off, this will be identical to current_pitch. */
static uint16_t dest_pitch = 0;

/* Glide rate. If 0, glide is off and pitch changes are instantaneous. */
static int16_t glide_rate = 100;

/* Pitch bend amount. Signed. */
static int16_t pitch_bend = 0;

/* Additional coarse tuning offsets applied to individual oscillators. */
static int16_t tuning_amounts[NUM_OSCILLATORS];

/* Additional fine tuning amounts applied to individual oscillators. */
static int16_t detune_amounts[NUM_OSCILLATORS];

/* 4-element queue used to keep track of which keys are being held down.
 * Each element is a MIDI note number. */
static byte_queue_t playing_notes = {0};

/* Indicates that the oscillator frequencies need to be recomputed.
 * If true, frequencies will be updated on the next tick of the low-frequency timer. */
static volatile _Bool freq_needs_update = false;

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
  osc_update_base[oscnum].waveform_code[2] = 0x0409;      /* lsl r1, #16 */
  osc_update_base[oscnum].waveform_code[3] = 0x404a;      /* eor r2, r1 */
}


/**
 * Modifies the waveform generation code to generate a sawtooth wave for the
 * specified voice.
 */
static inline void oscillator_set_sawtooth(int oscnum)
{
#if 0
  osc_update_base[oscnum].waveform_code[0] = 0x4252; /* neg r2, 2 */
  osc_update_base[oscnum].waveform_code[1] = 0x13d2; /* asr r2, #15 */
  osc_update_base[oscnum].waveform_code[2] = 0x434a; /* mul r2, r1 */ 
  osc_update_base[oscnum].waveform_code[3] = 0x46c0; /* nop */   
#else
  osc_update_base[oscnum].waveform_code[0] = 0x13d2; /* asr r2, #15 */
  osc_update_base[oscnum].waveform_code[1] = 0x434a; /* mul r2, r1 */ 
  osc_update_base[oscnum].waveform_code[2] = 0x46c0; /* nop */   
  osc_update_base[oscnum].waveform_code[3] = 0x46c0; /* nop */
#endif
}


/**
 * Modifies the waveform generation code to generate a ramp (reverse sawtooth)
 * wave for the specified voice.
 */
static inline void oscillator_set_lofi_sawtooth(int oscnum)
{
  osc_update_base[oscnum].waveform_code[0] = 0x1792; /* asr r2, #30 */
  osc_update_base[oscnum].waveform_code[1] = 0x03d2; /* lsl r2, #15 */
  osc_update_base[oscnum].waveform_code[2] = 0x434a; /* mul r2, r1 */ 
  osc_update_base[oscnum].waveform_code[3] = 0x46c0; /* nop */
}



void sound_init(void)
{
//  int i;
//  for (i = 0; i < NUM_OSCILLATORS; i++) {
//    oscillators[i].freq = TEST_FREQ;
//    osc_update_base[i].volume = 255;
//  }
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
//      /* is the oscillator already in sawtooth mode? if not, switch */
//      if (osc_update_base[i].waveform_code[0] != 0x13d2) {
        oscillator_set_sawtooth(i);
//      }
    }
  }
}


void sound_set_lofi_sawtooth(uint8_t oscmask)
{
  int i;
  for (i = 0; i < NUM_OSCILLATORS; i++) {
    if (oscmask & (1 << i)) {
//      /* is the oscillator already in sawtooth mode? if not, switch */
//      if (osc_update_base[i].waveform_code[0] != 0x13d2) {
        oscillator_set_lofi_sawtooth(i);
//      }
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
  if (playing_notes.count == 1) {
    int i;
    current_pitch = dest_pitch;
    for (i = 0; i < NUM_OSCILLATORS; i++) {
      oscillators[i].phase = 0;
      osc_update_base[i].volume = 255;
    }
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
    for (i = 0; i < NUM_OSCILLATORS; i++) {
      osc_update_base[i].volume = 0;
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


/**
 * Update envelopes, LFO, and glide.
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

  /* Update oscillator frequencies */
  if (freq_needs_update) {
    freq_needs_update = false;
    update_frequencies();
  }
}
