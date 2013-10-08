#include "sound.h"

#define NUM_OSCILLATORS 4

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
  osc_update_base[oscnum].waveform_code[0] = 0x13d2; /* asr r2, #15 */
  osc_update_base[oscnum].waveform_code[1] = 0x434a; /* mul r2, r1 */ 
  osc_update_base[oscnum].waveform_code[2] = 0x46c0; /* nop */   
  osc_update_base[oscnum].waveform_code[3] = 0x46c0; /* nop */
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
  int i;
  for (i = 0; i < NUM_OSCILLATORS; i++) {
    oscillators[i].freq = TEST_FREQ;
    osc_update_base[i].volume = 255;
  }
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




void sound_set_detune(uint8_t num_oscillators, uint8_t spread)
{
  int i;
  uint32_t phase = 0;
  uint32_t freq = TEST_FREQ;
  if (num_oscillators == 1) {
    for (i = 0; i < NUM_OSCILLATORS; i++) {
      oscillators[i].phase = phase;
      oscillators[i].freq = freq;
    }
  } else if (num_oscillators == 2) {
    oscillators[0].freq = freq;
    oscillators[1].freq = freq;
    oscillators[2].freq = freq + 1000*spread;
    oscillators[3].freq = freq + 1000*spread;
  } else if (num_oscillators == 3) {
    oscillators[0].freq = freq;
    oscillators[1].freq = freq;
    oscillators[2].freq = freq + 1000*spread;
    oscillators[3].freq = freq - 1000*spread;
  } else if (num_oscillators == 4) {
    oscillators[0].freq = freq;
    oscillators[1].freq = freq + 1000*spread;
    oscillators[2].freq = freq - 1000*spread;
    oscillators[3].freq = freq + 2000*spread;
  }
}

