#include "hardware.h"
#include "sound.h"
#include <stdbool.h>

/* if true, use the UART for debug output instead of MIDI */
#define DEBUG_LOGGING 0

#define NUM_KNOBS 8
typedef struct
{
  uint8_t value;
  void (*update_fn)(uint8_t);
} knob_t;
static knob_t knobs[NUM_KNOBS] = {{0}};


enum {
  SW_GLIDE,
  SW_CUTOFFMOD0,
  SW_CUTOFFMOD1,
  SW_PITCHMOD0,
  SW_PITCHMOD1,
  SW_ENVMODE0,
  SW_ENVMODE1,
  SW_LFOSHAPE,
  SW_PITCHPGM,
  SW_CHORDPGM,
  SW_ECHO,
  NUM_SWITCHES
};

typedef struct
{
  void (*pressed_fn)(void);
  void (*released_fn)(void);
  uint8_t state;
} switch_t;
static switch_t switches[NUM_SWITCHES] = {{0}};


#if DEBUG_LOGGING
/* for testing */
static char hexchars[16] = "0123456789ABCDEF";
void outhex8(uint8_t x)
{
  uart_send_byte(hexchars[x >> 4]);
  uart_send_byte(hexchars[x & 0xF]);
}


static void outhex16(uint16_t x)
{
  uart_send_byte(hexchars[x >> 12]);
  uart_send_byte(hexchars[(x >> 8) & 0xF]);
  uart_send_byte(hexchars[(x >> 4) & 0xF]);
  uart_send_byte(hexchars[x & 0xF]);
}


static void printgraph(uint8_t x)
{
  int i;
  for (i = 0; i < x; i++) {
    uart_send_byte(' ');
  }
  uart_send_byte('*');
  uart_send_byte('\n');
}
#endif


static void update_waveform(uint8_t knobval)
{
#if DEBUG_LOGGING
  uart_send_byte('w');
  outhex8(knobval);
  uart_send_byte('\n');
#endif

  /* First half: adjust duty cycle from 0% to 50%, pulse wave on all oscillators */
  if (knobval <= 127) {
    set_duty_cycle((127-knobval)<<1, 0xF);
  }
  /* Second half: pulse wave on 2 oscillators and sawtooth on the other two */
  else if (knobval < 0xFB) {
    set_duty_cycle(knobval-128, 0x3);
    set_sawtooth(0xC);
  }
  /* All the way to the right: sawtooth on all oscilators */
  else {
    set_sawtooth(0xF);
  }
}


static void update_detune(uint8_t knobval)
{
#if DEBUG_LOGGING
  uart_send_byte('d');
  outhex8(knobval >> 6);
  uart_send_byte(',');
  outhex8(knobval & 0x3f);
  uart_send_byte('\n');
#endif

  set_detune(knobval >> 6, knobval & 0x3f);
}


static void update_attack(uint8_t knobval)
{
#if DEBUG_LOGGING
  uart_send_byte('a');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  set_attack(knobval);
}


static void update_release(uint8_t knobval)
{
#if DEBUG_LOGGING
  uart_send_byte('r');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  set_release(knobval);
}


static void update_cutoff(uint8_t knobval)
{
#if DEBUG_LOGGING
  uart_send_byte('c');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  set_filter_cutoff(knobval);
}


static void update_resonance(uint8_t knobval)
{
#if DEBUG_LOGGING
  uart_send_byte('q');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  set_filter_resonance((0xFC-knobval) << 9);
}


static void update_cutoff_mod_amount(uint8_t knobval)
{
#if DEBUG_LOGGING
  uart_send_byte('c');
  uart_send_byte('m');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  int8_t bipolarval = knobval-128;
  set_filter_cutoff_mod_amount(bipolarval << 8);
}


static void update_lfo_rate(uint8_t knobval)
{
#if DEBUG_LOGGING
  uart_send_byte('l');
  uart_send_byte('r');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  set_lfo_rate(knobval);
}


static _Bool shift = false;
/* Only one programming mode can be active at a time. */
static _Bool chord_pgm_active = false;
static _Bool pitch_pgm_active = false;
static uint8_t note_input_idx = 0;
static int8_t note_inputs[NUM_OSCILLATORS];
static uint8_t glide_preset = 0;

static void env_mode_changed(void)
{
  sustain_mode_t mode = switches[SW_ENVMODE0].state | (switches[SW_ENVMODE1].state << 1);
  set_sustain_mode(mode);
}


static void echo_pressed(void)
{
  if (shift) {
    chord_pgm_active = false;
  }

  // for debugging
  SCB_AIRCR = SCB_AIRCR_VECTKEY_VALUE | SCB_AIRCR_SYSRESETREQ;
  while(1) {}

  uint8_t e = get_echoes();
  switch (e) {
    case 0: e = 1; break;
    case 1: e = 2; break;
    case 2: e = 3; break;
    default: e = 0; break;
  }
  set_echoes(e);
}


static void lfo_shape_pressed(void)
{
  if (!shift) {
    lfo_shape_t shape = get_lfo_shape();
    shape = (shape+1) % NUM_LFO_SHAPES;
    set_lfo_shape(shape);
  } else {
    chord_pgm_active = false;
    filter_mode_t mode = get_filter_mode();
    mode = (mode+1) % NUM_FILTER_MODES;
    set_filter_mode(mode);
  }
}


static void glide_pressed(void)
{
  if (shift) {
    chord_pgm_active = false;
  }

  glide_preset = (glide_preset+1) % NUM_GLIDE_PRESETS;
  set_glide_preset(glide_preset);
}


static void cutoff_mod_changed(void)
{
  uint8_t val = switches[SW_CUTOFFMOD0].state | (switches[SW_CUTOFFMOD1].state << 1);
  set_filter_cutoff_mod_sources(val);
}


static void pitch_mod_changed(void)
{
  uint8_t val = switches[SW_PITCHMOD0].state | (switches[SW_PITCHMOD1].state << 1);
  set_pitch_mod_sources(val);
}


static void chord_pgm_finish(void)
{
  /* compute all note positions relative to the first */
  int i;
  int8_t first_note = note_inputs[0];
  for (i = 0; i < NUM_OSCILLATORS; i++) {
    if (i < note_input_idx) {
      note_inputs[i] -= first_note;
    } else {
      note_inputs[i] = 0;
    }
  }
  set_oscillator_tuning(note_inputs);
  chord_pgm_active = false;
}


static void chord_pgm_pressed(void)
{
  /* also doubles as a shift key */
  shift = true;

  /* the second time the button is pressed, exit programming mode */
  if (chord_pgm_active) {
    chord_pgm_finish();
    return;
  }
  
  /* if the other programming mode is active, cancel it */
  if (pitch_pgm_active) {
    pitch_pgm_active = false;
  }

  chord_pgm_active = true;
  note_input_idx = 0;
}


static void chord_pgm_released(void)
{
  shift = false;
}


static void pitch_pgm_finish(void)
{
  /* compute the difference between the two notes and use it
   * as the pitch modulation amount */
  int8_t mod_amount = note_inputs[1] - note_inputs[0];
  set_pitch_mod_amount(mod_amount << 9);
  pitch_pgm_active = false;
}


static void pitch_pgm_pressed(void)
{
  if (!shift) {
    /* the second time the button is pressed, exit programming mode */    
    if (pitch_pgm_active) {
      pitch_pgm_finish();
      return;
    }

    /* if the other programming mode is active, cancel it */    
    if (chord_pgm_active) {
      chord_pgm_active = false;
    }

    pitch_pgm_active = true;
    note_input_idx = 0;

  } else {
    chord_pgm_active = false;
    set_keyboard_tracking(!get_keyboard_tracking());
  }
}


static void add_note_to_input(int8_t note)
{
  note_inputs[note_input_idx] = note;
  note_input_idx++;
  if (chord_pgm_active && note_input_idx == NUM_OSCILLATORS) {
    chord_pgm_finish();
  } else if (pitch_pgm_active && note_input_idx == 2) {
    pitch_pgm_finish();
  }
}


#if 0
static volatile uint8_t chprog_active = 0;
static uint8_t chprog_oscnum = 0;
static int8_t chprog_notes[NUM_OSCILLATORS];

static void chprog_finish(void)
{
  chprog_active = 0;
  gpio1_set_pin_low(9);

  /* compute all note positions relative to the first */
  int i;
  int8_t first_note = chprog_notes[0];
  for (i = 0; i < NUM_OSCILLATORS; i++) {
    if (i < chprog_oscnum) {
      chprog_notes[i] -= first_note;
    } else {
      chprog_notes[i] = 0;
    }
  }

  sound_set_oscillator_tuning(chprog_notes);
}


static void button_pressed(void)
{
  /* start chord programming */
  if (!chprog_active) {
    chprog_oscnum = 0;
    chprog_active = 1;
    gpio1_set_pin_high(9);    
  }
  /* end chord programming */
  else {
    chprog_finish();
  }
}


static void chprog_add_note(uint8_t note)
{
  chprog_notes[chprog_oscnum] = note;
  chprog_oscnum++;
  if (chprog_oscnum == NUM_OSCILLATORS) {
    chprog_finish();
  }
}
#endif


int main(void)
{
  /* FCLKIN = 25MHz
   * M = 2, so clock frequency is M*FCLKIN = 50MHz
   * P = 2
   * FCCO = 2*P*FCLKOUT = 200MHz */
  cpu_pll_setup(SCB_PLLCTRL_MSEL_2, SCB_PLLCTRL_PSEL_2);

  /* Enable pullup resistor on PIO0_1 */
  IOCON_PIO0_1 = IOCON_PIO0_1_MODE_PULLUP;

  //cpu_enable_clkout();
  adc_init();
  spi_init();
  
  /* PIO1_8 is used to control the 4053 analog mux */
  GPIO_GPIO1DIR |= (1<<8)|(1<<9);
  gpio1_set_pin_low(8);

  /* set up the knobs */
  knobs[0].update_fn = update_release;
  knobs[1].update_fn = update_detune;
  knobs[2].update_fn = update_waveform;
  knobs[3].update_fn = update_attack;
  knobs[4].update_fn = update_cutoff;
  knobs[5].update_fn = update_resonance;
  knobs[6].update_fn = update_cutoff_mod_amount;
  knobs[7].update_fn = update_lfo_rate;

  /* set up the switches */
  switches[SW_ENVMODE0].pressed_fn = env_mode_changed;
  switches[SW_ENVMODE0].released_fn = env_mode_changed;
  switches[SW_ENVMODE1].pressed_fn = env_mode_changed;
  switches[SW_ENVMODE1].released_fn = env_mode_changed;
  switches[SW_CUTOFFMOD0].pressed_fn = cutoff_mod_changed;
  switches[SW_CUTOFFMOD0].released_fn = cutoff_mod_changed;
  switches[SW_CUTOFFMOD1].pressed_fn = cutoff_mod_changed;
  switches[SW_CUTOFFMOD1].released_fn = cutoff_mod_changed;
  switches[SW_PITCHMOD0].pressed_fn = pitch_mod_changed;
  switches[SW_PITCHMOD0].released_fn = pitch_mod_changed;
  switches[SW_PITCHMOD1].pressed_fn = pitch_mod_changed;
  switches[SW_PITCHMOD1].released_fn = pitch_mod_changed;
  switches[SW_ECHO].pressed_fn = echo_pressed;
  switches[SW_LFOSHAPE].pressed_fn = lfo_shape_pressed;
  switches[SW_GLIDE].pressed_fn = glide_pressed;
  switches[SW_CHORDPGM].pressed_fn = chord_pgm_pressed;
  switches[SW_CHORDPGM].released_fn = chord_pgm_released;
  switches[SW_PITCHPGM].pressed_fn = pitch_pgm_pressed;

  sound_init();
  pwm_init(254);
  systick_init(200);
  timer32_init(200000);

#if !DEBUG_LOGGING
  uart_init(BAUD(31250, 50000000));
#else
  uart_init(BAUD(115200, 50000000));
#endif
  
  /* Allow use of PIO0_0 without resetting the CPU
   * We do this here because it seems like setting it at the
   * start of main prevents reset from working after serial programming...
   * oh well. */
  IOCON_nRESET_PIO0_0 = IOCON_nRESET_PIO0_0_FUNC_GPIO;

//  note_on(69);
  
  while (1) {
    /* read the knobs */
    uint8_t k, i;
    for (k = 0; k < NUM_KNOBS; k++) {
      /* discard the lowest 2 bits from the ADC input to reduce noise
       * use an exponential moving average with alpha=0.25 to prevent
       * "dither" between values */
      knob_t *knob = knobs+k;

      /* set the mux control line high when reading the last 3 channels */
      uint8_t ch = k;
      if (k >= 6) {
        ch -= 3;
        gpio1_set_pin_high(8);
      } else {
        gpio1_set_pin_low(8);
      }

      uint8_t input = adc_read_channel(ch) >> 2;
      int16_t delta = input - knob->value;
      uint8_t newval = knob->value + (delta >> 2);
      if (newval != knob->value) {
        knob->update_fn(newval);
        knob->value = newval;
      }
    }

    /* read the buttons/switches */
    uint16_t input = SSP_SSP0DR >> 7;
    input |= (gpio_pins(GPIO0, 0b11) << 9);
    for (i = 0; i < NUM_SWITCHES; i++, input >>= 1) {
      switch_t *sw = switches+i;
      uint8_t state = !(input & 1);
      if (state != sw->state) {
        sw->state = state;
        void (*handler)(void) = (state) ? sw->pressed_fn : sw->released_fn;
        if (handler) {
          handler();
        }
      }
    }
  }
  return 0;
}



/***** MIDI *****/
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


void HardFault_Handler(void)
{
  while (1) {}
}
