#include "hardware.h"
#include "sound.h"
#include "tables.h"
#include <stdbool.h>

/* if true, use the UART for debug output instead of MIDI */
#define DEBUG_LOGGING     0
#define LOG_KNOB_VALUES   0
#define LOG_SWITCH_EDGES  0

/* LED pins.
 * Bits 0-3 indicate pin number.
 * Bit 4 indicates port number. (0 or 1) */
#define LED_ANODE_0       7
#define LED_ANODE_1       10
#define LED_ANODE_2       (5|16)
#define NUM_LED_COLUMNS   3
#define LED_GREEN         (1<<LED_ANODE_0)
#define LED_BLUE          (1<<LED_ANODE_1)
#define LED_RED           (1<<LED_ANODE_2)
#define LED_CHORDPGM      (1<<LED_ANODE_0)
#define LED_PITCHPGM      (1<<LED_ANODE_1)
#define LED_MODENV        (1<<LED_ANODE_2)
#define LED_AMPENV        (1<<LED_ANODE_1)
#define LED_GLIDE         (1<<LED_ANODE_2)

enum {
  KNOB_RELEASE,
  KNOB_DETUNE,
  KNOB_WAVEFORM,
  KNOB_ATTACK,
  KNOB_CUTOFF,
  KNOB_RESONANCE,
  KNOB_CUTOFFMOD,
  KNOB_LFORATE,
  NUM_KNOBS
};

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
  SW_ENVSELECT,
  NUM_SWITCHES
};


typedef struct
{
  void (*pressed_fn)(void);
  void (*released_fn)(void);
  uint8_t state;
  int8_t debounce_count;
} switch_t;
static switch_t switches[NUM_SWITCHES] = {{0}};

static _Bool mod_env_select = false;


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


static void outhex32(uint32_t x)
{
  outhex16(x >> 16);
  outhex16(x & 0xFFFF);
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
#if LOG_KNOB_VALUES
  uart_send_byte('w');
  outhex8(knobval);
  uart_send_byte('\n');
#endif

  /* First half: adjust duty cycle from 0% to 50%, pulse wave on all oscillators */
  if (knobval <= 127) {
    set_oscillator_waveforms(0b0000, (127-knobval)<<1);
  }
  /* Second half: pulse wave on 2 oscillators and sawtooth on the other two */
  else if (knobval < 0xFB) {
    set_oscillator_waveforms(0b0011, knobval-128);
  }
  /* All the way to the right: sawtooth on all oscilators */
  else {
    set_oscillator_waveforms(0b1111, 128);
  }
}


static void update_detune(uint8_t knobval)
{
#if LOG_KNOB_VALUES
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
#if LOG_KNOB_VALUES
  uart_send_byte('a');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  if (!mod_env_select) {
    set_attack(knobval);
  } else {
    set_mod_attack(knobval);
  }
}


static void update_release(uint8_t knobval)
{
#if LOG_KNOB_VALUES
  uart_send_byte('r');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  if (!mod_env_select) {
    set_release(knobval);
  } else {
    set_mod_release(knobval);
  }
}


static void update_cutoff(uint8_t knobval)
{
#if LOG_KNOB_VALUES
  uart_send_byte('c');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  /* translate a value between 0 and 255 to a fractional value
   * between 0 and the maximum cutoff value */
  int32_t val = (knobval*num_cutoff_entries) << 1;
  set_filter_cutoff(val);
}


static void update_resonance(uint8_t knobval)
{
#if LOG_KNOB_VALUES
  uart_send_byte('q');
  outhex8(knobval);
  uart_send_`byte('\n');
#endif
  set_filter_resonance((0xFC-knobval) << 9);
}


static void update_cutoff_mod_amount(uint8_t knobval)
{
#if LOG_KNOB_VALUES
  uart_send_byte('c');
  uart_send_byte('m');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  int8_t bipolarval = knobval-128;
  set_filter_cutoff_mod_amount(bipolarval << 8);
  set_pulse_width_mod_amount(bipolarval);
}


static void update_lfo_rate(uint8_t knobval)
{
#if LOG_KNOB_VALUES
  uart_send_byte('l');
  uart_send_byte('r');
  outhex8(knobval);
  uart_send_byte('\n');
#endif
  set_lfo_rate(knobval);
}


static _Bool shift = false;
static _Bool reset_button_held = false;
static _Bool shift_button_held = false;
/* Only one programming mode can be active at a time. */
_Bool chord_pgm_active = false;
_Bool pitch_pgm_active = false;
static uint8_t note_input_idx = 0;
static int8_t note_inputs[NUM_OSCILLATORS];
static uint8_t glide_preset = 0;
uint32_t ledcolumns[3] = {0, 0, 0};
int16_t glide_led_blink_pattern = 0xFFFF;
static const int16_t glide_led_blink_patterns[NUM_GLIDE_PRESETS] = {
  0b1111111111111111,
  0b1111111111111111,
  0b1111111111110101,
  0b1111111111010101
};

static void update_leds(void)
{
  /* RGB LED is column 0 */
  uint32_t col = 0;
  if (!shift) {
    switch (get_lfo_shape()) {
      case LFO_TRIANGLE:
        col = LED_RED;
        break;
      case LFO_SAWTOOTH:
        col = LED_GREEN;
        break;
      case LFO_SQUARE:
        col = LED_BLUE;
        break;
      case LFO_RANDOM:
      default:
        col = LED_RED|LED_GREEN|LED_BLUE;
        break;
    }
  } else {
    switch (get_filter_mode()) {
      case FILTER_OFF:
      default:
        break;
      case FILTER_LOWPASS:
        col = LED_GREEN|LED_BLUE;
        break;
      case FILTER_HIGHPASS:
        col = LED_RED|LED_BLUE;
        break;
      case FILTER_BANDPASS:
        col = LED_RED|LED_GREEN;
        break;
    }
  }
  ledcolumns[0] = col;

  /* Column 1 */
  col = 0;
  if (chord_pgm_active) {
    col |= LED_CHORDPGM;
  }
  if (pitch_pgm_active || (shift && get_keyboard_tracking())) {
    col |= LED_PITCHPGM;
  }
  if (mod_env_select || shift) {
    col |= LED_MODENV;
  }
  ledcolumns[1] = col;

  /* Column 2 */
  col = 0;
  if (!mod_env_select || shift) {
    col |= LED_AMPENV;
  }
  if ((!shift && glide_preset && (glide_led_blink_pattern & 1)) || (shift && get_legato())) {
    col |= LED_GLIDE;
  }
  ledcolumns[2] = col;
}


static void env_mode_changed(void)
{
  sustain_mode_t mode = switches[SW_ENVMODE0].state | (switches[SW_ENVMODE1].state << 1);
  set_sustain_mode(mode);
}


static void env_select_pressed(void)
{
  /* Unshifted: select amplitude envelope or modulation envelope
   * Shifted: select next echo mode */
  reset_button_held = true;

  if (!shift) {
    mod_env_select = !mod_env_select;
  } else {
    chord_pgm_active = false;

    uint8_t e = get_echoes();
    switch (e) {
      case 0: e = 1; break;
      case 1: e = 2; break;
      case 2: e = 3; break;
      default: e = 0; break;
    }
    set_echoes(e);
  }
  update_leds();
}


static void env_select_released(void)
{
  reset_button_held = false;
}


static void lfo_shape_pressed(void)
{
  /* Unshifted: select next LFO shape
   * Shifted: select next filter mode */
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
  update_leds();
}


static void glide_pressed(void)
{
  /* Unshifted: select next glide mode
   * Shifted: toggle legato */
  if (!shift) {
    glide_preset = (glide_preset+1) % NUM_GLIDE_PRESETS;
    set_glide_preset(glide_preset);
    glide_led_blink_pattern = glide_led_blink_patterns[glide_preset];
  } else {
    chord_pgm_active = false;
    set_legato(!get_legato());
  }
  update_leds();
}


/* If the filter is off, change the modulation settings for pulse width
 * instead of cutoff frequency */
static void cutoff_mod_changed(void)
{
  uint8_t val = switches[SW_CUTOFFMOD0].state | (switches[SW_CUTOFFMOD1].state << 1);
  if (get_filter_mode() == FILTER_OFF) {
    set_pulse_width_mod_sources(val);
  } else {
    set_filter_cutoff_mod_sources(val);
  }
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
  update_leds();
}


static void chord_pgm_pressed(void)
{
  /* also doubles as a shift key when held */
  shift_button_held = true;

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
  update_leds();
}


static void chord_pgm_released(void)
{
  shift_button_held = false;
  shift = false;
  update_leds();
}


static void pitch_pgm_finish(void)
{
  /* compute the difference between the two notes and use it
   * as the pitch modulation amount */
  int8_t mod_amount = note_inputs[1] - note_inputs[0];
  set_pitch_mod_amount(mod_amount << 9);
  pitch_pgm_active = false;
  update_leds();
}


static void pitch_pgm_pressed(void)
{
  /* Unshifted: enter pitch range programming mode
   * Shifted: toggle keyboard tracking */
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
  update_leds();
}


void add_note_to_input(int8_t note)
{
  note_inputs[note_input_idx] = note;
  note_input_idx++;
  if (chord_pgm_active && note_input_idx == NUM_OSCILLATORS) {
    chord_pgm_finish();
  } else if (pitch_pgm_active && note_input_idx == 2) {
    pitch_pgm_finish();
  }
}


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
  
  /* PIO0_7, PIO0_10, and PIO1_5 are the LED anodes */
  /* PIO0_3, PIO0_4, and PIO0_5 are the LED cathodes */
  /* PIO1_8 is used to control the 4053 analog mux */
  GPIO_GPIO0DIR |= (1<<3) | (1<<4) | (1<<5) |
    (1<<LED_ANODE_0) |
    (1<<LED_ANODE_1) |
    (1<<LED_ANODE_2);

  GPIO_GPIO1DIR |= (1<<8) | (1<<9) |
    ((LED_ANODE_0>>4)<<(LED_ANODE_0&15)) |
    ((LED_ANODE_1>>4)<<(LED_ANODE_1&15)) |
    ((LED_ANODE_2>>4)<<(LED_ANODE_2&15));

  gpio1_set_pin_low(8);

  /* set up the knobs */
  knobs[KNOB_RELEASE].update_fn = update_release;
  knobs[KNOB_DETUNE].update_fn = update_detune;
  knobs[KNOB_WAVEFORM].update_fn = update_waveform;
  knobs[KNOB_ATTACK].update_fn = update_attack;
  knobs[KNOB_CUTOFF].update_fn = update_cutoff;
  knobs[KNOB_RESONANCE].update_fn = update_resonance;
  knobs[KNOB_CUTOFFMOD].update_fn = update_cutoff_mod_amount;
  knobs[KNOB_LFORATE].update_fn = update_lfo_rate;

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
  switches[SW_ENVSELECT].pressed_fn = env_select_pressed;
  switches[SW_ENVSELECT].released_fn = env_select_released;
  switches[SW_LFOSHAPE].pressed_fn = lfo_shape_pressed;
  switches[SW_GLIDE].pressed_fn = glide_pressed;
  switches[SW_CHORDPGM].pressed_fn = chord_pgm_pressed;
  switches[SW_CHORDPGM].released_fn = chord_pgm_released;
  switches[SW_PITCHPGM].pressed_fn = pitch_pgm_pressed;

  sound_init();
  pwm_init(254);
  systick_init(200);
  timer32_init(0,200000);

#if !DEBUG_LOGGING
  uart_init(BAUD(31250, 50000000));
#else
  uart_init(BAUD(115200, 50000000));
#endif
  
  /* Allow use of PIO0_0 as an input without resetting the CPU
   * We do this here because it seems like setting it at the
   * start of main prevents reset from working after serial programming...
   * oh well. */
  IOCON_nRESET_PIO0_0 = IOCON_nRESET_PIO0_0_FUNC_GPIO;

  //note_on(69);
  
  /* if the echo button is held long enough, we reset and enter
   * serial programming mode */
  uint16_t reset_hold_count = 0;

  /* if the chord program button is held enough, it becomes a shift key */
  uint16_t shift_hold_count = 0;

  /* for timing LED blinks */
  uint16_t blink_count = 0;

  while (1) {
    if (reset_button_held) {
      reset_hold_count++;
      if (reset_hold_count >= 10000) {
        cpu_reset();
      }
    } else {
      reset_hold_count = 0;
    }

    if (shift_button_held) {
      if (shift_hold_count < 2000) {
        shift_hold_count++;
      } else if (shift_hold_count == 2000) {
        chord_pgm_active = false;
        shift = true;
        update_leds();
      }
    } else {
      shift_hold_count = 0;
    }

    blink_count++;
    if (blink_count == 1500 && glide_led_blink_pattern != 0xFFFF) {
      blink_count = 0;
      glide_led_blink_pattern >>= 1;
      update_leds();
    }

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
        sw->debounce_count = 0;
      } else {
        /* only call the handler after the value has been steady for
         * a few iterations */
        if (sw->debounce_count >= 0) {
          sw->debounce_count++;
          if (sw->debounce_count == 8) {
            void (*handler)(void) = (state) ? sw->pressed_fn : sw->released_fn;
#if LOG_SWITCH_EDGES
            uart_send_byte((state) ? 'A'+i : 'a'+i);
            uart_send_byte('\n');
#endif
            if (handler) {
              handler();
            }
          }
        }
      }
    }
  }
  return 0;
}


void HardFault_Handler(void)
{
  /* turn off all the LEDs to prevent overcurrent */
  GPIO_GPIO0DIR = 0;
  GPIO_GPIO1DIR = 0;

#if DEBUG_LOGGING
  /* print address that caused the fault */
  uint32_t faultaddr;
  asm("ldr %[result], [sp, #0x20]" : [result] "=r" (faultaddr));
  outhex32(faultaddr);
  uart_send_byte('\n');
#endif

  while (1) {}
}
