#include "hardware.h"
#include "sound.h"

#define NUM_KNOBS 4
typedef struct
{
//  uint32_t accum;
//  uint8_t samples;
  uint8_t bits;
  uint16_t last_value;
  void (*update_fn)(uint16_t);
} knob_t;
static knob_t knobs[NUM_KNOBS] = {{0}};


static void update_waveform(uint16_t knobval)
{
  if (knobval <= 255) {
    sound_set_duty_cycle(255-knobval, 0xF);
  } else if (knobval <= 500) {
    sound_set_duty_cycle(knobval-256, 0x3);
    sound_set_sawtooth(0xC);
  } else {
    sound_set_sawtooth(0xF);
  }
}


static void update_detune(uint16_t knobval)
{
  sound_set_detune(knobval >> 6, knobval & 0x3f);
}


static void update_attack(uint16_t knobval)
{
  set_attack((256 - knobval) << 7);
}


static void update_release(uint16_t knobval)
{
  set_release((256 - knobval) << 7);
}


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


int main(void)
{
  cpu_pll_setup(CPU_MULTIPLIER_4);
  //cpu_enable_clkout();
  adc_init();
  spi_init();
  GPIO_GPIO1DIR |= (1<<8)|(1<<9);
  gpio1_set_pin_low(9);

  /* set up the knobs */
  knobs[0].update_fn = update_waveform;
  knobs[0].bits = 9;
  knobs[1].update_fn = update_detune;
  knobs[1].bits = 8;
  knobs[2].update_fn = update_attack;
  knobs[2].bits = 8;
  knobs[3].update_fn = update_release;
  knobs[3].bits = 8;


  sound_init();
  systick_init(200);
  timer32_init(200000);

  uart_rx_init(BAUD(31250, 48000000));
  
  uint8_t buttoncount = 0;

  while (1) {
    /* read the knobs */
/*    int ch;
    for (ch = 0; ch < NUM_KNOBS; ch++) {
      knob_t *knob = knobs+ch;
      knob->accum += adc_read_channel(ch);
      knob->samples++;
      if (knob->samples == 4) {
        uint16_t newval = knob->accum >> 2;
        knob->accum = 0;
        knob->samples = 0;
        if (newval != knob->last_value) {
          knob->update_fn(newval);
          knob->last_value = newval;
        }
      }
    }*/

    int ch;
    for (ch = 0; ch < NUM_KNOBS; ch++) {
      uint32_t newval = adc_read_channel(ch);
      newval >>= 10 - knobs[ch].bits;
      if (newval != knobs[ch].last_value) {
        knobs[ch].update_fn(newval);
        knobs[ch].last_value = newval;
      }
    }

    /* read the button */
    uint16_t button = gpio_pin(GPIO0, 1);
    if (!button) {
      buttoncount++;
      if (buttoncount == 253) {
        button_pressed();
      }
      if (buttoncount >= 254) {
        buttoncount = 254;
      }
    } else {
      buttoncount = 0;
    }
  }

  return 0;
}


static volatile uint8_t midibuf[3];
static volatile uint8_t midibytesleft = 0;
static inline void handle_midi_command(void)
{
  switch (midibuf[0]) {
    case 0x80:  /* note off */
      note_off(midibuf[1]);
      break;
    case 0x90:  /* note on */
      if (!chprog_active) {
        note_on(midibuf[1]);
      } else {
        chprog_add_note(midibuf[1]);
      }
      break;
    case 0xC0:  /* program change */
      /* temporary; using this to control glide amount for now */
      set_glide(midibuf[2]);
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
  gpio1_set_pin_high(9);
  while (1) {}
}
