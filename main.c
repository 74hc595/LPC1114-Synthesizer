#include "hardware.h"
#include "sound.h"

#define NUM_KNOBS 2
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


int main(void)
{
  cpu_pll_setup(CPU_MULTIPLIER_4);
  cpu_enable_clkout();
  adc_init();
  spi_init();
  GPIO_GPIO1DIR |= (1<<8)|(1<<9);
  gpio1_set_pin_low(9);

  /* set up the knobs */
  knobs[0].update_fn = update_waveform;
  knobs[0].bits = 9;
  knobs[1].update_fn = update_detune;
  knobs[1].bits = 8;

  sound_init();

  systick_init(200);

  uart_rx_init(BAUD(31250, 48000000));
  
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
      note_on(midibuf[1]);
      break;
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
        midibytesleft = 2;
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
