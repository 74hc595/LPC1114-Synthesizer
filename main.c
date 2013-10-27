#include "hardware.h"
#include "sound.h"

/* if true, use the UART for debug output instead of MIDI */
#define DEBUG_LOGGING 0

#define NUM_KNOBS 4
typedef struct
{
  uint8_t value;
  void (*update_fn)(uint8_t);
} knob_t;
static knob_t knobs[NUM_KNOBS] = {{0}};


#if DEBUG_LOGGING
/* for testing */
static char hexchars[16] = "0123456789ABCDEF";
static void outhex8(uint8_t x)
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
    sound_set_duty_cycle((127-knobval)<<1, 0xF);
  }
  /* Second half: pulse wave on 2 oscillators and sawtooth on the other two */
  else if (knobval < 0xFB) {
    sound_set_duty_cycle(knobval-128, 0x3);
    sound_set_sawtooth(0xC);
  }
  /* All the way to the right: sawtooth on all oscilators */
  else {
    sound_set_sawtooth(0xF);
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

  sound_set_detune(knobval >> 6, knobval & 0x3f);
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
  /* FCLKIN = 25MHz
   * M = 2, so clock frequency is M*FCLKIN = 50MHz
   * P = 2
   * FCCO = 2*P*FCLKOUT = 200MHz */
  cpu_pll_setup(SCB_PLLCTRL_MSEL_2, SCB_PLLCTRL_PSEL_2);

  /* Allow use of PIO0_0 without resetting the CPU */
  //IOCON_nRESET_PIO0_0 = IOCON_nRESET_PIO0_0_FUNC_GPIO;

  //cpu_enable_clkout();
  adc_init();
  spi_init();
  
  /* PIO1_8 is used to control the 4053 analog mux */
  GPIO_GPIO1DIR |= (1<<8)|(1<<9);
  gpio1_set_pin_low(8);

  /* set up the knobs */
  knobs[0].update_fn = update_waveform;
  knobs[1].update_fn = update_detune;
  knobs[2].update_fn = update_attack;
  knobs[3].update_fn = update_release;

  sound_init();
  pwm_init(254);
  systick_init(200);
  timer32_init(200000);

#if !DEBUG_LOGGING
  uart_init(BAUD(31250, 50000000));
#else
  uart_init(BAUD(115200, 50000000));
#endif
  
  while (1) {

    /* read the knobs */
    uint8_t k;
    for (k = 0; k < NUM_KNOBS; k++) {
      /* discard the lowest 2 bits from the ADC input to reduce noise
       * use an exponential moving average with alpha=0.25 to prevent
       * "dither" between values */
      knob_t *knob = knobs+k;

      /* set the mux control line high when reading the last 3 channels */
      uint8_t ch = k;
      if (k >= 6) {
        k -= 3;
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
  while (1) {}
}
