#include "lpc1xxx/cpu.h"
#include "lpc1xxx/gpio.h"
#include "lpc1xxx/systick.h"

#if 0
extern volatile uint8_t *waveform;
extern volatile uint8_t *waveform_end;
extern volatile uint8_t *waveform_ptr;
#endif

extern volatile uint8_t waveform[256];
extern volatile uint8_t *waveform_ptr;
volatile uint8_t *waveform_end;

int main(void)
{
  cpuPllSetup(CPU_MULTIPLIER_4);
  cpuEnableClkout();

  waveform_ptr = waveform;
  waveform_end = waveform+256;

  GPIO_GPIO1DIR |= (1 << 5)|(1 << 8);
  systickInit(1704);

  while (1) {
//    gpioPin(GPIO1, 8) = (SYSTICK_STCURR & 256) ? 0xFFF : 0;
  }

  return 0;
}

