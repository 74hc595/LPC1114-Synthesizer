#include "lpc1xxx/cpu.h"
#include "lpc1xxx/gpio.h"
#include "lpc1xxx/systick.h"

static const uint8_t waveform[64] = {
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
};
static volatile uint8_t waveidx = 0;


void SysTick_Handler(void)
{
  gpioPin(GPIO1,8) = (waveform[waveidx++] << 8);
  waveidx &= 63;
}


int main(void)
{
  volatile uint32_t count, count_max = 500000;

  cpuPllSetup(CPU_MULTIPLIER_4);

  GPIO_GPIO1DIR |= (1 << 8);
  systickInit(1704);

  while (1) {
  }

  return 0;
}
