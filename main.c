#include "lpc1xxx/cpu.h"
#include "lpc1xxx/gpio.h"

int main(void)
{
  volatile uint32_t count, count_max = 500000;

  cpuPllSetup(CPU_MULTIPLIER_4);

  GPIO_GPIO1DIR |= (1 << 8);
  while (1) {
    for (count = 0; count < count_max; count++);
    gpioSetPinHigh(GPIO1,8)
    for (count = 0; count < count_max; count++);
    gpioSetPinLow(GPIO1,8);
  }

  return 0;
}
