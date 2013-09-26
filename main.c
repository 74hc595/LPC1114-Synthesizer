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

/**
 * Starts the ADC (channel 0 only)
 */
void adcInit(void)
{
  /* Disable Power down bit to the ADC block. */
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_ADC);

  /* Enable AHB clock to the ADC. */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_ADC);

  /* Set AD0 to analog input */
  IOCON_JTAG_TDI_PIO0_11 &= ~(IOCON_JTAG_TDI_PIO0_11_ADMODE_MASK |
                              IOCON_JTAG_TDI_PIO0_11_FUNC_MASK |
                              IOCON_JTAG_TDI_PIO0_11_MODE_MASK);
  IOCON_JTAG_TDI_PIO0_11 |=  (IOCON_JTAG_TDI_PIO0_11_FUNC_AD0 &
                              IOCON_JTAG_TDI_PIO0_11_ADMODE_ANALOG);

  /* Set channel and clock divider but don't start */
  ADC_AD0CR = (ADC_AD0CR_SEL_AD0 |      /* SEL=1,select channel 0 on ADC0 */
              (11 << 8) |               /* CLKDIV = 48MHz/11 = 4.36MHz */ 
              ADC_AD0CR_BURST_SWMODE |  /* BURST = 0, no BURST, software controlled */
              ADC_AD0CR_CLKS_10BITS |   /* CLKS = 0, 11 clocks/10 bits */
              ADC_AD0CR_START_NOSTART | /* START = 0 A/D conversion stops */
              ADC_AD0CR_EDGE_RISING);   /* EDGE = 0 (CAP/MAT signal falling, trigger A/D conversion) */ 
}


/**
 * Read a 10-bit value from the specified ADC channel.
 * Blocks until conversion has finished.
 */
uint32_t adcReadChannel(uint8_t channel)
{
  /* Deselect all channels */
  ADC_AD0CR &= ~ADC_AD0CR_SEL_MASK;

  /* Start converting now on the appropriate channel */
  ADC_AD0CR |= ADC_AD0CR_START_STARTNOW | (1 << channel);

  uint32_t regVal = 0;
  do {
    regVal = *(pREG32(ADC_AD0DR0));
  } while ((regVal & ADC_DR_DONE) == 0);

  /* stop ADC */
  ADC_AD0CR &= ~ADC_AD0CR_START_MASK;

  /* return 0 if an overrun occurred */
  if (regVal & ADC_DR_OVERRUN) {
    return 0;
  }

  /* return conversion results */
  return (regVal >> 6) & 0x3FF;
}


int main(void)
{
  cpuPllSetup(CPU_MULTIPLIER_4);
  cpuEnableClkout();
  adcInit();

  waveform_ptr = waveform;
  waveform_end = waveform+256;

  GPIO_GPIO1DIR |= (1 << 5)|(1 << 8);
  systickInit(1704);

  while (1) {
    uint32_t val = adcReadChannel(0);
    val <<= 2;
    SYSTICK_STRELOAD = val + 60;
  }

  return 0;
}

