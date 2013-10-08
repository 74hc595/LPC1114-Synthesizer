/**
 * Hardware-related subroutines
 */

#include "hardware.h"

/**
 * Starts the ADC (channels 0 and 1 only)
 */
void adc_init(void)
{
  /* Disable Power down bit to the ADC block. */
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_ADC);

  /* Enable AHB clock to the ADC. */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_ADC);

  /* Set AD0 to analog input */
  IOCON_JTAG_TDI_PIO0_11 = IOCON_JTAG_TDI_PIO0_11_FUNC_AD0;

  /* Set AD1 to analog input */
  IOCON_JTAG_TMS_PIO1_0 = IOCON_JTAG_TMS_PIO1_0_FUNC_AD1;

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
uint32_t adc_read_channel(uint8_t channel)
{
  /* Deselect all channels */
  ADC_AD0CR &= ~ADC_AD0CR_SEL_MASK;

  /* Start converting now on the appropriate channel */
  ADC_AD0CR |= ADC_AD0CR_START_STARTNOW | (1 << channel);

  uint32_t regVal = 0;
  do {
    regVal = *(pREG32(ADC_AD0DR0+(channel*4)));
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


/**
 * Set up SPI master operation (output only) in mode 0.
 */
void spi_init(void)
{
  /* Reset SSP */
  SCB_PRESETCTRL &= ~SCB_PRESETCTRL_SSP0_MASK;
  SCB_PRESETCTRL |= SCB_PRESETCTRL_SSP0_RESETDISABLED;

  /* Enable AHB clock to the SSP domain. */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_SSP0);

  /* Set P0.9 to SSP MOSI */
  IOCON_PIO0_9 &= ~IOCON_PIO0_9_FUNC_MASK;
  IOCON_PIO0_9 |= IOCON_PIO0_9_FUNC_MOSI0;

  /* Set P0.6 to SSP SCK */
  IOCON_SCKLOC = IOCON_SCKLOC_SCKPIN_PIO0_6;
  IOCON_PIO0_6 = IOCON_PIO0_6_FUNC_SCK;

  /* Set P0.2 to SSP SSEL */
  IOCON_PIO0_2 = IOCON_PIO0_2_FUNC_SSEL|IOCON_PIO0_3_MODE_INACTIVE;

  /* 16-bit transfers, mode 0, 1 bit per prescaler clock cycle */
  SSP_SSP0CR0 = SSP_SSP0CR0_DSS_16BIT |
                SSP_SSP0CR0_FRF_SPI |
                SSP_SSP0CR0_CPOL_LOW |
                SSP_SSP0CR0_CPHA_FIRST;

  /* 1/4 clock prescaler */
  SSP_SSP0CPSR = SSP_SSP0CPSR_CPSDVSR_DIV4;

  /* SPI master enabled */
  SSP_SSP0CR1 = SSP_SSP0CR1_LBM_NORMAL |
                SSP_SSP0CR1_SSE_ENABLED |
                SSP_SSP0CR1_MS_MASTER;
}
