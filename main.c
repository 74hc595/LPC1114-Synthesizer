#include "lpc1xxx/cpu.h"
#include "lpc1xxx/gpio.h"
#include "lpc1xxx/systick.h"

/**
 * Oscillator state.
 */
typedef struct
{
  uint32_t *phaseptr;
  uint32_t freq;
  uint32_t phase;
} oscillator_state_t;


/**
 * This structure mirrors the oscillator-updating code in the SysTick handler
 * and allows it to be modified in a structured manner.
 * The __pad members should not be touched!
 */
typedef struct
{
  uint16_t __pad1[3];
  uint8_t volume;
  uint8_t __pad2;
  union {
    uint16_t waveform_code[4];
    struct {
      uint16_t __pad3;
      uint8_t duty;
      uint8_t __pad4;
      uint16_t __pad5[2];
    };
  };
  uint16_t __pad6;
} oscillator_control_t;


extern volatile oscillator_state_t oscillators[4];
extern volatile oscillator_control_t osc_update_base[4];
extern volatile uint32_t filter_cutoff; /* 12-bit unsigned */
extern volatile uint32_t filter_q; /* 12-bit unsigned */

/**
 * Starts the ADC (channels 0 and 1only)
 */
void adcInit(void)
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
uint32_t adcReadChannel(uint8_t channel)
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
void spiInit(void)
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


void oscillator_set_sawtooth(int oscnum)
{
  osc_update_base[oscnum].waveform_code[0] = 0x13d2; /* asr r2, #15 */
  osc_update_base[oscnum].waveform_code[1] = 0x434a; /* mul r2, r1 */ 
  osc_update_base[oscnum].waveform_code[2] = 0x46c0; /* nop */
  osc_update_base[oscnum].waveform_code[3] = 0x46c0; /* nop */
}


void oscillator_set_pulse(int oscnum)
{
  osc_update_base[oscnum].waveform_code[0] = 0x15d2; /* asr r2, #23 */
  osc_update_base[oscnum].waveform_code[1] = 0x3a00; /* sub r2, #<duty> */ 
  osc_update_base[oscnum].waveform_code[2] = 0x0409; /* lsl r1, #16 */
  osc_update_base[oscnum].waveform_code[3] = 0x404a; /* eor r2, r1 */
}


int main(void)
{
  cpuPllSetup(CPU_MULTIPLIER_4);
  cpuEnableClkout();
  adcInit();
  spiInit();
  GPIO_GPIO1DIR |= (1<<8)|(1<<9);
  gpioSetPinLow(GPIO1, 9);

  osc_update_base[0].volume = 255;
//  osc_update_base[1].volume = 255;
//  oscillator_set_sawtooth(0);
//  oscillator_set_sawtooth(1);

  oscillators[0].freq = 5000000;
//  oscillators[1].freq = 5005000;
  //oscillators[2].freq = 5005000;
  //oscillators[3].freq = 5005000;

  systickInit(200);
  

  while (1) {
    volatile int i = 0;
//    for (i = 0; i < 5000; i++) {}
    uint32_t val = adcReadChannel(0);
    filter_cutoff = val >> 2;
    val = adcReadChannel(1);
    filter_q = val << 2;
  }

  return 0;
}


void HardFault_Handler(void)
{
  gpioSetPinHigh(GPIO1, 9);
  while (1) {}
}
