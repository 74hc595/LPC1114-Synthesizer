/**
 * Hardware-related subroutines
 */

#include "hardware.h"

/**
 * Starts the ADC for all channels.
 */
void adc_init(void)
{
  /* Disable Power down bit to the ADC block. */
  SCB_PDRUNCFG &= ~(SCB_PDRUNCFG_ADC);

  /* Enable AHB clock to the ADC. */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_ADC);

  /* Set AD0 to analog input */
  IOCON_JTAG_TDI_PIO0_11 = IOCON_JTAG_TDI_PIO0_11_FUNC_AD0|IOCON_JTAG_TDI_PIO0_11_MODE_PULLUP;

  /* Set AD1 to analog input */
  IOCON_JTAG_TMS_PIO1_0 = IOCON_JTAG_TMS_PIO1_0_FUNC_AD1;
  
  /* Set AD2 to analog input */
  IOCON_JTAG_TDO_PIO1_1 = IOCON_JTAG_TDO_PIO1_1_FUNC_AD2;

  /* Set AD3 to analog input */
  IOCON_JTAG_nTRST_PIO1_2 = IOCON_JTAG_nTRST_PIO1_2_FUNC_AD3;

  /* Set AD4 to analog input */
  IOCON_SWDIO_PIO1_3 = IOCON_SWDIO_PIO1_3_FUNC_AD4;

  /* Set AD5 to analog input */
  IOCON_PIO1_4 = IOCON_PIO1_4_FUNC_AD5;

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

  /* Set P0.8 to SSP MISO */
//  IOCON_PIO0_8 &= ~IOCON_PIO0_8_FUNC_MASK;
  IOCON_PIO0_8 = IOCON_PIO0_8_FUNC_MISO0;

  /* Set P0.9 to SSP MOSI */
//  IOCON_PIO0_9 &= ~IOCON_PIO0_9_FUNC_MASK;
  IOCON_PIO0_9 = IOCON_PIO0_9_FUNC_MOSI0;

  /* Set P0.10 to SSP SCK */
//  IOCON_SCKLOC = IOCON_SCKLOC_SCKPIN_PIO0_10;
//  IOCON_JTAG_TCK_PIO0_10 = IOCON_JTAG_TCK_PIO0_10_FUNC_SCK;
  IOCON_SCKLOC = IOCON_SCKLOC_SCKPIN_PIO0_6;
  IOCON_PIO0_6 = IOCON_PIO0_6_FUNC_SCK;

  IOCON_JTAG_TCK_PIO0_10 = IOCON_JTAG_TCK_PIO0_10_FUNC_GPIO;
  GPIO_GPIO0DIR |= (1 << 10);
  gpio0_set_pin_low(10);

  /* Set P0.2 to SSP SSEL */
  IOCON_PIO0_2 = IOCON_PIO0_2_FUNC_SSEL;

  /* 16-bit transfers, mode 0, 1 bit per prescaler clock cycle */
  SSP_SSP0CR0 = SSP_SSP0CR0_DSS_16BIT |
                SSP_SSP0CR0_FRF_SPI |
                SSP_SSP0CR0_CPOL_HIGH |
                SSP_SSP0CR0_CPHA_SECOND;

  /* 1/4 clock prescaler */
  SSP_SSP0CPSR = SSP_SSP0CPSR_CPSDVSR_DIV4;

  /* SPI master enabled */
  SSP_SSP0CR1 = SSP_SSP0CR1_LBM_NORMAL |
                SSP_SSP0CR1_SSE_ENABLED |
                SSP_SSP0CR1_MS_MASTER;
}


void uart_init(uint16_t divisor)
{
  /* Set 1.6 UART RXD */
  IOCON_PIO1_6 &= ~IOCON_PIO1_6_FUNC_MASK;
  IOCON_PIO1_6 |= IOCON_PIO1_6_FUNC_UART_RXD;

  /* Set 1.7 UART TXD */
  IOCON_PIO1_7 &= ~IOCON_PIO1_7_FUNC_MASK;	
  IOCON_PIO1_7 |= IOCON_PIO1_7_FUNC_UART_TXD;

  /* Enable UART clock */
  SCB_SYSAHBCLKCTRL |= (SCB_SYSAHBCLKCTRL_UART);
  SCB_UARTCLKDIV = SCB_UARTCLKDIV_DIV1;     /* divided by 1 */

  /* 8 bits, no parity, 1 stop bit, prepare to set baud rate */
  UART_U0LCR = UART_U0LCR_Word_Length_Select_8Chars |
               UART_U0LCR_Stop_Bit_Select_1Bits |
               UART_U0LCR_Parity_Disabled |
               UART_U0LCR_Divisor_Latch_Access_Enabled;

  /* Set baud rate */
  UART_U0DLM = divisor >> 8;
  UART_U0DLL = divisor & 0xFF;
  UART_U0LCR &= (~UART_U0LCR_Divisor_Latch_Access_MASK);

  /* Enable RX and TX FIFO */
  UART_U0FCR = UART_U0FCR_FIFO_Enabled |
               UART_U0FCR_Tx_FIFO_Reset |
               UART_U0FCR_Rx_FIFO_Reset;

  /* Enable the UART interrupt */
  NVIC_EnableIRQ(UART_IRQn);
  UART_U0IER = UART_U0IER_RBR_Interrupt_Enabled;
}


void uart_send_byte(uint8_t byte)
{
  while (!(UART_U0LSR & UART_U0LSR_THRE)) {}
  UART_U0THR = byte;
}


void timer32_init(uint32_t rate)
{
  /* Enable the clock for CT32B0 */
  SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_CT32B0;

  /* Set rate */
  TMR_TMR32B0MR0 = rate;

  /* Configure match control register to raise an interrupt and reset on MR0 */
  TMR_TMR32B0MCR = (TMR_TMR32B0MCR_MR0_INT_ENABLED | TMR_TMR32B0MCR_MR0_RESET_ENABLED);

  /* Enable the TIMER0 interrupt */
  NVIC_EnableIRQ(TIMER_32_0_IRQn);

  /* Start the timer */
  TMR_TMR32B0TCR = TMR_TMR32B0TCR_COUNTERENABLE_ENABLED;
}


void pwm_init(uint16_t pulsewidth)
{
  /* Enable clock for 16-bit timer 1 */
  SCB_SYSAHBCLKCTRL |= SCB_SYSAHBCLKCTRL_CT16B1;

  /* Configure PIO1.9 (pin 18) as 16-bit timer 1 MAT0 output */
  IOCON_PIO1_9 = IOCON_PIO1_9_FUNC_CT16B1_MAT0;

  /* Frequency; PWM period resets at this value */
  TMR_TMR16B1MR3 = pulsewidth;

  /* Initial duty cycle */
  TMR_TMR16B1MR0 = pulsewidth/2;

  /* Reset timer when value reaches MR3 */
  TMR_TMR16B1MCR = TMR_TMR16B1MCR_MR3_RESET_ENABLED;

  /* Toggle pin when value reaches MR0 */
  TMR_TMR16B1EMR = TMR_TMR16B1EMR_EM0|TMR_TMR16B1EMR_EMC0_TOGGLE;

  /* Enable PWM0 and PWM3 */
  TMR_TMR16B1PWMC = TMR_TMR16B1PWMC_PWM0_ENABLED|TMR_TMR16B1PWMC_PWM3_ENABLED;

  /* Start the timer */
  TMR_TMR16B1TCR = TMR_TMR16B1TCR_COUNTERENABLE_ENABLED;
}

