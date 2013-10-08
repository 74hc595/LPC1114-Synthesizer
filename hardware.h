/**
 * Hardware-related subroutines
 */

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "lpc1xxx/cpu.h"
#include "lpc1xxx/gpio.h"
#include "lpc1xxx/systick.h"

/**
 * Starts the ADC (channels 0 and 1 only)
 */
void adc_init(void);


/**
 * Read a 10-bit value from the specified ADC channel.
 * Blocks until conversion has finished.
 */
uint32_t adc_read_channel(uint8_t channel);


/**
 * Set up SPI master operation (output only) in mode 0.
 */
void spi_init(void);

#endif
