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
 * Reads a 10-bit value from the specified ADC channel.
 * Blocks until conversion has finished.
 */
uint32_t adc_read_channel(uint8_t channel);


/**
 * Sets up SPI master operation (output only) in mode 0.
 */
void spi_init(void);


/**
 * Sets up the UART at the specified baud rate, 8 data bits,
 * no parity, 1 stop bit, and enables the receive interrupt.
 * "divisor" should be the output of the BAUD() macro.
 */
void uart_init(uint16_t divisor);


/**
 * Sends a byte over the UART.
 */
void uart_send_byte(uint8_t byte);


/**
 * Computes the divisor for a given baud rate and CPU frequency.
 */
#define BAUD(rate, cpufreq) (cpufreq/(16*(rate)))


/**
 * Sets up 32-bit timer 0 to trigger an interrupt at the given rate.
 */
void timer32_init(uint32_t rate);

#endif
