/**
 * LPC1114 GPIO convenience macros
 * Matt Sarnoff (msarnoff.org)
 * October 31, 2013
 */

#ifndef _GPIO_H_
#define _GPIO_H_

#include "lpc111x.h"

/**
 * Changes the value of the pins specified by pinmask.
 */
#define gpio_pins(gpio, pinmask) *((volatile uint32_t *)(GPIO_ ## gpio ## _BASE + ((pinmask) << 2)))
#define gpio0_set_pins_low(pinmask) gpio_set_pins_high(GPIO0, pinmask) = 0;
#define gpio0_set_pins_high(pinmask) gpio_set_pins_high(GPIO0, pinmask) = 0xFFF;
#define gpio1_set_pins_low(pinmask) gpio_set_pins_high(GPIO1, pinmask) = 0;
#define gpio1_set_pins_high(pinmask) gpio_set_pins_high(GPIO1, pinmask) = 0xFFF;

/**
 * Changes the value of the numbered pin.
 * */
#define gpio_pin(gpio, pin) *((volatile uint32_t *)(GPIO_ ## gpio ## _BASE + (1 << ((pin)+2))))
#define gpio0_set_pin_low(pin) gpio_pin(GPIO0, pin) = 0;
#define gpio0_set_pin_high(pin) gpio_pin(GPIO0, pin) = 0xFFF;
#define gpio1_set_pin_low(pin) gpio_pin(GPIO1, pin) = 0;
#define gpio1_set_pin_high(pin) gpio_pin(GPIO1, pin) = 0xFFF;

#endif
