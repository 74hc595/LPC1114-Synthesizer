/**
 * Sound parameter control functions.
 */


#ifndef _SOUND_H_
#define _SOUND_H_

#include <stdint.h>

void sound_init(void);

void sound_set_duty_cycle(uint8_t val, uint8_t oscmask);
void sound_set_sawtooth(uint8_t oscmask);
void sound_set_lofi_sawtooth(uint8_t oscmask);

void sound_set_detune(uint8_t num_oscillators, uint8_t spread);

#endif
