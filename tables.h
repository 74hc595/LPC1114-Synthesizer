#include <stdint.h>

/* Map from knob position to envelope rate */
extern const uint16_t envtable[256];

/* Map from MIDI note number to frequency control value */
extern const uint32_t notetable[128];

/* Map from MIDI note number to cutoff frequency parameter
 * Entries are 16-bit fixed point numbers in the range [0, 2) with the
 * binary point between bit 15 (msb) and bit 14 */
#define NUM_CUTOFF_ENTRIES 167
extern const uint16_t cutofftable[NUM_CUTOFF_ENTRIES];

