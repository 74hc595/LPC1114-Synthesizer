#include <stdint.h>

/* Map from knob position to envelope rate */
extern const uint16_t envtable[256];

/* Map from MIDI note number to frequency control value */
extern const uint32_t notetable[128];

/* Map from MIDI note number to cutoff frequency parameter
 * Entries are 16-bit binary fractions in the range [0, 1) */
extern const uint16_t *cutofftable;
extern const int num_cutoff_entries;

/* Map from knob position to LFO rate */
extern const uint16_t lfofreqtable[256];

/* Map from knob position to modulation rate */
extern const uint16_t modenvtable[256];
