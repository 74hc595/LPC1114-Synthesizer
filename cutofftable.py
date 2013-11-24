# LPC1114 Synthesizer
# Matt Sarnoff (msarnoff.org)
# November 24, 2013
#
# Generates map from MIDI note number to cutoff frequency parameter.
# Output values are 16-bit binary fractions in the range [0, 1)

from tablegen import *

element_type = 'uint16_t'
comment = 'Cutoff frequency values corresponding to knob positions'

CUTOFF_BITS = 16

def preamble():
  return "const int num_cutoff_entries = %d;" % len(generate_values())

def generate_values():
  i = 0
  values = []
  while True:
    freq = note_to_freq(i)
    cutoff = 2.0*math.sin(math.pi*freq/SAMPLE_RATE)
    if cutoff > 1.0:
      break
    binary = int(round(cutoff*(1<<CUTOFF_BITS)))
    values.append(binary)
    i += 1
  return values


