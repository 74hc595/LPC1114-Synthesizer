#!/usr/bin/env python

from math import log, sin, pi, floor

CPU_FREQ = 50000000.0
UPDATE_DIVISOR = 200.0
SAMPLE_RATE = CPU_FREQ/UPDATE_DIVISOR
NYQUIST = SAMPLE_RATE/2.0
CUTOFF_BITS = 16

def note_to_freq(note):
  return 440.0 * 2**((note-69.0)/12.0)

def freq_to_note(freq):
  # inverse of the previous equation, according to Wolfram Alpha
  return int(floor(12.0*log((4.0/55.0)*2**(0.75)*freq)/log(2)))

max_note = freq_to_note(NYQUIST)

print """#include "tables.h"
/* %d entries */
const uint16_t cutofftable[NUM_CUTOFF_ENTRIES] = {""" % (max_note+1)
for i in range(0,max_note+1):
  freq = note_to_freq(i)
  cutoff = 2.0*sin(pi*freq/SAMPLE_RATE) # with correction factor, according to Chamberlin p. 492
  binary = int(round(cutoff*(1<<(CUTOFF_BITS-1))))
  print "  %d," % binary
print "};"
