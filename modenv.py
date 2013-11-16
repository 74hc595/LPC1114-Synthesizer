#!/usr/bin/env python

CPU_FREQ = 50000000.0
ENV_UPDATE_DIVISOR = 200000.0
ENV_UPDATE_RATE = CPU_FREQ/ENV_UPDATE_DIVISOR
ACCUM_BITS = 16

def freq_to_fcw(freq):
  return int(((1<<ACCUM_BITS)*freq)/ENV_UPDATE_RATE)

def fcw_to_freq(fcw):
  return (ENV_UPDATE_RATE*float(fcw))/(1<<ACCUM_BITS)

print """#include <stdint.h>

const uint16_t modenvtable[256] = {"""

# linear scale from 0 to 1 second
for i in range(0,256):
  seconds = i/255.0;
  fcw = freq_to_fcw(1.0/seconds) if seconds > 0 else (1<<ACCUM_BITS)-1
  fcw = min(fcw, (1<<ACCUM_BITS)-1)
  print "  %d, /* %g sec */" % (fcw, seconds)
print "};"
