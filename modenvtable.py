# LPC1114 Synthesizer
# Matt Sarnoff (msarnoff.org)
# November 24, 2013
#
# Generates map from knob position to modulation envelope rate.

from tablegen import *

element_type = 'uint16_t'
comment = 'Modulation envelope rates corresponding to knob positions'

ENV_ACCUM_BITS = 16

def generate_values():
  values = []
  # linear scale from 0 to 1 second
  for i in range(0,256):
    seconds = i/255.0;
    fcw = freq_to_fcw(1.0/seconds, MOD_UPDATE_RATE, ENV_ACCUM_BITS) if seconds > 0 else (1<<ENV_ACCUM_BITS)-1
    fcw = min(fcw, (1<<ENV_ACCUM_BITS)-1)
    comment = "%g sec" % seconds
    values.append((fcw,comment))
  return values
