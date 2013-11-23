from tablegen import *

element_type = 'uint16_t'
comment = 'LFO frequency values corresponding to knob positions'

LFO_ACCUM_BITS = 16
A = 0.195871
B = 5.31918

def generate_values():
  values = [0]
  for i in range(1,256):
    freq = A * math.exp(B * (i/255.0))
    fcw = freq_to_fcw(freq, MOD_UPDATE_RATE, LFO_ACCUM_BITS)
    comment = "%g Hz" % freq
    values.append((fcw, comment))
  return values

