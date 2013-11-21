from tablegen import *

FREQ_ACCUM_BITS = 32

element_type = 'uint32_t'
comment = 'Frequency values corresponding to MIDI note numbers'

def generate_values():
  values = []
  for note in range(0,128):
    fcw = note_to_fcw(note, SAMPLE_RATE, FREQ_ACCUM_BITS)
    idealfreq = note_to_freq(note)
    actualfreq = fcw_to_freq(fcw, SAMPLE_RATE, FREQ_ACCUM_BITS)
    error = actualfreq-idealfreq
    percenterror = (abs(actualfreq-idealfreq)/idealfreq)*100.0
    comment = "%d %.2f Hz (error: %+.5f Hz), %s" % (
        note,
        actualfreq,
        error,
        note_name(note))
    values.append((fcw,comment))
  return values

