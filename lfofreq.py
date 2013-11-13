#!/usr/bin/env python

from math import exp
A = 0.48929
B = 5.54311

CPU_FREQ = 50000000.0
LFO_UPDATE_DIVISOR = 200000.0
LFO_UPDATE_RATE = CPU_FREQ/LFO_UPDATE_DIVISOR
ACCUM_BITS = 16

def note_to_freq(note):
  return 440.0 * 2**((note-69.0)/12.0)

def note_name(note):
  notename = ['C','C#','D','D#','E','F','F#','G','G#','A','A#','B'][note%12]
  octave = (note/12)-1
  return "%s%d" % (notename, octave)

def freq_to_fcw(freq):
  return int(((1<<ACCUM_BITS)*freq)/LFO_UPDATE_RATE)

def fcw_to_freq(fcw):
  return (LFO_UPDATE_RATE*float(fcw))/(1<<ACCUM_BITS)

def note_to_fcw(note):
  freq = note_to_freq(note)
  # first try rounding down
  fcw = freq_to_fcw(freq)
  actualfreq = fcw_to_freq(fcw)
  # will rounding up be more accurate?
  actualfreq_plus1 = fcw_to_freq(fcw+1)
  if abs(actualfreq_plus1-freq) < abs(actualfreq-freq):
    fcw += 1
  return fcw

for i in range(0,256):
  freq = A*exp(B*(i/255.0))
  fcw = freq_to_fcw(freq)
  print "  %d, /* %g Hz */" % (fcw, freq)

"""

250-125
125-62.5
62.5-31.25
31.25-15.625
15.625-7.8125
7.8125-3.90625
3.90625-1.953125


"""

"""
for note in range(0,128):
  if not DELTAS:
    fcw = note_to_fcw(note)
    idealfreq = note_to_freq(note)
    actualfreq = fcw_to_freq(fcw)
    error = actualfreq-idealfreq
    percenterror = (abs(actualfreq-idealfreq)/idealfreq)*100.0
    #print ".int %d\t;%d\t%.2f Hz (error: %+.3f Hz), %s" % (
    print "  %d,\t/* %d\t%.2f Hz (error: %+.5f Hz), %s */" % (
        fcw,
        note,
        actualfreq,
        error,
        note_name(note))
    #nextfcw = note_to_fcw(note+1)
    #delta = nextfcw-fcw
    #step = delta/256.0
    #shiftedstep = int(step*64.0)
    #print "%d %d %d" % (fcw, fcw+((256*shiftedstep)>>6), nextfcw)
  else:
    fcw = note_to_fcw(note)
    nextfcw = note_to_fcw(note+1)
    delta = nextfcw-fcw
    step = delta/512.0
#    shiftedstep = int(step*64.0)  # output 11.5 fixed point
    print "  %d,\t/* %d, %s */" % (step, note, note_name(note))
"""
