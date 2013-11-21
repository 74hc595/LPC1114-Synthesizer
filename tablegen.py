#!/usr/bin/env python

import math
import re
import sys
import time

### Constants
CPU_FREQ = 50000000.0
SYSTICK_DELAY = 200.0
MOD_UPDATE_DIVISOR = 200000.0
SAMPLE_RATE = CPU_FREQ/SYSTICK_DELAY
MOD_UPDATE_RATE = CPU_FREQ/MOD_UPDATE_DIVISOR
ENV_ACCUM_BITS = 16



### Utility functions
def note_to_freq(note):
  return 440.0 * 2**((note-69.0)/12.0)


def freq_to_note(freq):
  # inverse of the previous equation, according to Wolfram Alpha
  return int(math.floor(12.0*math.log((4.0/55.0)*2**(0.75)*freq)/math.log(2)))


def note_name(note):
  notename = ['C','C#','D','D#','E','F','F#','G','G#','A','A#','B'][note%12]
  octave = (note/12)-1
  return "%s%d" % (notename, octave)


def freq_to_fcw(freq,samplerate,bits):
  return int(((1<<bits)*freq)/samplerate)


def fcw_to_freq(fcw,samplerate,bits):
  return (samplerate*float(fcw))/(1<<bits)


def note_to_fcw(note,samplerate,bits):
  freq = note_to_freq(note)
  # first try rounding down
  fcw = freq_to_fcw(freq,samplerate,bits)
  actualfreq = fcw_to_freq(fcw,samplerate,bits)
  # will rounding up be more accurate?
  actualfreq_plus1 = fcw_to_freq(fcw+1,samplerate,bits)
  if abs(actualfreq_plus1-freq) < abs(actualfreq-freq):
    fcw += 1
  return fcw



### Main
def main(args):
  if len(args) < 1:
    print """usage: tablegen.py <generator> [outputfile]
If outputfile is not specified, output will be written to <generator>.c.
Specify - for outputfile to write to standard output."""
    sys.exit(1)

  # remove the .py extension if it's there
  generator_name = re.sub(r'\.py$', '', args[0])
  output_name = generator_name + '.c' if len(args) < 2 else args[1]

  generator = None
  try:
    sys.dont_write_bytecode = True    
    generator = __import__(generator_name, fromlist=[])
  except ImportError:
    sys.stderr.write("generator '%s' not found\n" % generator_name)
    sys.exit(2)

  values = generator.generate_values()
  comment = generator.comment if hasattr(generator, 'comment') else None
  tablename = generator.table_name if hasattr(generator, 'table_name') else generator_name

  # generate the code
  f = open(output_name, 'w') if output_name != '-' else sys.stdout
  f.write("/* Generated from '%s' by tablegen.py at %s */\n" % (
    generator_name,
    time.strftime("%Y-%m-%d %H:%M:%S")))

  if comment is not None:
    f.write("/* %s */\n" % comment)

  f.write("#include <stdint.h>\n")

  if hasattr(generator, 'preamble'):
    f.write(generator.preamble()+"\n")

  f.write("const %s %s[%d] = {\n" % (
    generator.element_type,
    tablename,
    len(values)))
  for value in values:
    # if value is a scalar, output it
    if not hasattr(value, '__iter__'):
      f.write("  %d,\n" % value)
    # otherwise, treat it as a value and comment pair
    else:
      f.write("  %d, /* %s */\n" % value)

  f.write("};\n")
  f.close()


if __name__ == "__main__":
  main(sys.argv[1:])

