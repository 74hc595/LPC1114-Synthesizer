# LPC1114 Synthesizer
# Matt Sarnoff (msarnoff.org)
# November 24, 2013
#
# Generates map from knob position to envelope rate.
# Amplitude envelope is exponential; the output values are coefficients
# that control the rise/fall time.

from tablegen import *

ALPHA_BITS = 15 # not to be confused with the cereal
SECS_PER_STEP = MOD_UPDATE_DIVISOR/CPU_FREQ

element_type = 'uint16_t'
comment = 'Envelope coefficients corresponding to knob positions'

def rise_time_for_alpha(alpha):
  envelope_value = 0
  envelope_dest = 0xFFFF
  steps = 0
  while True:
    delta = envelope_dest - envelope_value
    delta = (delta*alpha) >> ALPHA_BITS
    envelope_value += delta
    if delta == 0 or envelope_value >= 0xFF00:
      break
    steps += 1
  return steps


def generate_values():
  # first, compute the time in seconds we want for each step
  # scale is piecewise
  env_times = [0.0]

  # first 'half' is linear from 0 to 1 second
  # (the ADC channel used for the attack knob is a little nonlinear)
  for i in range(0x01,0xA0):
    env_times.append(1.60377*(i/255.0))

  # second half is exponential from 1 second to 10 seconds
  for i in range(0xA0,256):
    env_times.append(0.0220673*math.exp(6.11624*(i/255.0)))

  if len(env_times) != 256:
    print >> sys.stderr, "Expected exactly 256 time values (%d specified)" % len(env_times)
    sys.exit(1)

  # second, for every possible "alpha" value, compute the rise time
  # yeah it's brute force so what
  last_time_taken = 0
  rise_time_pairs = []
  for alpha in range(0,(1<<ALPHA_BITS)):
    time_taken = rise_time_for_alpha(alpha)
    if time_taken != last_time_taken:
      time_in_seconds = time_taken*SECS_PER_STEP
      rise_time_pairs.append((alpha,time_in_seconds))
      last_time_taken = time_taken

  # last, find the best matching alpha value for each time
  values = []
  for t in env_times:
    best_match = (0, 0)
    for rp in rise_time_pairs:
      diff = abs(t-rp[1])
      if diff < abs(t-best_match[1]):
        best_match = rp
    comment = "%.4g sec" % best_match[1]
    values.append((best_match[0], comment))
  return values

