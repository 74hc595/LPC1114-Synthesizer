#!/usr/bin/env python

from math import exp
import sys

CPU_FREQ = 50000000.0
ENV_UPDATE_DIVISOR = 200000.0
ALPHA_BITS = 15 # not to be confused with the cereal
SECS_PER_STEP = ENV_UPDATE_DIVISOR/CPU_FREQ

# first, compute the time in seconds we want for each step
# scale is piecewise
env_times = []

env_times.append(0.0)

# first half is linear from 0 to 1 second
for i in range(0x01,0xA0):
  env_times.append(1.60377*(i/255.0))
#  env_times.append(0.01*exp(8.4973*(i/255.0)))

# second half is exponential from 1 second to 10 seconds
for i in range(0xA0,256):
  env_times.append(0.0220673*exp(6.11624*(i/255.0)))

if len(env_times) != 256:
  print >> sys.stderr, "Expected exactly 256 time values (%d specified)" % len(env_times)
  sys.exit(1)


# second, for every possible "alpha" value, compute the rise time
# yeah it's brute force so what
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


last_time_taken = 0
rise_time_pairs =[]
for alpha in range(0,(1<<ALPHA_BITS)):
  time_taken = rise_time_for_alpha(alpha)
  if time_taken != last_time_taken:
    time_in_seconds = time_taken*SECS_PER_STEP
    rise_time_pairs.append((alpha,time_in_seconds))
    last_time_taken = time_taken


# last, find the best matching alpha value for each time
for t in env_times:
  best_match = (0, 0)
  for rp in rise_time_pairs:
    diff = abs(t-rp[1])
    if diff < abs(t-best_match[1]):
      best_match = rp
  print "  %d, /* %.4g sec */" % best_match
