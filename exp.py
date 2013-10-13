#!/usr/bin/env python

from math import exp

F_CPU = 48000000.0
ENV_FREQ = F_CPU/200000.0
STEP_LENGTH = 1.0/ENV_FREQ
ENV_MAX = 65535

# exponential coefficients: y=A*exp(B*x)
A1 = 0.01
B1 = 9.21034

A2 = 0.1
B2 = 4.60517

print "  %d, /* %g sec */" % (ENV_MAX, 0)

for i in range(1,128):
  seconds = A1*exp(B1*(i/255.0))
  steps = seconds/STEP_LENGTH
  amt = round(ENV_MAX/steps)
  print "  %d, /* %g sec */" % (int(amt), seconds)

for i in range (128,256):
  seconds = A2*exp(B2*(i/255.0))
  steps = seconds/STEP_LENGTH
  amt = round(ENV_MAX/steps)
  print "  %d, /* %g sec */" % (int(amt), seconds)


