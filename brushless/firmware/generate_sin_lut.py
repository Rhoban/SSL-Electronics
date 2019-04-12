#!/usr/bin/python3

from math import *
sin_input_resolution = 16384
sin_output_max = 8192
quadrant = sin_input_resolution//4

sinus = [
  int(sin_output_max*sin(2*pi*i/sin_input_resolution))
  for i in range(quadrant+1)
]
res = "static int16_t sin_samples[%s] = {"%(quadrant+1)
for i in range(quadrant):
  res += str(sinus[i])
  res += ", "
res += str(sinus[quadrant])
res += "};"
print(res)
