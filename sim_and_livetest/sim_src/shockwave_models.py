
import numpy as np
from numpy import array, sin, cos, exp
from numpy import random

def sinusoidal(t, amplitude, frequency, phase_shift):
    return amplitude * sin(frequency * t + phase_shift)

def spiky(t, amplitude, frequency, phase_shift):
    return random.uniform(-1.0, 1.0)*amplitude*100

def spiky_sinusoidal(t, amplitude, frequency, phase_shift):
    return sinusoidal(t, amplitude, frequency, phase_shift) + spiky(t, amplitude, frequency, phase_shift)