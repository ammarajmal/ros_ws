#!/usr/bin/env python3
import numpy as np
from scipy.signal import resample
t = list(range(3))
x = list(range(1, 4))

up_factor = 2
upsampled_length = len(t) * up_factor
# upsampled_time = np.linspace(0, len(t) - 1, len(t) * up_factor)
upsampled_time2 = np.linspace(t[0], t[-1], len(t) * up_factor)

# Now, upsample the x signal using linear interpolation and resample.
x_upsample_linear = np.interp(upsampled_time, t, x)
x_upsample_resample = resample(x, upsampled_length)


print('t:', t)
print('x:',x)
print('upsampling factor:', up_factor)
print('upsampled_length:',upsampled_length)
print('upsampled_time :',upsampled_time)
print('upsampled_time2:',upsampled_time2)
print('len(upsampled_time):', len(upsampled_time))
print('Upsampling x by linear interpolation:', x_upsample_linear)
print('Upsampling x by resample:', x_upsample_resample)