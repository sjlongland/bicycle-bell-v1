#!/usr/bin/python

import math
import wave
import struct
import textwrap

# Generate a bell sound
F_sample    = 6400      # 6.4kHz sample rate
F_bell      = 2063      # Fundamental frequency
F_bell_mod  = 15.625    # Modulated amplitude
F_reverb    = 3.77      # Reverberation frequency
A_reverb    = 0.07      # Reverberation amplitude
duration    = 1.0       # Duration
decay       = 1.0       # Decay

samples = []
bell = lambda t : ((1.0+math.cos(math.pi*t/duration))/2) * \
        (decay ** t) * \
        (1.0 - (A_reverb * math.cos(2*math.pi*F_reverb*t)) - (A_reverb/2)) * \
        (math.cos(2*math.pi*F_bell*t) * \
         math.sin(2*math.pi*F_bell_mod*t))
for n in range(0, int(duration*F_sample)):
    t = float(n)/float(F_sample)
    samples.append(int(127*bell(t)) + 127)
    assert samples[-1] >= 0, '%s <= 0' % samples[-1]
    assert samples[-1] <= 255, '%s > 255' % samples[-1]

num_samples = len(samples)

# C output
with file('bellsnd.c','w') as f:
    f.write('#include "bellsnd.h"\n')
    f.write('const uint8_t bell[%s] PROGMEM = {\n' % num_samples)
    f.write('\n'.join(textwrap.wrap(', '.join([str(s) for s in samples]),
        initial_indent='\t', subsequent_indent='\t',
        expand_tabs=False)))
    f.write('\n};\n')
with file('bellsnd.h','w') as f:
    f.write('#include <stdint.h>\n')
    f.write('#include <avr/pgmspace.h>\n')
    f.write('#define BELL_RATE  (%s)\n' % F_sample)
    f.write('#define BELL_SZ    (%s)\n' % num_samples)
    f.write('const uint8_t bell[%s] PROGMEM;\n' % num_samples)
# For the sake of analysis, we'll dump wav audio too
f = wave.open('bellsnd.wav','w')
f.setnchannels(1)
f.setsampwidth(1)
f.setframerate(F_sample)
f.setnframes(num_samples)
f.setcomptype('NONE','not compressed')
f.writeframes(''.join([
    struct.pack('B', s) for s in samples
]))
f.close()
