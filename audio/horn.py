#!/usr/bin/python

import math
import wave
import struct
import textwrap

# Generate a horn sound
F_sample    = 6400      # 6.4kHz sample rate
# Horn frequencies and amplitudes
A_horn      = 0.8
FA_horn     = [
        (485,   10**(-20.4/20)),
        (618,   10**(-15.4/20)),
        (762,   10**(-7.3/20)),
        (967,   10**(-13.4/20)),
        (1147,  10**(-15.0/20)),
]
T_attack    = 0.029     # Rise time, seconds
T_hold      = 1.000     # Hold time, seconds
T_decay     = 0.300     # Decay time, seconds
N_samples   = int(math.ceil((T_attack+T_hold+T_decay) * F_sample))

T_hold_end      = T_attack + T_hold
duration        = T_hold_end + T_decay
def amplitude(t):
    if t < T_attack:
        return (1.0 + math.sin((t*math.pi)/(2*T_attack)))/2.0
    elif t < T_hold_end:
        return 1.0
    else:
        t -= T_hold_end
        return (1.0 + math.cos((math.pi*t)/T_decay))/2.0
horn = lambda t : amplitude(t) * sum([  \
    a * A_horn * math.sin(2*math.pi*f*t)\
    for f, a in FA_horn                 \
])
samples = []
for n in range(0, int(duration*F_sample)):
    t = float(n)/float(F_sample)
    samples.append(int(127*horn(t)) + 127)
    assert samples[-1] >= 0, '%s <= 0' % samples[-1]
    assert samples[-1] <= 255, '%s > 255' % samples[-1]

num_samples = len(samples)

# C output
with file('hornsnd.c','w') as f:
    f.write('#include "hornsnd.h"\n')
    f.write('const uint8_t horn[%s] PROGMEM = {\n' % num_samples)
    f.write('\n'.join(textwrap.wrap(', '.join([str(s) for s in samples]),
        initial_indent='\t', subsequent_indent='\t',
        expand_tabs=False)))
    f.write('\n};\n')
with file('hornsnd.h','w') as f:
    f.write('#include <stdint.h>\n')
    f.write('#include <avr/pgmspace.h>\n')
    f.write('#define HORN_RATE        (%s)\n' % F_sample)
    f.write('#define HORN_LOOP_OFFSET (%s)\n' % int(T_attack*F_sample))
    f.write('#define HORN_LOOP_SZ     (%s)\n' % int(T_hold_end*F_sample))
    f.write('#define HORN_SZ          (%s)\n' % num_samples)
    f.write('const uint8_t horn[%s] PROGMEM;\n' % num_samples)
# For the sake of analysis, we'll dump wav audio too
f = wave.open('hornsnd.wav','w')
f.setnchannels(1)
f.setsampwidth(1)
f.setframerate(F_sample)
f.setnframes(num_samples)
f.setcomptype('NONE','not compressed')
f.writeframes(''.join([
    struct.pack('B', s) for s in samples
]))
f.close()
