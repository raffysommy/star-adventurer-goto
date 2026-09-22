#!/usr/bin/env python3
"""PHD2-like guiding soak: random N/S/E/W pulse-guide commands (:Mg), then checks
that both axes moved exactly as commanded and that the RA motor was never
stopped/restarted during guiding. Moves the mount only by guide amounts.
"""
import random
import sys
import time

from starmount import DEC_GUIDE_STEPS, SIDEREAL_COUNTS, LX200, status

N = int(sys.argv[1]) if len(sys.argv) > 1 else 40
lx = LX200()
random.seed(3)
s0 = status()
t0 = time.time()
ra_extra = dec_cmd = 0.0
for _ in range(N):
    d, ms = random.choice('nsew'), random.randint(100, 1200)
    lx.cmd(f':Mg{d}{ms}#')
    if d in 'ew':  # east 1.5x, west 0.5x sidereal
        ra_extra += (0.5 if d == 'e' else -0.5) * SIDEREAL_COUNTS * ms / 1000
    else:
        dec_cmd += (1 if d == 'n' else -1) * DEC_GUIDE_STEPS * ms / 1000
    time.sleep(ms / 1000 + random.uniform(0.3, 0.7))
time.sleep(2)
lx.drain()
s1 = status()
dt = time.time() - t0

ra_got = s1['counts'] - s0['counts']
ra_exp = SIDEREAL_COUNTS * dt + ra_extra
dec_got = s1['dec_steps'] - s0['dec_steps']
restarts = s1['stalls'] - s0['stalls'] + s1['kicks'] - s0['kicks'] + s1['keep_alives'] - s0['keep_alives']
print(f'{N} pulses over {dt:.0f} s')
print(f'  RA : {ra_got} counts, expected {ra_exp:.0f} ({(ra_got - ra_exp) / SIDEREAL_COUNTS:+.2f} s of sidereal)')
print(f'  DEC: {dec_got:+d} steps, commanded {dec_cmd:+.2f}')
print(f'  RA motor restarts during guiding: {restarts}')
ok = abs(ra_got - ra_exp) / SIDEREAL_COUNTS < 0.5 and abs(dec_got - dec_cmd) <= 1 and restarts == 0
print('PASS' if ok else 'FAIL')
