#!/usr/bin/env python3
"""Star Adventurer RA rate vs T1: set raw step periods and measure how fast the position
register advances, timed by the ESP's own register-read timestamps (counts_ms).
Refraction and PEC are switched off meanwhile; tracking is restored at the end."""
import json, sys, time
from starmount import LX200, status, _http

T1S = [280, 287, 300, 330, 360, 400, 415, 421, 425, 428, 430, 432, 435, 439, 447, 460, 500, 600, 700, 800, 861, 900]
SECONDS = 25
out = open(sys.argv[1] if len(sys.argv) > 1 else 'rate_calibration.csv', 'w')
out.write('t1,cps,samples\n')
lx = LX200()
lx.cmd(':$QZ-#'); lx.cmd(':Tn#'); lx.cmd(':TQ#')
time.sleep(2)

def set_t1(t1):
    h = f'{t1:06X}'
    le = h[4:6] + h[2:4] + h[0:2]
    r = _http('GET', f'/cmd?c=:I1{le}')
    got = _http('GET', '/cmd?c=:i1')
    return got.split()[0]

for t1 in T1S:
    echo = set_t1(t1)
    time.sleep(3)
    pts = []
    t0 = time.time()
    while time.time() - t0 < SECONDS:
        s = status()
        if not pts or s['counts_ms'] != pts[-1][0]:
            pts.append((s['counts_ms'], s['counts']))
        time.sleep(0.5)
    # least-squares slope, counts per second of ESP time
    n = len(pts)
    mt = sum(p[0] for p in pts) / n; mc = sum(p[1] for p in pts) / n
    b = sum((p[0] - mt) * (p[1] - mc) for p in pts) / sum((p[0] - mt) ** 2 for p in pts) * 1000
    out.write(f'{t1},{b:.4f},{n}\n'); out.flush()
    print(f'T1 {t1:4d} (mount {echo})  {b:9.3f} counts/s   62338/T1 = {62338 / t1:9.3f}   ratio {b * t1 / 62338:.5f}', flush=True)

set_t1(430)
lx.cmd(':Tr#')
print('restored T1 430, refraction on')
