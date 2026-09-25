#!/usr/bin/env python3
"""PEC end to end on the mount, no sky needed (~17 min, RA tracking only).

1. Record: act as the guider for one worm turn, sending RA pulses that correct a known
   periodic error (a sine of AMP counts over the worm period).
2. Check the recorded table (:VR[n]#) has that shape.
3. Play: after the 60 s guiding-quiet rule, the register itself should follow the sine.
"""
import math
import sys
import time

from starmount import LX200, status

AMP = 600  # counts (~62") peak periodic error to correct
lx = LX200()


def q(c):
    return lx.cmd(c)


worm = int(q(':GXE7#').strip('#'))
segs = int(q(':GXE8#').strip('#'))
cps = float(q(':GXE6#').strip('#'))
period = worm / cps
print(f'worm {worm} counts, {segs} segments, {cps:.3f} counts/s, period {period:.0f} s')

q(':$QZZ#')
time.sleep(1)
q(':$QZ/#')
time.sleep(1)
print('record:', q(':$QZ?#'))
t0 = time.time()
rate_ms = 0.5 * cps / 1000  # counts per ms of pulse (0.5x sidereal extra)
while time.time() - t0 < period + 15:
    t = time.time() - t0
    # correction the guider sends over the next 2 s: derivative of the PE (in counts)
    c = AMP * 2 * math.pi / period * math.cos(2 * math.pi * t / period) * 2.0
    ms = int(abs(c) / rate_ms)
    if ms >= 10:
        lx.cmd(f':Mg{"w" if c > 0 else "e"}{ms:04d}#')
    time.sleep(2.0)
    if int(t) % 60 < 2:
        print(f'  {t:5.0f} s  {q(":$QZ?#")}  pec {status()["pec"]}', flush=True)
lx.drain()
st = status()['pec']
print('after recording:', q(':$QZ?#'), st)
if not st['recorded']:
    sys.exit('FAIL: nothing recorded')

table = [int(q(f':VR{n}#').strip('#')) for n in range(0, segs, 1)]
exp = [AMP * 2 * math.pi / period * math.cos(2 * math.pi * n / segs) for n in range(segs)]
num = sum(a * b for a, b in zip(table, exp))
corr = num / math.sqrt(sum(a * a for a in table) * sum(b * b for b in exp))
gain = num / sum(b * b for b in exp)
print(f'table vs expected: correlation {corr:.3f}, gain {gain:.2f} (peak {max(map(abs, table))} counts/seg)')

q(':$QZ+#')
print('play requested:', q(':$QZ?#'), '- waiting for 60 s without guide pulses')
while status()['pec']['state'] != 2:
    time.sleep(2)
print('playing:', q(':$QZ?#'))
samples = []
t1 = time.time()
while time.time() - t1 < 300:
    s = status()
    samples.append((time.time(), s['counts'], s['pec']['segment']))
    time.sleep(3)
# register minus the best straight line should follow the integral of the table
ts = [s[0] - samples[0][0] for s in samples]
cs = [s[1] - samples[0][1] for s in samples]
n = len(ts)
mt, mc = sum(ts) / n, sum(cs) / n
b = sum((t - mt) * (c - mc) for t, c in zip(ts, cs)) / sum((t - mt) ** 2 for t in ts)
res = [c - mc - b * (t - mt) for t, c in zip(ts, cs)]
# expected position from the table: cumulative sum up to each sample's segment
cum = [0.0]
for v in table:
    cum.append(cum[-1] + v)
seg0 = samples[0][2]
expd = []
for s in samples:
    k = (s[2] - seg0) % segs
    expd.append(sum(table[(seg0 + i) % segs] for i in range(k)))
me = sum(expd) / n
eb = sum((t - mt) * (e - me) for t, e in zip(ts, expd)) / sum((t - mt) ** 2 for t in ts)
eres = [e - me - eb * (t - mt) for t, e in zip(ts, expd)]
num = sum(a * c for a, c in zip(res, eres))
corr2 = num / math.sqrt(sum(a * a for a in res) * sum(c * c for c in eres))
gain2 = num / sum(c * c for c in eres)
rms = math.sqrt(sum((a - gain2 * c) ** 2 for a, c in zip(res, eres)) / n)
print(f'playback: register follows the table with correlation {corr2:.3f}, gain {gain2:.2f}, '
      f'residual {rms:.1f} counts rms (curve p-p {max(eres) - min(eres):.0f} counts)')
q(':$QZ-#')
print('stopped:', q(':$QZ?#'))
ok = corr > 0.95 and 0.8 < gain < 1.2 and corr2 > 0.95 and 0.8 < gain2 < 1.2
print('PASS' if ok else 'FAIL')
