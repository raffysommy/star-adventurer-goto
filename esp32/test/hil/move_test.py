#!/usr/bin/env python3
"""Manual-move speeds (:Rn + :Me/:Mw/:Mn/:Ms), small moves (~1 min)."""
import time
from starmount import LX200, status, _http, SIDEREAL_COUNTS

lx = LX200()
ok = True
def check(name, cond, detail=''):
    global ok
    ok &= bool(cond)
    print(f'{"PASS" if cond else "FAIL"}  {name}  {detail}')
def ra_rate(sec=2.0):
    a = status(); time.sleep(sec); b = status()
    return (b['counts'] - a['counts']) / ((b['counts_ms'] - a['counts_ms']) / 1000) / SIDEREAL_COUNTS
def t1():
    return _http('GET', '/cmd?c=:i1').split()[0]

# max: ~63x with the keep-alive gaps. After a reversal the SA auto-stops once; the tracking
# watchdog restarts it within a few seconds, so the rate is checked after 5 s.
for idx, d, expect in ((5, 'w', 9), (5, 'e', -7), (2, 'e', 0), (1, 'w', 1.5), (9, 'w', 63)):
    lx.cmd(f':R{idx}#'); lx.cmd(f':M{d}#'); time.sleep(1.5)
    r = ra_rate()
    st = status()['state']
    lx.cmd(f':Q{d}#'); time.sleep(5)
    back = ra_rate(3)
    tol = max(0.05, abs(expect) * (0.08 if idx == 9 else 0.03))
    check(f'RA :R{idx} {d}', abs(r - expect) < tol and abs(back - 1) < 0.05 and t1() == '=AE0100',
          f'motor {r:+.2f}x (expect {expect:+g}x), state "{st}", after stop {back:.3f}x, T1 {t1()}')

d0 = status()['dec_steps']
lx.cmd(':R7#'); lx.cmd(':Mn#'); time.sleep(2); lx.cmd(':Qn#'); time.sleep(1)
d1 = status()['dec_steps']
check('DEC :R7 north 2 s', 150 < d1 - d0 < 360, f'{d1 - d0} steps (48x = 326 in 2 s, less any backlash take-up)')
lx.cmd(':Ms#'); time.sleep(2); lx.cmd(':Qs#'); time.sleep(1)
d2 = status()['dec_steps']
# start/stop are timed over Wi-Fi: +-0.2 s at ~160 steps/s on each end
check('DEC back south', abs(d2 - d0) < 130, f'{d2 - d0} steps from the start (backlash take-up at slew speed)')
s = status()
check('tracking again', s['state'] == 'TRACKING', f'{s["state"]}, :GU# {lx.cmd(":GU#")}')
print('PASS' if ok else 'FAIL')
