#!/usr/bin/env python3
"""Tracking rates, refraction, guide rate, backlash, DEC axis limits and the strict PEC
rule, on the mount (small moves only, ~3 min)."""
import time
from starmount import LX200, status, SIDEREAL_COUNTS

lx = LX200()
ok = True
def q(c):
    return lx.cmd(c)
def check(name, cond, detail=''):
    global ok
    ok &= bool(cond)
    print(f'{"PASS" if cond else "FAIL"}  {name}  {detail}')
def rate(seconds=15):
    # timed by the ESP's own register-read timestamps: laptop-side timing over HTTP adds
    # tenths of a percent of error (the "lunar 0.36% slow" of the first run)
    a = status(); time.sleep(seconds); b = status()
    return (b['counts'] - a['counts']) / ((b['counts_ms'] - a['counts_ms']) / 1000)

# --- tracking rates (refraction off so the ratio is clean)
q(':Tn#'); time.sleep(1)
sid = rate()
q(':TL#'); time.sleep(1)
lun = rate()
check('lunar rate', abs(lun / sid - 57.9 / 60.16427) < 0.0015, f'ratio {lun / sid:.4f} (expect {57.9 / 60.16427:.4f}), :GT# {q(":GT#")}')
q(':ST60.0#'); time.sleep(1)
check('custom :ST60.0 (solar)', q(':GT#').startswith('60.000'), q(':GT#'))
q(':TQ#'); time.sleep(1)
check('back to sidereal', q(':GT#').startswith('60.164'), q(':GT#'))
q(':Tr#'); time.sleep(2)
s = status()
check('refraction on', s['refraction'] and s['refraction_factor'] < 1, f'factor {s["refraction_factor"]:.6f}, :GU# {q(":GU#")}')

# --- guide rate 0.3x
q(':SX90,0.3#'); time.sleep(1)
check(':GX90#', q(':GX90#') == '0.30#', q(':GX90#'))
base = rate(4)
q(':Mgw5000#'); t0 = time.time(); a = status(); time.sleep(4.5); b = status()
w = (b['counts'] - a['counts']) / (time.time() - t0)
time.sleep(1)
check('RA west pulse at 0.3x', abs((w - base) / SIDEREAL_COUNTS - 0.3) < 0.05, f'{(w - base) / SIDEREAL_COUNTS:.3f}x extra')
d0 = status()['dec_steps']; q(':Mgn10000#'); time.sleep(11); d1 = status()['dec_steps']
check('DEC pulse at 0.3x', abs((d1 - d0) - 0.3 * 3.398 * 10) <= 1.5, f'{d1 - d0} steps (expect {0.3 * 3.398 * 10:.1f})')
q(':SX90,0.5#')

# --- backlash from the client
q(':$BD1000#'); time.sleep(0.5)
s = status()
check('backlash set', s['dec_backlash'] == 226 and q(':%BD#') == '1000#', f'{s["dec_backlash"]} steps, :%BD# {q(":%BD#")}')
check('RA backlash only 0', q(':$BR5#') == '0' and q(':$BR0#') == '1')
q(':$BD1107#')
check('backlash restored', status()['dec_backlash'] == 250, str(status()['dec_backlash']))

# --- DEC axis limits
pos = status()['dec_steps']
mech = pos * 360 / 292800 - 180
q(f':SXEC,{int(mech) - 5}#'); q(f':SXED,{mech + 0.02:.2f}#')
check(':GXEC/:GXED', q(':GXEC#') == f'{int(mech) - 5}#', f'{q(":GXEC#")} {q(":GXED#")}')
q(f':Sr{q(":GR#").strip("#")}#'); q(':Sd+20*00:00#')
check('GoTo past the DEC limit refused', q(':MS#') == '6')
q(':Mn#'); time.sleep(12); q(':Qn#')
s = status()
lim = round((mech + 0.02 + 180) * 292800 / 360)
check('manual move stops at the limit', s['dec_steps'] <= lim + 2, f'at {s["dec_steps"]}, limit {lim}')
q(':SXEC,-180#'); q(':SXED,180#')

# --- strict PEC: a synthetic table, play before guiding, refuse after, play after a GoTo
segs = int(q(':GXE8#').strip('#'))
for n in range(0, segs):
    lx.cmd(f':WR{n},{1 if n < segs // 2 else -1}#')
time.sleep(0.5)
q(':$QZ+#'); time.sleep(2)
st = q(':$QZ?#')
print('   after play request (no guiding since boot):', st)
guided = st.startswith('p')  # the guide pulses above count as guiding
check('strict: waits after guiding', guided, st)
here = q(':GR#').strip('#'); dec = q(':GD#').strip('#')
q(f':Sr{here}#'); q(f':Sd{dec}#'); q(':MS#')
t0 = time.time()
while time.time() - t0 < 30 and not q(':$QZ?#').startswith('P'):
    time.sleep(1)
check('strict: plays after a GoTo', q(':$QZ?#').startswith('P'), q(':$QZ?#'))
q(':$QZ-#'); q(':$QZZ#'); time.sleep(1)
check('PEC cleared', not status()['pec']['recorded'], str(status()['pec']))
print('PASS' if ok else 'FAIL')
