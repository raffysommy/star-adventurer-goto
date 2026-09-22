#!/usr/bin/env python3
"""GoTo test through the LX200 port, as a client would do it: sync on the current
position, slew RA 5 deg CW + DEC +5, then RA 3 deg CCW + DEC to +2, then back.
THE MOUNT MOVES (a few degrees on both axes). Needs the RA register away from
zero (HA >= ~2 deg) and room for DEC to go to +5.
"""
import time

from starmount import LX200, dms_to_deg, fmt_dec, fmt_ra, hms_to_deg, status

lx = LX200()


def goto(ra, dec):
    print(f'\nGoTo RA {fmt_ra(ra)} DEC {fmt_dec(dec)}:', lx.cmd(f':Sr{fmt_ra(ra)}#'), lx.cmd(f':Sd{fmt_dec(dec)}#'),
          ':MS ->', lx.cmd(':MS#'))
    t0, last = time.time(), -9
    while True:
        time.sleep(0.5)
        busy = lx.cmd(':D#') != '#'
        t = time.time() - t0
        if t - last >= 3 or not busy:
            last = t
            s = status()
            print(f'  t={t:5.1f}s {s["state"]:18} :GR={lx.cmd(":GR#")} :GD={lx.cmd(":GD#")} stalls={s["stalls"]}',
                  flush=True)
        if not busy or t > 150:
            break
    time.sleep(6)  # let tracking settle after the direction change
    gr, gd = hms_to_deg(lx.cmd(':GR#')), dms_to_deg(lx.cmd(':GD#'))
    err_ra, err_dec = (gr - ra) * 240, (gd - dec) * 3600
    print(f'  arrived: RA error {err_ra:+.0f} s, DEC error {err_dec:+.0f} arcsec')
    return abs(err_ra) <= 2 and abs(err_dec) <= 10


ra0 = hms_to_deg(lx.cmd(':GR#'))
dec0 = dms_to_deg(lx.cmd(':GD#'))
print('sync on current position', fmt_ra(ra0), fmt_dec(dec0), lx.cmd(f':Sr{fmt_ra(ra0)}#'),
      lx.cmd(f':Sd{fmt_dec(dec0)}#'), repr(lx.cmd(':CM#')))
time.sleep(8)
ok = goto(ra0 - 5, dec0 + 5) & goto(ra0 - 2, dec0 + 2) & goto(ra0, dec0)
print('\nPASS' if ok else '\nFAIL')
