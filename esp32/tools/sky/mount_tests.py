#!/usr/bin/env python3
"""Mount tests through a narrow window, measured with plate solves.

  mount_tests.py dec      DEC scale, direction, backlash, RA/DEC orthogonality
  mount_tests.py sync     solve + LX200 sync (JNow)
  mount_tests.py goto     small GoTos after sync, solved error
  mount_tests.py guide    guide pulse rates (RA and DEC)
  mount_tests.py drift [minutes]   tracking drift, one frame per minute
"""
import sys, time, socket, json
import numpy as np
from astropy.coordinates import SkyCoord, FK5
from astropy.time import Time
import astropy.units as u
import pa

RA_AXIS = pa.unit(40.996, 0.172)   # measured polar axis (alt, az), after alignment
STEPS_PER_DEG = 292800 / 360


class LX:
    def __init__(self):
        self.s = socket.socket()
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'wlo1')
        self.s.settimeout(3)
        self.s.connect((pa.ESP, 5001))

    def cmd(self, c, reply=True):
        self.s.sendall(c.encode())
        if not reply:
            return ''
        d = b''
        single = (':Sr', ':Sd', ':MS', ':Sg', ':St', ':SG', ':SL', ':SC')  # OnStep: '1'/'0'
        try:
            while not d.endswith(b'#') and not (c.startswith(single) and d):
                d += self.s.recv(256)
        except socket.timeout:
            pass
        return d.decode('latin1')


def fmt_ra(deg):
    t = round((deg % 360) * 240)
    return f'{t // 3600:02d}:{t % 3600 // 60:02d}:{t % 60:02d}'


def fmt_dec(deg):
    a = round(abs(deg) * 3600)
    return f'{"-" if deg < 0 else "+"}{a // 3600:02d}*{a % 3600 // 60:02d}:{a % 60:02d}'


def jnow(rec):
    c = SkyCoord(rec['ra'] * u.deg, rec['dec'] * u.deg).transform_to(FK5(equinox=Time(rec['t'], format='unix')))
    return c.ra.deg, c.dec.deg


def rot(axis, deg):
    k = axis / np.linalg.norm(axis)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    a = np.radians(deg)
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def dec_rotation(a, b):
    """Rotation between frames with the RA motion (register delta about the measured
    polar axis) removed: what is left is the DEC axis rotation."""
    R = np.array(b['G']) @ np.array(a['G']).T
    R = rot(RA_AXIS, -(b['axis_ha'] - a['axis_ha'])) @ R
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    k = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    k /= np.linalg.norm(k)
    return ang, k


def sep(a, b):
    return SkyCoord(a['ra'] * u.deg, a['dec'] * u.deg).separation(SkyCoord(b['ra'] * u.deg, b['dec'] * u.deg)).deg


def dec_move(steps):
    pa.http('POST', '/api/dec', f'action=move&steps={steps}')
    return pa.wait_idle()


def test_dec():
    print('== DEC: +1 deg, -1 deg (back), -1 deg, +1 deg (back)')
    f = [pa.shoot('d0')]
    for i, st in enumerate((813, -813, -813, 813), 1):
        dec_move(st)
        f.append(pa.shoot(f'd{i}'))
    for i in range(4):
        a, b = f[i], f[i + 1]
        ang, k = dec_rotation(a, b)
        # sign: + if the boresight moved toward the north celestial pole
        ddec = b['dec'] - a['dec']
        ortho = np.degrees(np.arccos(abs(k @ RA_AXIS)))
        print(f'  d{i}->d{i + 1}: steps {b["dec_steps"] - a["dec_steps"]:+5d}  rotation {ang:.4f} deg '
              f'({(b["dec_steps"] - a["dec_steps"]) / ang if ang else 0:+.1f} steps/deg, nominal {STEPS_PER_DEG:.1f})  '
              f'boresight dDec {ddec * 60:+.2f}\'  DEC axis vs RA axis {ortho:.3f} deg')
    print(f'  return error after +1/-1: {sep(f[0], f[2]) * 3600:.1f}"   after -1/+1: {sep(f[2], f[4]) * 3600:.1f}"')


def test_sync():
    print('== sync')
    r = pa.shoot('sy')
    ra, dec = jnow(r)
    lx = LX()
    print('  before :GR', lx.cmd(':GR#'), ':GD', lx.cmd(':GD#'))
    print('  :Sr', fmt_ra(ra), lx.cmd(f':Sr{fmt_ra(ra)}#'), ' :Sd', fmt_dec(dec), lx.cmd(f':Sd{fmt_dec(dec)}#'))
    print('  :CM', lx.cmd(':CM#'))
    time.sleep(2)
    print('  after  :GR', lx.cmd(':GR#'), ':GD', lx.cmd(':GD#'), ' status', {k: pa.status()[k] for k in ('axis_ha', 'dec_steps', 'state')})


def goto(ra, dec):
    lx = LX()
    lx.cmd(f':Sr{fmt_ra(ra)}#')
    lx.cmd(f':Sd{fmt_dec(dec)}#')
    print('  :MS', repr(lx.cmd(':MS#')))
    time.sleep(2)
    pa.wait_idle(180)
    time.sleep(3)  # settle


def test_goto():
    print('== GoTo (targets relative to the synced position, JNow)')
    r0 = json.load(open(f'{pa.DIR}/sy.json'))
    ra0, dec0 = jnow(r0)
    for i, (dra, ddec) in enumerate(((0, 1.2), (0, 0.4), (2, 1.0), (0, 0.6), (-1, 0.2), (0, 1.0))):
        tra, tdec = ra0 + dra, dec0 + ddec
        print(f' target {i}: RA {dra:+} deg, Dec {ddec:+} deg')
        goto(tra, tdec)
        r = pa.shoot(f'g{i}')
        ra, dec = jnow(r)
        e_ra = ((ra - tra + 180) % 360 - 180) * np.cos(np.radians(tdec)) * 60
        e_dec = (dec - tdec) * 60
        lx = LX()
        print(f'  solved error: RA {e_ra:+.2f}\'  Dec {e_dec:+.2f}\'  total {np.hypot(e_ra, e_dec):.2f}\'   '
              f'mount says {lx.cmd(":GR#")} {lx.cmd(":GD#")}')


def pulse(d, ms):
    lx = LX()
    lx.cmd(f':Mg{d}{ms:04d}#', reply=False)
    time.sleep(ms / 1000 + 1.5)


def test_guide():
    seq = [('w', 5000), ('e', 5000), ('w', 5000)] + [('n', 10000)] * 5 + [('s', 10000)] * 5
    print('== guide pulses, rate in x sidereal (15.04"/s)')
    prev = pa.shoot('p0')
    for i, (d, ms) in enumerate(seq, 1):
        pulse(d, ms)
        r = pa.shoot(f'p{i}')
        dra = ((r['ra'] - prev['ra'] + 180) % 360 - 180) * np.cos(np.radians(r['dec'])) * 3600
        ddec = (r['dec'] - prev['dec']) * 3600
        print(f'  {d} {ms / 1000:.0f}s: dRA {dra:+6.1f}"  dDec {ddec:+6.1f}"  -> {np.hypot(dra, ddec) / (ms / 1000) / 15.04:.3f}x sidereal'
              f'  (dec steps {r["dec_steps"] - prev["dec_steps"]:+d}, dt {r["t"] - prev["t"]:.0f}s)')
        prev = r


def test_drift(minutes):
    print(f'== tracking drift, {minutes} min')
    f = []
    for i in range(minutes + 1):
        t0 = time.time()
        f.append(pa.shoot(f'dr{i}'))
        d = f[-1]
        dra = ((d['ra'] - f[0]['ra'] + 180) % 360 - 180) * np.cos(np.radians(d['dec'])) * 3600
        print(f'  {(d["t"] - f[0]["t"]) / 60:5.2f} min  dRA {dra:+6.1f}"  dDec {(d["dec"] - f[0]["dec"]) * 3600:+6.1f}"')
        if i < minutes:
            time.sleep(max(0, 60 - (time.time() - t0)))
    t = np.array([d['t'] - f[0]['t'] for d in f])
    for name, v in (('RA', [((d['ra'] - f[0]['ra'] + 180) % 360 - 180) * np.cos(np.radians(d['dec'])) * 3600 for d in f]),
                    ('Dec', [(d['dec'] - f[0]['dec']) * 3600 for d in f])):
        p = np.polyfit(t, v, 1)
        res = np.array(v) - np.polyval(p, t)
        print(f'  {name}: linear drift {p[0] * 60:+.2f}"/min, residual rms {res.std():.2f}" p-p {np.ptp(res):.2f}"')


if __name__ == '__main__':
    c = sys.argv[1]
    {'dec': test_dec, 'sync': test_sync, 'goto': test_goto, 'guide': test_guide,
     'drift': lambda: test_drift(int(sys.argv[2]) if len(sys.argv) > 2 else 10)}[c]()
