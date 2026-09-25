#!/usr/bin/env python3
"""Polar-alignment / axis test: shoot + plate-solve, move RA (or DEC) by a known amount,
shoot + solve again, and recover the mount axis in the local alt/az frame.

Each solved frame gives the camera attitude in ICRS; converting three points of it to
AltAz (no refraction) at the exposure mid-time gives the attitude G in the ground frame.
Between two frames the camera only rotated about a mount axis, so R = G2 G1^T is a
rotation whose axis is that mount axis (Earth rotation drops out in the ground frame).

  pa.py shoot NAME            capture + solve, save NAME.json (attitude + time)
  pa.py axis A B              axis of rotation between frames A and B
  pa.py run [--delta 6]       shoot a, RA +delta, shoot b, report polar alignment
"""
import json, subprocess, sys, time, socket, glob, os
import numpy as np
from astropy.coordinates import SkyCoord, EarthLocation, AltAz
from astropy.time import Time
from astropy.wcs import WCS
from astropy.io import fits
import astropy.units as u

LAT, LON = 40.8728, 14.4377
LOC = EarthLocation(lat=LAT * u.deg, lon=LON * u.deg)
ESP = '192.168.5.157'
EXPOSURE = 2.0
DIR = os.path.dirname(os.path.abspath(__file__))


def http(method, path, data=''):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, b'wlo1')
    s.settimeout(5)
    s.connect((ESP, 80))
    s.sendall(f'{method} {path} HTTP/1.0\r\nHost: x\r\nContent-Type: application/x-www-form-urlencoded\r\n'
              f'Content-Length: {len(data)}\r\n\r\n{data}'.encode())
    r = b''
    while c := s.recv(4096):
        r += c
    return r.split(b'\r\n\r\n', 1)[1].decode()


def status():
    return json.loads(http('GET', '/api/status'))


def wait_idle(timeout=120):
    time.sleep(1.5)
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = status()
        if not s['state'].startswith('SLEWING') and not s['dec_moving']:
            return s
        time.sleep(0.5)
    raise RuntimeError('slew timeout')


def capture(name):
    """Capture to the card and download (gphoto2 is a snap: only $HOME is writable)."""
    for f in glob.glob(f'{DIR}/{name}.*'):
        os.remove(f)
    t0 = time.time()
    r = subprocess.run(['gphoto2', '--capture-image-and-download', '--filename', f'{name}.jpg',
                        '--force-overwrite'], cwd=DIR, capture_output=True, text=True, timeout=90)
    if not os.path.exists(f'{DIR}/{name}.jpg'):
        raise RuntimeError('capture failed: ' + r.stdout + r.stderr)
    # exposure starts ~0.3 s after the command; take mid-exposure
    return t0 + 0.3 + EXPOSURE / 2


def solve(name):
    r = subprocess.run(['solve-field', '--config', f'{DIR}/astrometry.cfg', f'{name}.jpg',
                        '--scale-units', 'arcsecperpix', '--scale-low', '3', '--scale-high', '12',
                        '--downsample', '4', '--no-plots', '--overwrite', '--new-fits', 'none',
                        '--cpulimit', '60'], cwd=DIR, capture_output=True, text=True)
    if not os.path.exists(f'{DIR}/{name}.wcs'):
        raise RuntimeError('solve failed')
    return WCS(fits.getheader(f'{DIR}/{name}.wcs'))


def unit(alt, az):
    alt, az = np.radians(alt), np.radians(az)
    return np.array([np.cos(alt) * np.cos(az), np.cos(alt) * np.sin(az), np.sin(alt)])  # N, E, Up


def attitude(wcs, t):
    """3x3 camera attitude in the ground frame (columns: boresight, image x, image y)."""
    nx, ny = wcs.pixel_shape if wcs.pixel_shape else (6000, 4000)
    cx, cy = nx / 2, ny / 2
    d = 500  # pixels
    pts = wcs.pixel_to_world([cx, cx + d, cx], [cy, cy, cy + d])
    aa = pts.transform_to(AltAz(obstime=Time(t, format='unix'), location=LOC))
    v = [unit(a, z) for a, z in zip(aa.alt.deg, aa.az.deg)]
    b = v[0]
    x = v[1] - b; x -= b * (b @ x); x /= np.linalg.norm(x)
    y = np.cross(b, x)
    return np.column_stack([b, x, y]), (pts[0].ra.deg, pts[0].dec.deg, aa[0].alt.deg, aa[0].az.deg)


def shoot(name):
    t = capture(name)
    s = status()
    w = solve(name)
    G, (ra, dec, alt, az) = attitude(w, t)
    rec = dict(name=name, t=t, G=G.tolist(), ra=ra, dec=dec, alt=alt, az=az,
               axis_ha=s['axis_ha'], dec_steps=s['dec_steps'])
    json.dump(rec, open(f'{DIR}/{name}.json', 'w'), indent=1)
    print(f'{name}: RA {ra:.4f} Dec {dec:.4f}  alt {alt:.2f} az {az:.2f}  axis_ha {s["axis_ha"]:.4f}')
    return rec


def axis(a, b):
    A, B = (json.load(open(f'{DIR}/{n}.json')) if isinstance(n, str) else n for n in (a, b))
    R = np.array(B['G']) @ np.array(A['G']).T
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    k = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    k /= np.linalg.norm(k)
    if k[2] < 0:  # report the axis end above the horizon
        k, ang = -k, -ang
    alt = np.degrees(np.arcsin(k[2]))
    az = np.degrees(np.arctan2(k[1], k[0])) % 360
    dt = B['t'] - A['t']
    return dict(alt=alt, az=az, angle=ang, dt=dt,
                commanded_ha=B['axis_ha'] - A['axis_ha'], dec_steps=B['dec_steps'] - A['dec_steps'])


def report(ax):
    p = unit(LAT, 0)
    m = unit(ax['alt'], ax['az'])
    sep = np.degrees(np.arccos(np.clip(p @ m, -1, 1)))
    daz = ((ax['az'] + 180) % 360) - 180
    print(f"rotation {ax['angle']:+.3f} deg in {ax['dt']:.1f} s  (register moved {ax['commanded_ha']:+.3f} deg"
          f", sidereal over dt {ax['dt'] * 360.9856 / 86400:.3f}, DEC steps {ax['dec_steps']:+d})")
    print(f"axis: alt {ax['alt']:.3f}  az {ax['az']:.3f}")
    print(f"pole: alt {LAT:.3f}  az 0")
    print(f"polar error {sep * 60:.1f}'  -> altitude {(ax['alt'] - LAT) * 60:+.1f}'  azimuth {daz * 60 * np.cos(np.radians(LAT)):+.1f}' "
          f"(on-sky; {daz * 60:+.1f}' of az)")


def main():
    cmd = sys.argv[1]
    if cmd == 'shoot':
        shoot(sys.argv[2])
    elif cmd == 'axis':
        report(axis(sys.argv[2], sys.argv[3]))
    elif cmd == 'run':
        delta = float(sys.argv[sys.argv.index('--delta') + 1]) if '--delta' in sys.argv else 6.0
        a = shoot('pa_a')
        s = status()
        http('POST', '/api/goto_ha', f'ha={s["axis_ha"] + delta:.4f}')
        wait_idle()
        b = shoot('pa_b')
        report(axis(a, b))


if __name__ == '__main__':
    main()
