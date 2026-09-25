#!/usr/bin/env python3
"""Record RA periodic error from the camera's live view (no shutter actuations).

Streams MJPEG live-view frames with `gphoto2 --capture-movie`, tracks the centroid of
the brightest star and logs time, x, y, mount register. Frames are averaged into 2 s
bins to beat down seeing. Axis directions come from the calibration moves done by the
analysis, so here only pixels are logged.

  pe_record.py MINUTES [out.csv]
"""
import subprocess, sys, time, io, threading
import numpy as np
from PIL import Image
from scipy import ndimage
import pa

minutes = float(sys.argv[1])
out = sys.argv[2] if len(sys.argv) > 2 else f'{pa.DIR}/pe.csv'
BOX = 24  # tracking half-window, px (24"/px)

reg = {'ha': None}


def poll_register():
    while True:
        try:
            reg['ha'] = pa.status()['axis_ha']
        except Exception:
            pass
        time.sleep(1)


threading.Thread(target=poll_register, daemon=True).start()


def frames(stream):
    buf = b''
    while True:
        chunk = stream.read(65536)
        if not chunk:
            return
        buf += chunk
        while True:
            s = buf.find(b'\xff\xd8')
            e = buf.find(b'\xff\xd9', s + 2)
            if s < 0 or e < 0:
                break
            yield time.time(), buf[s:e + 2]
            buf = buf[e + 2:]


def find_star(a):
    bg = ndimage.uniform_filter(a, 31)
    d = ndimage.gaussian_filter(a - bg, 1.2)
    d[:, :8] = d[:, -8:] = d[:8] = d[-8:] = 0
    y, x = np.unravel_index(np.argmax(d), d.shape)
    return x, y


def centroid(a, x0, y0):
    x0, y0 = int(round(x0)), int(round(y0))
    y1, y2, x1, x2 = max(0, y0 - BOX), y0 + BOX, max(0, x0 - BOX), x0 + BOX
    w = a[y1:y2, x1:x2]
    w = w - np.median(w)
    # re-centre on the peak, then a small weighted window
    py, px = np.unravel_index(np.argmax(ndimage.gaussian_filter(w, 1)), w.shape)
    r = 5
    sub = w[max(0, py - r):py + r + 1, max(0, px - r):px + r + 1].clip(0)
    if sub.sum() <= 0:
        return None
    yy, xx = np.mgrid[max(0, py - r):max(0, py - r) + sub.shape[0], max(0, px - r):max(0, px - r) + sub.shape[1]]
    return x1 + (xx * sub).sum() / sub.sum(), y1 + (yy * sub).sum() / sub.sum(), w.max()


p = subprocess.Popen(['gphoto2', f'--capture-movie={int(minutes * 60)}s', '--stdout'],
                     stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
f = open(out, 'w')
f.write('t,x,y,peak,axis_ha,n\n')
pos = None
acc = []
t_bin = None
n = 0
for t, jpg in frames(p.stdout):
    a = np.asarray(Image.open(io.BytesIO(jpg)).convert('L'), dtype=float)
    if pos is None:
        pos = find_star(a)
        print('tracking star at', pos, flush=True)
    c = centroid(a, *pos)
    if c is None:
        continue
    pos = c[:2]
    acc.append((t, *c))
    n += 1
    if t_bin is None:
        t_bin = t
    if t - t_bin >= 2:
        m = np.mean(acc, axis=0)
        f.write(f'{m[0]:.2f},{m[1]:.3f},{m[2]:.3f},{m[3]:.0f},{reg["ha"]},{len(acc)}\n')
        f.flush()
        acc, t_bin = [], None
    if n % 300 == 0:
        print(f'{n} frames, star at {pos[0]:.1f},{pos[1]:.1f}', flush=True)
print('done', n, 'frames')
