#!/usr/bin/env python3
"""OnStepX stand-in for capturing what a client (INDI OnStep driver) sends.

Replies follow OnStepX conventions: known commands get their reply, anything
else gets "0" (OnStepX's numeric "error / not supported" reply, no '#').
Every command and reply is logged with a timestamp.
"""
import socket, sys, threading, time, datetime

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 9998
LOG = open(sys.argv[2] if len(sys.argv) > 2 else 'capture.log', 'a', buffering=1)

state = {'ra': 17.0, 'dec': 5.0, 'tracking': True, 'pier': 'W'}


def fmt_ra(h):
    s = h * 3600
    return f'{int(s // 3600):02d}:{int(s % 3600 // 60):02d}:{s % 60:07.4f}'


def fmt_dec(d):
    sgn = '-' if d < 0 else '+'
    s = abs(d) * 3600
    return f'{sgn}{int(s // 3600):02d}*{int(s % 3600 // 60):02d}:{s % 60:06.3f}'


def status_gu():
    r = ''
    if not state['tracking']:
        r += 'n'
    r += 'N'           # no goto
    r += 'p'           # not parked
    r += 'E'           # GEM
    r += {'E': 'T', 'W': 'W'}.get(state['pier'], 'o')
    r += '2' + '2' + '0'  # pulse-guide rate select, guide rate select, error code
    return r + '#'


REPLIES = {
    ':GVP#': 'On-Step#',
    ':GVN#': '10.26a#',
    ':GVD#': 'Sep 08 2026#',
    ':GVT#': '12:00:00#',
    ':GU#': status_gu,
    ':Gm#': lambda: state['pier'] + '#',
    ':GR#': lambda: fmt_ra(state['ra']) + '#',
    ':GD#': lambda: fmt_dec(state['dec']) + '#',
    ':GRH#': lambda: fmt_ra(state['ra']) + '#',
    ':GDH#': lambda: fmt_dec(state['dec']) + '#',
    ':Gt#': '+40*52:22#',
    ':Gg#': '-014*26:16#',
    ':GG#': '-02:00#',
    ':GL#': lambda: datetime.datetime.now().strftime('%H:%M:%S') + '#',
    ':GC#': lambda: datetime.datetime.now().strftime('%m/%d/%y') + '#',
    ':GT#': '60.16427#',
    ':GX90#': '0.50#',
    ':Gc#': '24#',
    ':GM#': 'Site 1#', ':GN#': 'Site 2#', ':GO#': 'Site 3#', ':GP#': 'Site 4#',
    ':GtH#': '+40*52:22.000#',
    ':GgH#': '-014*26:16.000#',
    ':%BD#': '1100#',   # arcsec (~250 steps)
    ':%BR#': '0#',
    ':GX98#': 'N#',     # no rotator
    ':Gh#': '-10*#',
    ':Go#': '85*#',
    ':GXE9#': '120#',   # minutes past meridian, east
    ':GXEA#': '90#',    # minutes past meridian, west
    ':GX95#': '0#',
    ':GX96#': 'B#',
}


def reply_for(cmd):
    r = REPLIES.get(cmd)
    if r is None:
        if cmd.startswith(':S') or cmd.startswith(':St') or cmd.startswith(':Sg'):
            return '1'   # accept settings
        return '0'
    return r() if callable(r) else r


def handle(conn, addr):
    buf = b''
    t0 = time.time()
    while True:
        try:
            d = conn.recv(256)
        except OSError:
            break
        if not d:
            break
        buf += d
        while buf:
            if buf[0] == 6:  # ACK
                buf = buf[1:]
                conn.sendall(b'G')
                LOG.write(f'{time.time() - t0:8.3f} <ACK> -> G\n')
                continue
            i = buf.find(b'#')
            if i < 0:
                break
            cmd = buf[:i + 1].decode('latin1').strip()
            buf = buf[i + 1:]
            r = reply_for(cmd)
            conn.sendall(r.encode('latin1'))
            LOG.write(f'{time.time() - t0:8.3f} {cmd:<16} -> {r!r}\n')
    conn.close()


s = socket.socket()
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('127.0.0.1', PORT))
s.listen(4)
print('stub on', PORT, flush=True)
while True:
    c, a = s.accept()
    threading.Thread(target=handle, args=(c, a), daemon=True).start()
