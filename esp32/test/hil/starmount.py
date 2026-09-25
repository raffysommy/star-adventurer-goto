"""Helpers for the hardware-in-the-loop tests: LX200 and HTTP access to the ESP.

Environment:
  STARMOUNT_HOST   ESP address (default starmount.local)
  STARMOUNT_IFACE  bind sockets to this interface, e.g. wlo1 - needed when a VPN
                   (Tailscale subnet route) captures the LAN route
"""
import json
import os
import socket
import time
from urllib.parse import urlencode

HOST = os.environ.get('STARMOUNT_HOST', 'starmount.local')
IFACE = os.environ.get('STARMOUNT_IFACE')
LX200_PORT = 5001
SIDEREAL_COUNTS = 144.96  # RA register counts per second at sidereal rate
DEC_GUIDE_STEPS = 1.699   # DEC steps per second at guide rate (0.5x sidereal)


def _connect(port, timeout=5):
    s = socket.socket()
    if IFACE:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, IFACE.encode())
    s.settimeout(timeout)
    s.connect((socket.gethostbyname(HOST), port))
    return s


def _http(method, path, data=None):
    body = urlencode(data).encode() if data else b''
    s = _connect(80)
    s.sendall(f'{method} {path} HTTP/1.0\r\nHost: {HOST}\r\n'
              f'Content-Type: application/x-www-form-urlencoded\r\nContent-Length: {len(body)}\r\n\r\n'.encode() + body)
    resp = b''
    while chunk := s.recv(4096):
        resp += chunk
    s.close()
    return resp.split(b'\r\n\r\n', 1)[1].decode()


def status():
    return json.loads(_http('GET', '/api/status'))


def post(path, **data):
    return _http('POST', path, data)


class LX200:
    """Minimal LX200/OnStep client. Replies follow OnStepX: set commands answer a
    single '1'/'0' (NO_HASH), moves, stops and pulses answer nothing (NO_REPLY)."""
    NO_HASH = (':Sr', ':Sd', ':Sg', ':St', ':MS', ':SL', ':SC', ':SG', ':Sh', ':So', ':ST', ':SX', ':T',
               ':hR', ':$B')
    NO_REPLY = (':Q', ':R', ':U', ':Mg', ':Mn', ':Ms', ':Me', ':Mw', ':WR', ':CS', ':$QZ+', ':$QZ-', ':$QZ/',
                ':$QZZ', ':$QZ!')

    def __init__(self):
        self.s = _connect(LX200_PORT, timeout=2)

    def cmd(self, c):
        self.s.sendall(c.encode('latin1'))
        if c.startswith(self.NO_REPLY):
            time.sleep(0.05)
            return ''
        data = b''
        try:
            while not data.endswith(b'#'):
                data += self.s.recv(256)
                if c.startswith(self.NO_HASH) and data:
                    break
        except socket.timeout:
            pass
        return data.decode('latin1')

    def drain(self):
        """Discard replies left unread (none expected with the OnStep front-end)."""
        self.s.settimeout(0.2)
        try:
            while self.s.recv(256):
                pass
        except socket.timeout:
            pass
        self.s.settimeout(2)


def hms_to_deg(s):
    h, m, x = map(int, s.strip('#').split(':'))
    return (h * 3600 + m * 60 + x) / 240.0


def dms_to_deg(s):
    s = s.strip('#')
    sign = -1 if s[0] == '-' else 1
    d, rest = s.lstrip('+-').split('*')
    m, x = rest.split(':')
    return sign * (int(d) + int(m) / 60 + int(x) / 3600)


def fmt_ra(deg):
    t = round((deg % 360) * 240)
    return f'{t // 3600:02d}:{t % 3600 // 60:02d}:{t % 60:02d}'


def fmt_dec(deg):
    a = round(abs(deg) * 3600)
    return f'{"-" if deg < 0 else "+"}{a // 3600:02d}*{a % 3600 // 60:02d}:{a % 60:02d}'
