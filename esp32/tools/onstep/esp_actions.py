"""Drive the INDI OnStep driver against the real ESP with small, safe moves."""
import subprocess, time, datetime, sys
D = 'LX200 OnStep'
P = sys.argv[1] if len(sys.argv) > 1 else '7625'
def setp(p):
    r = subprocess.run(['indi_setprop', '-p', P, f'{D}.{p}'], capture_output=True, text=True)
    if r.returncode: print('setprop failed', p, r.stderr.strip())
def getp(p):
    r = subprocess.run(['indi_getprop', '-1', '-p', P, f'{D}.{p}'], capture_output=True, text=True)
    return r.stdout.strip()
def show(label, *props):
    print(f'{label:>12}:', '  '.join(f'{p.split(".")[-1]}={getp(p)}' for p in props), flush=True)
ra, dec = float(getp('EQUATORIAL_EOD_COORD.RA')), float(getp('EQUATORIAL_EOD_COORD.DEC'))
show('start', 'EQUATORIAL_EOD_COORD.RA', 'EQUATORIAL_EOD_COORD.DEC', 'TELESCOPE_PIER_SIDE.PIER_EAST', 'TELESCOPE_TRACK_STATE.TRACK_ON')
steps = [
    ('sync here', [f'ON_COORD_SET.SYNC=On', f'EQUATORIAL_EOD_COORD.RA={ra};DEC={dec}'], 4),
    ('goto +0.5', [f'ON_COORD_SET.TRACK=On', f'EQUATORIAL_EOD_COORD.RA={ra - 0.5/15};DEC={dec + 0.5}'], 12),
    ('pulse N', ['TELESCOPE_TIMED_GUIDE_NS.TIMED_GUIDE_N=1000;TIMED_GUIDE_S=0'], 3),
    ('pulse W', ['TELESCOPE_TIMED_GUIDE_WE.TIMED_GUIDE_W=1000;TIMED_GUIDE_E=0'], 3),
    ('move N', ['TELESCOPE_MOTION_NS.MOTION_NORTH=On;MOTION_SOUTH=Off'], 2),
    ('stop N', ['TELESCOPE_MOTION_NS.MOTION_NORTH=Off;MOTION_SOUTH=Off'], 3),
    ('track off', ['TELESCOPE_TRACK_STATE.TRACK_OFF=On;TRACK_ON=Off'], 4),
    ('track on', ['TELESCOPE_TRACK_STATE.TRACK_ON=On;TRACK_OFF=Off'], 4),
    ('time', [f'TIME_UTC.UTC={datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S")};OFFSET=2'], 4),
    ('location', ['GEOGRAPHIC_COORD.LAT=40.8728;LONG=14.4377;ELEV=50'], 5),
    ('goto back', [f'ON_COORD_SET.TRACK=On', f'EQUATORIAL_EOD_COORD.RA={ra};DEC={dec}'], 12),
]
for name, props, t in steps:
    for p in props: setp(p)
    time.sleep(t)
    show(name, 'EQUATORIAL_EOD_COORD.RA', 'EQUATORIAL_EOD_COORD.DEC', 'TELESCOPE_PIER_SIDE.PIER_EAST', 'TELESCOPE_TRACK_STATE.TRACK_ON')
