import socket, subprocess, time
D = 'LX200 OnStep'
def mark(name):
    s = socket.create_connection(('127.0.0.1', 9998)); s.sendall(f':MARK_{name}#'.encode()); s.recv(16); s.close()
def setp(*props):
    for p in props:
        r = subprocess.run(['indi_setprop', '-p', '7625', f'{D}.{p}'], capture_output=True, text=True)
        if r.returncode: print('setprop failed', p, r.stderr.strip())
def wait(t): time.sleep(t)
steps = [
    ('goto', ['ON_COORD_SET.TRACK=On', 'EQUATORIAL_EOD_COORD.RA=17.5;DEC=10'], 6),
    ('sync', ['ON_COORD_SET.SYNC=On', 'EQUATORIAL_EOD_COORD.RA=17.4;DEC=9.5'], 5),
    ('pulse_N', ['TELESCOPE_TIMED_GUIDE_NS.TIMED_GUIDE_N=500;TIMED_GUIDE_S=0'], 3),
    ('pulse_W', ['TELESCOPE_TIMED_GUIDE_WE.TIMED_GUIDE_W=700;TIMED_GUIDE_E=0'], 3),
    ('slewrate', ['TELESCOPE_SLEW_RATE.4=On'], 3),
    ('move_N_start', ['TELESCOPE_MOTION_NS.MOTION_NORTH=On;MOTION_SOUTH=Off'], 3),
    ('move_N_stop', ['TELESCOPE_MOTION_NS.MOTION_NORTH=Off;MOTION_SOUTH=Off'], 3),
    ('move_E_start', ['TELESCOPE_MOTION_WE.MOTION_EAST=On;MOTION_WEST=Off'], 3),
    ('move_E_stop', ['TELESCOPE_MOTION_WE.MOTION_EAST=Off;MOTION_WEST=Off'], 3),
    ('abort', ['TELESCOPE_ABORT_MOTION.ABORT=On'], 3),
    ('track_off', ['TELESCOPE_TRACK_STATE.TRACK_OFF=On;TRACK_ON=Off'], 3),
    ('track_on', ['TELESCOPE_TRACK_STATE.TRACK_ON=On;TRACK_OFF=Off'], 3),
    ('track_rate', ['TELESCOPE_TRACK_MODE.TRACK_LUNAR=On'], 3),
    ('time', ['TIME_UTC.UTC=2026-09-26T01:00:00;OFFSET=2'], 4),
    ('location', ['GEOGRAPHIC_COORD.LAT=40.8728;LONG=14.4377;ELEV=50'], 5),
    ('park', ['TELESCOPE_PARK.PARK=On;UNPARK=Off'], 4),
    ('unpark', ['TELESCOPE_PARK.UNPARK=On;PARK=Off'], 4),
    ('end', [], 0),
]
for name, props, t in steps:
    mark(name); setp(*props); wait(t)
