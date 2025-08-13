# ========================== CONFIGURATION ============================

SERVER_HOST         = '127.0.0.1'
SERVER_PORT         = 5000

SYNASCAN_HOST       = '127.0.0.1'
SYNASCAN_PORT       = 1880

SERIAL_PORT         = '/dev/ttyUSB1'
SERIAL_BAUDRATE     = 115200
SERIAL_TIMEOUT      = 1

OBSERVER_LON        = 14.4377
OBSERVER_LAT        = 40.8728

RA_HOME_OFFSET      = 2.0  # degrees (hardware mechanical RA home offset)
DEC_STEPS_PER_180_DEG= 585600 / 2
SIDEREAL_SPEED      = 0.004176  # degrees per sec

GOTO_SPEED_MULT     = 128
FINE_SPEED_MULT     = 8
GUIDE_EAST_MULT     = 1.5
GUIDE_WEST_MULT     = 0.5
PIER_EAST_SIDE = False ## Side of the lights on the star adventurer, with camera pointing in the opposite direction of north pole

# ========================== IMPORTS & UTILS ==========================

import socket, time, re, threading
from datetime import datetime
import ephem
import synscan
import serial

def clamp360(degrees):
    return degrees % 360.0

def time_to_degrees(hours, minutes, seconds):
    total_seconds = hours * 3600 + minutes * 60 + seconds
    return (total_seconds % (24*3600)) / (24*3600) * 360.0

def degrees_to_hms(degrees):
    degrees = clamp360(degrees)
    total_hours = degrees / 15.0
    h = int(total_hours)
    m = int((total_hours - h)*60)
    s = int((((total_hours - h)*60) - m)*60)
    return h, m, s

def dec_to_steps(dec_deg):
    dec_in_centi_degrees = abs(dec_deg) * 3600.0
    steps = int(((dec_in_centi_degrees + 3600.0 * 180.0) / (3600.0 * 360.0)) * DEC_STEPS_PER_180_DEG)
    return steps

def steps_to_coord(steps, meridian_flipped):
    norm_dec = (steps / DEC_STEPS_PER_180_DEG) * 360.0 - 180.0
    if meridian_flipped:
        norm_dec = ((180 - norm_dec) + 180) % 360.0 - 180.0
    sign = -1 if norm_dec < 0 else 1
    norm_dec = abs(norm_dec)
    deg = int(norm_dec)
    minutes = int((norm_dec - deg) * 60)
    seconds = round(((norm_dec - deg) * 60 - minutes) * 60)
    return f"{sign*deg:02d}*{minutes:02d}:{seconds:02d}"

# ========================== EVENT SCHEDULER ==========================

class EventScheduler:
    def __init__(self):
        self.events = []
        self.lock = threading.Lock()
    def schedule_event(self, delay_ms: int, callback):
        event = threading.Timer(delay_ms / 1000.0, callback)
        with self.lock:
            self.events.append(event)
        event.start()
    def cancel_all_events(self):
        with self.lock:
            for event in self.events:
                event.cancel()
            self.events.clear()

# ========================== HARDWARE COMM (NO LOGIC!) ===============

class HardwareComm:
    def __init__(self, pier_east_side=False):
        # Motor controller and Arduino connections
        self.smc = None
        self.arduino = None
        self.pier_east_side = pier_east_side
        self.connect()
    def connect(self):
        # Connect to SynScan
        self.smc = synscan.motors(SYNASCAN_HOST, SYNASCAN_PORT)
        self.smc.get_values({}, True)
        self.smc.set_pos(RA_HOME_OFFSET, 0)
        # Connect to Arduino/DEC
        self.arduino = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
        time.sleep(2)
        self.arduino.flushInput()
        self.arduino.flushOutput()
        self.arduino.write(b":Y#")
        self.arduino.readline()
        if self.pier_east_side:
            self.arduino.write(b":CP#")
            self.arduino.readline() 
    # -------- SMC methods --------
    def ra_axis_stop(self, hard=False):  self.smc.axis_stop_motion(1, hard)
    def ra_axis_start(self):            self.smc.axis_start_motion(1)
    def ra_axis_set_speed(self, speed): self.smc.axis_set_speed(1, speed)
    def ra_axis_set_mode(self, enable, cw, goto): self.smc.axis_set_motion_mode(1, enable, cw, goto)
    def ra_axis_get_pos(self):          return self.smc.axis_get_pos(1)
    def ra_axis_set_pos(self, pos):     self.smc.axis_set_pos(1, pos)
    def ra_status_stopped(self):
        self.smc.update_current_values()
        return self.smc.values[1]['Status']['Stopped']
    def update_smc(self):
        self.smc.update_current_values()

    # -------- Arduino methods --------
    def ardu_write(self, cmd: str):
        # Send to Arduino, return string
        self.arduino.write(cmd.encode('utf-8'))
        return self.arduino.readline().decode('utf-8').strip()

# ========================== MOUNT CONTROL (LOGIC+STATE) =============

class MountControl:
    def __init__(self, hardware, lon, lat):
        self.hw = hardware  # HardwareComm instance
        # Observer setup for coordinate transforms
        self.observer = ephem.Observer()
        self.observer.lon = str(lon)
        self.observer.lat = str(lat)

        # --- All "state" here ---
        self.lock = threading.RLock()
        self.ra_current = 0.0
        self.ra_target  = 0.0
        self.dec_target = 0.0
        self.dec_steps  = 0
        self.meridian_flipped = False
        self.ra_slewing       = False
        self.ra_tracking      = True
        self.guide = {'n': False, 's': False, 'e': False, 'w': False}
        self.goto_thread = None

        # Guiding schedulers
        self.sched_n = EventScheduler()
        self.sched_s = EventScheduler()
        self.sched_e = EventScheduler()
        self.sched_w = EventScheduler()

        # Start background RA monitor
        threading.Thread(target=self._update_ra_bg, daemon=True).start()
        self.set_tracking(True)

    # --- Coordinate/time utilities ---
    def _now(self):
        self.observer.date = datetime.now()
    def get_sidereal_time(self):
        self._now()
        return self.observer.sidereal_time() * 180.0 / ephem.pi
    def get_ha(self, ra_deg):
        lst = self.get_sidereal_time()
        return (lst - ra_deg) % 360.0 + RA_HOME_OFFSET
    def get_ra(self, ha_deg):
        lst = self.get_sidereal_time()
        return (lst - (ha_deg - RA_HOME_OFFSET)) % 360.0

    # --- Tracking ---
    def set_tracking(self, enable: bool):
        with self.lock:
            if enable:
                self.hw.ra_axis_stop(True)
                self.hw.ra_axis_set_speed(SIDEREAL_SPEED)
                self.hw.ra_axis_set_mode(True, True, False)
                self.hw.ra_axis_start()
                self.ra_tracking = True
            else:
                self.hw.ra_axis_stop(True)
                self.ra_tracking = False

    # --- Background RA update thread ---
    def _update_ra_bg(self):
        last_ah = -2000
        while True:
            try:
                with self.lock:
                    ah_read = self.hw.ra_axis_get_pos()
                    self.ra_current = self.get_ra(ah_read)
                    # Motor stall check can be done here (optional)
                    last_ah = ah_read
            except Exception as e:
                print(f"[MountControl] BG RA update error: {e}")
            time.sleep(0.2)

    # --- GOTO/Movement ---
    def set_ra_target(self, h, m, s):
        ra = time_to_degrees(h, m, s)
        ha = self.get_ha(ra)
        print(f"Setting RA target {h:02d}:{m:02d}:{s:02d} (deg={ra:.3f}, HA={ha:.3f})")
        if ha > 180:
            self.meridian_flipped = True
            # This is exactly your original logic: target is on the "other" side
            self.ra_target = self.get_ra((ha + 180) % 360)
        else:
            self.meridian_flipped = False
            self.ra_target = ra

    def set_dec_target(self, sign, d, m, s):
        deg = (abs(d) + abs(m)/60.0 + abs(s)/3600.0) * (1 if sign == '+' else -1)
        if self.meridian_flipped:
            deg = 180 - deg
        self.dec_target = deg
        steps = dec_to_steps(deg)
        self.dec_steps = steps
        print(f"Setting DEC target: {deg:.3f} deg => {steps} steps (meridian_flipped={self.meridian_flipped})")
        resp = self.hw.ardu_write(f":Sd{steps}#")
        return resp == "#"

    def goto(self):
        print("[MountControl] Starting GOTO: RA and DEC")
        # Start GOTO to RA in thread; DEC slew immediately after
        if self.goto_thread is None or not self.goto_thread.is_alive():
            self.goto_thread = threading.Thread(target=self.goto_ra)
            self.goto_thread.start()
        dec_status = self.hw.ardu_write(":MS#")
        return dec_status == "#"

    def goto_ra(self):
        # Moves the RA axis from current to target
        print("[MountControl] GOTO RA")
        with self.lock:
            self.ra_tracking = False
            self.ra_slewing = True
            ha_cur = self.get_ha(self.ra_current)
            ha_tgt = self.get_ha(self.ra_target)
            d_ha = ha_cur - ha_tgt
            if d_ha > 180: d_ha -= 360
            if d_ha < -180: d_ha += 360
            direction_cw = (d_ha <= 0)
            self.hw.ra_axis_stop(True)
            while not self.hw.ra_status_stopped():
                time.sleep(0.1)
            speed = GOTO_SPEED_MULT * SIDEREAL_SPEED
            self.hw.ra_axis_set_mode(True, direction_cw, False)
            self.hw.ra_axis_set_speed(speed)
            self.hw.ra_axis_start()
            last_remaining = d_ha
            nearing = False
        # Slewing loop:
        while self.ra_slewing:
            with self.lock:
                ha_cur = self.get_ha(self.ra_current)
                ha_tgt = self.get_ha(self.ra_target)
                remaining = ha_cur - ha_tgt
                if remaining > 180: remaining -= 360
                if remaining < -180: remaining += 360
                if abs(remaining) - abs(last_remaining) <= 0.01:
                    if self.hw.ra_status_stopped():
                        print("Motor stopped unexpectedly, restarting...")
                        self.hw.ra_axis_start()
                if abs(remaining) < 1.0 and not nearing:
                    print("Slowing for final approach ...")
                    self.hw.ra_axis_stop(True)
                    time.sleep(0.05)
                    self.hw.ra_axis_set_speed(FINE_SPEED_MULT * SIDEREAL_SPEED)
                    self.hw.ra_axis_start()
                    nearing = True
                if d_ha * remaining <= 0 and abs(remaining) < 0.1:
                    print(f"GOTO RA done, error {remaining:.3f}")
                    break
                last_remaining = remaining
            time.sleep(0.1)
        with self.lock:
            self.ra_slewing = False
            self.set_tracking(True)

    # --- GUIDING ---
    def guide(self, axis, direction, duration_ms=0):
        # Flip guiding directions if meridian flipped
        if self.meridian_flipped:
            if axis == 'ra':
                direction = 'e' if direction == 'w' else 'w' if direction == 'e' else direction
            elif axis == 'dec':
                direction = 'n' if direction == 's' else 's' if direction == 'n' else direction
                
        if axis == 'ra':
            if direction == 'e':
                self.guide['e'] = True
                self.hw.ra_axis_set_speed(GUIDE_EAST_MULT * SIDEREAL_SPEED)
                if duration_ms > 0:
                    self.sched_e.schedule_event(duration_ms, self.stop_guide_ra)
            elif direction == 'w':
                self.guide['w'] = True
                self.hw.ra_axis_set_speed(GUIDE_WEST_MULT * SIDEREAL_SPEED)
                if duration_ms > 0:
                    self.sched_w.schedule_event(duration_ms, self.stop_guide_ra)
        elif axis == 'dec':
            if direction == 'n':
                self.guide['n'] = True
                self.hw.ardu_write(":Mn#")
                if duration_ms > 0:
                    self.sched_n.schedule_event(duration_ms, lambda: self.stop_guide_dec('n'))
            elif direction == 's':
                self.guide['s'] = True
                self.hw.ardu_write(":Ms#")
                if duration_ms > 0:
                    self.sched_s.schedule_event(duration_ms, lambda: self.stop_guide_dec('s'))

    def stop_guide_ra(self):
        self.guide['e'] = self.guide['w'] = False
        self.sched_e.cancel_all_events()
        self.sched_w.cancel_all_events()
        self.hw.ra_axis_set_speed(SIDEREAL_SPEED)

    def stop_guide_dec(self, which=None):
        if which: self.guide[which] = False
        else: self.guide['n'] = self.guide['s'] = False
        self.sched_n.cancel_all_events()
        self.sched_s.cancel_all_events()
        self.hw.ardu_write(":Q#")

    # --- Status/utility queries ---
    def get_ra_coord(self):
        with self.lock:
            pos = self.ra_current
            if self.meridian_flipped:
                pos = self.get_ra((self.get_ha(pos) - 180) % 360)
            h, m, s = degrees_to_hms(pos)
            return f"{h:02d}:{m:02d}:{s:02d}"

    def get_dec_coord(self) -> str:
        """
        Read the current DEC position from Arduino and return as formatted
        declination string matching LX200 protocol (±DD*MM:SS), precisely
        handling meridian flip as in original code.
        """
        response = self.hw.ardu_write(":Gd#")
        if response:
            try:
                steps = int(response)
                self.dec_current_steps = steps  # cache last valid steps
            except ValueError:
                # fallback in case Arduino returns invalid data
                steps = getattr(self, 'dec_current_steps', 0)
        else:
            steps = getattr(self, 'dec_current_steps', 0)
        coord = self._steps_to_dec_coord(steps)
        return coord

    def _steps_to_dec_coord(self, steps: int) -> str:
        """
        Convert declination steps read from hardware back to ±DD*MM:SS string,
        handling the meridian flip.
        """
        # Convert steps to normalized declination degrees in range [-180, +180]
        normalized_dec = (steps / DEC_STEPS_PER_180_DEG) * 360.0 - 180.0

        # Apply meridian flip if active, same formula as original
        if self.meridian_flipped:
            normalized_dec = ((180 - normalized_dec) + 180) % 360 - 180

        # Determine sign
        sign = '-' if normalized_dec < 0 else '+'

        # Work with absolute value for formatting
        normalized_dec = abs(normalized_dec)

        deg = int(normalized_dec)
        minutes = int((normalized_dec - deg) * 60)
        seconds = round(((normalized_dec - deg) * 60 - minutes) * 60)

        # Handle rounding overflow for seconds/minutes if needed
        if seconds >= 60:
            seconds = 0
            minutes += 1
        if minutes >= 60:
            minutes = 0
            deg += 1

        return f"{sign}{deg:02d}*{minutes:02d}:{seconds:02d}"


    def is_dec_slewing(self):
        resp = self.hw.ardu_write(":D#")
        return resp.strip() == "1"

    def sync(self):
        with self.lock:
                # Stop RA motor first and wait until stopped
                self.hw.ra_axis_stop(True)
                while not self.hw.ra_status_stopped():
                    time.sleep(0.05)

                # Set RA hardware position to target hour angle counts
                ha_target = self.get_ha(self.ra_target)
                # Assuming ra_axis_set_pos accepts counts corresponding to degrees
                self.hw.ra_axis_set_pos(ha_target)

                # Update internal current RA
                self.ra_current = self.ra_target

                # Re-enable tracking
                self.set_tracking(True)

                # Send DEC sync command to Arduino WITHOUT expecting any reply
                self.hw.ardu_write(":CM#")

                # Assume sync succeeded; do NOT check response because none is expected
                return True

    def stop_all(self):
        with self.lock:
            self.ra_slewing = False
            self.stop_guide_ra()
            self.stop_guide_dec()
            if not any(self.guide.values()):
                self.set_tracking(True)

    def slewing_any(self):
        return self.ra_slewing or self.is_dec_slewing() or any(self.guide.values())

# ========================== LX200 PROXY SERVER =======================

class LX200Proxy:
    def __init__(self, mount: MountControl):
        self.mount = mount
        self.host = SERVER_HOST
        self.port = SERVER_PORT

    def start(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        sock.listen(1)
        print(f"LX200 Proxy listening on {self.host}:{self.port}")
        try:
            while True:
                client, addr = sock.accept()
                print(f"Connection from {addr}")
                threading.Thread(target=self.handle_client, args=(client,), daemon=True).start()
        except KeyboardInterrupt:
            print("Shutdown.")
        finally:
            sock.close()

    def handle_client(self, client_sock):
        with client_sock:
            try:
                while True:
                    data = client_sock.recv(1024).decode('utf-8').strip()
                    if not data: break
                    print(f"<- Received: {data}")
                    resp = self.process_command(data)
                    if resp:
                        print(f"-> Sending: {resp}")
                        client_sock.sendall(resp.encode('utf-8'))
            except Exception as e:
                print("Client error:", e)

    def process_command(self, cmd):
        # RA/DEC Get
        if cmd == ':GR#':
            return self.mount.get_ra_coord() + "#"
        if cmd == ':GD#':
            return self.mount.get_dec_coord() + "#"
        if cmd == ':D#':
            return chr(127)+"#" if self.mount.slewing_any() else "#"
        # Set target RA
        if cmd.startswith(':Sr'):
            m = re.match(r':Sr(\d{2}):(\d{2}):(\d{2})#', cmd)
            if m:
                h, m_, s = map(int, m.groups())
                self.mount.set_ra_target(h, m_, s)
                return "1"
            return "0"
        # Set target DEC
        if cmd.startswith(':Sd'):
            m = re.match(r':Sd([+-])(\d{2})\*(\d{2}):(\d{2})#', cmd)
            if m:
                sign, d, mm, ss = m.groups()
                d, mm, ss = map(int, (d, mm, ss))
                ok = self.mount.set_dec_target(sign, d, mm, ss)
                return "1" if ok else "0"
            return "0"
        # Slew (goto)
        if cmd == ':MS#':
            ok = self.mount.goto()
            return "0" if ok else "1"
        # Manual guiding
        if cmd.startswith(':M'):
            dir_match = re.match(r':M([gnsew])#', cmd)
            if dir_match:
                direction = dir_match.group(1)
                if direction in "ns":
                    self.mount.guide('dec', direction)
                if direction in "ew":
                    self.mount.guide('ra', direction)
                return ""
            timed_match = re.match(r':Mg([nsew])(\d+)#', cmd)
            if timed_match:
                d, time_ms = timed_match.groups()
                time_ms = int(time_ms)
                if d in "ns":
                    self.mount.guide('dec', d, time_ms)
                if d in "ew":
                    self.mount.guide('ra', d, time_ms)
                return ""
            return "0"
        # Stop all
        if cmd.startswith(':Q'):
            self.mount.stop_all()
            return ""
        # Sync
        if cmd == ':CM#':
            ok = self.mount.sync()
            return ":Coordinates matched #" if ok else "Sync failed.#"
        # Static/Boilerplate
        if cmd == ':GVP#': return "Proxino#"
        if cmd == ':GVN#': return "1.0#"
        if cmd == ':GVR#': return "001012023#"
        if cmd == ':GVD#': return "01.1#"
        if cmd == ':GVF#': return "11#"
        if cmd == ':GM#':  return "Site 1#"
        if cmd == ':GT#':  return "60.0#"
        if cmd == ':Gc#':  return "24#"
        if cmd == ':GVT#' or cmd == ':GL#': return datetime.now().strftime("%H:%M:%S#")
        if cmd == ':GC#':  return datetime.now().strftime("%m/%d/%y#")
        if cmd == ':GG#':  return f"{-time.timezone / 3600.0:+.1f}#"
        # Lat/Lon get/set
        if cmd == ':Gg#':
            lon = float(self.mount.observer.lon)
            return f"{-lon:+04.0f}*00#"
        if cmd == ':Gt#':
            lat = float(self.mount.observer.lat)
            d = int(lat)
            m = abs((lat-d) * 60)
            return f"{d:+03d}*{int(m):02d}#"
        if cmd.startswith(':Sg'):
            m = re.match(r":Sg([+-]?\d{2,3})\*(\d{2})#", cmd)
            if m:
                deg, mins = map(int, m.groups())
                self.mount.observer.lon = str(-(deg + mins/60.0))
                return "1"
            return "0"
        if cmd.startswith(':St'):
            m = re.match(r":St([+-]?\d{2})\*(\d{2})#", cmd)
            if m:
                deg, mins = map(int, m.groups())
                self.mount.observer.lat = str(deg + mins/60.0)
                return "1"
            return "0"
        # Acknowledge to prevent errors
        if cmd.startswith((':SC', ':SL', ':So', ':Sh', ':SG')):
            return "1"
        if cmd.startswith(':R'):
            return ""
        print("Unknown command:", cmd)
        return "0"

# ========================== ENTRYPOINT ===============================

if __name__ == "__main__":
    print("Initializing hardware ...")
    hw = HardwareComm(pier_east_side=PIER_EAST_SIDE)
    mount = MountControl(hw, OBSERVER_LON, OBSERVER_LAT)
    print("Starting LX200 Proxy ...")
    proxy = LX200Proxy(mount)
    proxy.start()

