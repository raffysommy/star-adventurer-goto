"""
LX200 Proxy for a custom telescope mount - FIXED VERSION

This script acts as a TCP server that emulates the Meade LX200 protocol.
It translates LX200 commands into instructions for a hybrid telescope mount:
- Right Ascension (RA) is controlled by a SynScan-based motor (e.g., Sky-Watcher Star Adventurer).
- Declination (DEC) is controlled by a custom Arduino-based stepper motor system.

It uses threading to handle mount position updates and slewing operations concurrently.
The astronomical calculations are performed using the 'ephem' library.

FIXED: Restored proper DEC sync functionality that was broken in the refactor.
"""

import socket
import re
import time
import threading
from datetime import datetime, UTC
from typing import Callable

import ephem
import serial
import synscan

# --- Constants and Configuration ---

# -- Network Configuration
SERVER_HOST = '127.0.0.1'
SERVER_PORT = 5000
SYNASCAN_HOST = '127.0.0.1'
SYNASCAN_PORT = 1880

# -- Serial Port Configuration
# Replace with your Arduino's serial port
SERIAL_PORT = '/dev/ttyUSB1'
SERIAL_BAUDRATE = 115200

# -- Mount Configuration
# Longitude and Latitude of the observer's location.
# TODO: Update with your specific coordinates.
OBSERVER_LON = '14.4377'  # Degrees East
OBSERVER_LAT = '40.8728'  # Degrees North

# This compensates for the fact that the Star Adventurer's home position might not
# be exactly at 0 Hour Angle. This is the physical offset in degrees.
RA_HOME_OFFSET = 2.0

# Total steps for a full 360-degree revolution of the DEC axis.
# This value is specific to your Arduino motor setup. (e.g., 585600 for a full circle)
# Here, we use steps for a 180-degree range.
DEC_STEPS_PER_180_DEG = 585600 / 2

# The speed of the RA motor required to track celestial objects.
# This value is in degrees per second, adjusted for atmospheric refraction.
SIDEREAL_SPEED_DEGREES_PER_SEC = 0.004176

# Speed multipliers for slewing operations
GOTO_SPEED_MULTIPLIER = 128  # Speed for long-distance slews (Goto)
FINE_TUNE_SPEED_MULTIPLIER = 8    # Speed for final approach to target
GUIDE_SPEED_MULTIPLIER_EAST = 1.5 # Guiding speed when moving East
GUIDE_SPEED_MULTIPLIER_WEST = 0.5 # Guiding speed when moving West

# --- Event Scheduler Class ---

class EventScheduler:
    """Simple event scheduler for timed operations."""
    def __init__(self):
        self.events = []
        self.lock = threading.Lock()

    def schedule_event(self, delay_ms: int, callback: Callable):
        """Schedule an event to run in the future."""
        event = threading.Timer(delay_ms / 1000.0, callback)
        with self.lock:
            self.events.append(event)
        event.start()

    def cancel_all_events(self):
        """Cancel all scheduled events."""
        with self.lock:
            for event in self.events:
                event.cancel()
            self.events.clear()

# --- Telescope Controller Class ---

class TelescopeController:
    """
    Manages the state and hardware control for the telescope mount.

    This class encapsulates all interactions with the SynScan motor (RA),
    the Arduino (DEC), and manages the mount's current state, including
    position, slewing status, and meridian flips.
    """

    def __init__(self):
        """Initializes the telescope controller and connects to hardware."""
        # -- State Variables
        self.ra_current_deg = 0.0
        self.ra_target_deg = 0.0
        self.dec_current_steps = 0
        self.dec_target_deg = 0.0  # FIXED: Added missing attribute

        self.is_ra_slewing = False
        self.is_ra_tracking = True
        self.is_guiding = {'n': False, 's': False, 'e': False, 'w': False}

        # A meridian flip occurs when the telescope passes the meridian
        # and needs to rotate to the other side to continue tracking.
        self.is_meridian_flipped = False

        # Pier side indicates which side of the pier the telescope is on.
        # False = West (default), True = East.
        self.is_pier_east_side = False

        # A lock to ensure thread-safe access to mount state and hardware.
        self.lock = threading.RLock()

        # -- Event schedulers for timed guiding
        self.scheduler_north = EventScheduler()
        self.scheduler_south = EventScheduler()
        self.scheduler_east = EventScheduler()
        self.scheduler_west = EventScheduler()

        # -- Hardware and Ephemeris Setup
        self.smc = None
        self.arduino_dec = None
        self.observer = self._setup_observer()

        # -- Connect to Hardware
        self._connect_hardware()

        # Start the background thread that continuously updates the current RA.
        self.update_thread = threading.Thread(target=self._update_ra_continuously, daemon=True)
        self.update_thread.start()

    def _setup_observer(self) -> ephem.Observer:
        """Configures the ephem observer with location coordinates."""
        obs = ephem.Observer()
        obs.lon = OBSERVER_LON
        obs.lat = OBSERVER_LAT
        return obs

    def _connect_hardware(self):
        """Initializes connections to SynScan and Arduino."""
        try:
            # Connect to SynScan motor controller for RA
            self.smc = synscan.motors(SYNASCAN_HOST, SYNASCAN_PORT)
            self.smc.get_values({}, True)
            self.smc.set_pos(RA_HOME_OFFSET, 0) # Set initial position with offset
            print("Successfully connected to SynScan controller.")

            # Connect to Arduino for DEC
            self.arduino_dec = serial.Serial(SERIAL_PORT, baudrate=SERIAL_BAUDRATE, timeout=1)
            time.sleep(2)  # Wait for serial connection to establish
            self.arduino_dec.flushInput()
            self.arduino_dec.flushOutput()
            
            # Initialize Arduino
            self._send_arduino_command(":Y#")  # Initialize
            if self.is_pier_east_side:
                self._send_arduino_command(":CP#")  # Set pier side
            
            print(f"Successfully connected to Arduino on {SERIAL_PORT}.")

            # Initialize tracking
            self.set_tracking(True)

        except serial.SerialException as e:
            print(f"Error: Could not connect to Arduino on {SERIAL_PORT}. {e}")
            self.arduino_dec = None
        except Exception as e:
            print(f"Error: Could not connect to SynScan controller. {e}")
            self.smc = None

    # --- Time and Coordinate Calculations ---

    def _update_observer_time(self):
        """Updates the observer's date and time to now."""
        self.observer.date = datetime.now(UTC)

    def get_sidereal_time_deg(self) -> float:
        """Calculates the current Local Sidereal Time (LST) in degrees."""
        self._update_observer_time()
        # ephem.sidereal_time() returns radians, so we convert to degrees.
        return self.observer.sidereal_time() * 180.0 / ephem.pi

    def get_hour_angle_deg(self, ra_deg: float) -> float:
        """
        Calculates the Hour Angle (HA) for a given Right Ascension (RA).
        HA = LST - RA
        """
        lst_deg = self.get_sidereal_time_deg()
        # The result is normalized to the range [0, 360) and the home offset is added.
        return (lst_deg - ra_deg) % 360.0 + RA_HOME_OFFSET

    def get_right_ascension_deg(self, ha_deg: float) -> float:
        """
        Calculates the Right Ascension (RA) for a given Hour Angle (HA).
        RA = LST - HA
        """
        lst_deg = self.get_sidereal_time_deg()
        ha_deg_corrected = ha_deg - RA_HOME_OFFSET
        return (lst_deg - ha_deg_corrected) % 360.0

    # --- RA Motor Control ---

    def set_tracking(self, enabled: bool):
        """Starts or stops sidereal tracking on the RA axis."""
        with self.lock:
            if not self.smc:
                return
                
            if enabled:
                self.smc.axis_stop_motion(1, True)
                self.smc.axis_set_speed(1, SIDEREAL_SPEED_DEGREES_PER_SEC)
                # Mode: High speed, Forward direction (can be changed later), Not Goto
                self.smc.axis_set_motion_mode(1, True, True, False)
                self.smc.axis_start_motion(1)
                self.is_ra_tracking = True
                print("RA tracking started")
            else:
                self.smc.axis_stop_motion(1, True)
                self.is_ra_tracking = False
                print("RA tracking stopped")

    def _update_ra_continuously(self):
        """
        A background task running in a thread to poll the RA motor's position
        and update the `ra_current_deg` state.
        """
        ha_previous_deg = -2000  # Initialize with an impossible value

        while True:
            if not self.smc:
                time.sleep(1)
                continue

            try:
                with self.lock:
                    ha_read_deg = self.smc.axis_get_pos(1)
                    self.ra_current_deg = self.get_right_ascension_deg(ha_read_deg)

                    # --- Sanity Check: Detect if the motor has stalled ---
                    # If tracking or guiding, the HA should always be increasing.
                    # If it's the same or less, the motor might be stuck.
                    is_stalled = (ha_read_deg <= ha_previous_deg)
                    is_moving = self.is_ra_tracking or self.is_guiding['e'] or self.is_guiding['w']

                    if is_stalled and is_moving:
                        print("Warning: RA motor may be stalled. Resetting to tracking speed.")
                        self.set_tracking(True) # Attempt to recover

                    ha_previous_deg = ha_read_deg

            except Exception as e:
                print(f"Error updating RA position: {e}")

            time.sleep(0.2) # Poll every 200ms

    def goto_ra(self):
        """
        Slews the RA axis to the `ra_target_deg`. This function is blocking
        and intended to be run in its own thread.
        """
        if not self.smc:
            print("Cannot slew RA: SynScan not connected")
            return

        with self.lock:
            self.is_ra_tracking = False
            self.is_ra_slewing = True
            
            ha_current = self.get_hour_angle_deg(self.ra_current_deg)
            ha_target = self.get_hour_angle_deg(self.ra_target_deg)
            
            delta_ha = ha_current - ha_target
            
            # Determine shortest direction. Normalize to [-180, 180]
            if delta_ha > 180:
                delta_ha -= 360
            if delta_ha < -180:
                delta_ha += 360

            # True for Clockwise (West), False for Counter-Clockwise (East)
            direction_is_cw = (delta_ha <= 0)
            
            print(f"Starting RA slew: Current HA={ha_current:.2f}°, Target HA={ha_target:.2f}°, Delta={delta_ha:.2f}°")
            
            self.smc.axis_stop_motion_hard(1, True) # Immediate stop
            
            # Wait for stop
            while not self.smc.get_status(1)['Stopped']:
                time.sleep(0.1)
            
            # Set slew speed and direction
            slew_speed = GOTO_SPEED_MULTIPLIER * SIDEREAL_SPEED_DEGREES_PER_SEC
            self.smc.axis_set_motion_mode(1, True, direction_is_cw, False)
            self.smc.axis_set_speed(1, slew_speed)
            self.smc.axis_start_motion(1)

        # --- Slewing Loop ---
        approaching_target = False
        last_remaining = delta_ha  # Keep track of movement
        
        while self.is_ra_slewing:
            ha_current = self.get_hour_angle_deg(self.ra_current_deg)
            remaining_dist = ha_current - ha_target
            
            # Normalize remaining distance for checking sign change
            if remaining_dist > 180: 
                remaining_dist -= 360
            if remaining_dist < -180: 
                remaining_dist += 360

            # Check if motor is stuck (distance not changing)
            if abs(abs(remaining_dist) - abs(last_remaining)) < 0.01:
                with self.lock:
                    if not self.smc.get_status(1)['Running']:
                        print("Motor stopped unexpectedly, restarting...")
                        self.smc.axis_start_motion(1)

            # Slow down when we get close to the target
            if abs(remaining_dist) < 1.0 and not approaching_target:
                with self.lock:
                    print("Slowing down for final approach...")
                    self.smc.axis_stop_motion(1, True)
                    fine_speed = FINE_TUNE_SPEED_MULTIPLIER * SIDEREAL_SPEED_DEGREES_PER_SEC
                    self.smc.axis_set_speed(1, fine_speed)
                    self.smc.axis_start_motion(1)
                    approaching_target = True

            # Check for overshoot. `delta_ha` is the initial distance vector.
            # If the sign of the remaining distance is opposite to the initial one, we've passed the target.
            if delta_ha * remaining_dist <= 0 and abs(remaining_dist) < 0.1:
                print(f"Target reached. Final error: {remaining_dist:.3f}°")
                break
            
            last_remaining = remaining_dist
            time.sleep(0.1)

        # --- Finalize Slew ---
        with self.lock:
            self.is_ra_slewing = False
            self.set_tracking(True) # Resume tracking

    def guide_ra(self, direction: str, duration_ms: int = 0):
        """Handles guiding pulses for the RA axis."""
        if not self.smc:
            return
            
        with self.lock:
            if direction == 'e':
                self.is_guiding['e'] = True
                speed = GUIDE_SPEED_MULTIPLIER_EAST * SIDEREAL_SPEED_DEGREES_PER_SEC
                print(f"Guiding East at {speed:.6f}°/s")
            elif direction == 'w':
                self.is_guiding['w'] = True
                speed = GUIDE_SPEED_MULTIPLIER_WEST * SIDEREAL_SPEED_DEGREES_PER_SEC
                print(f"Guiding West at {speed:.6f}°/s")
            else:
                return # Invalid direction

            self.smc.axis_set_speed(1, speed)

        # Schedule the stop command after the duration
        if duration_ms > 0:
            if direction == 'e':
                self.scheduler_east.schedule_event(duration_ms, self.stop_guide_ra)
            else:
                self.scheduler_west.schedule_event(duration_ms, self.stop_guide_ra)
    
    def stop_guide_ra(self):
        """Stops RA guiding and returns to sidereal tracking speed."""
        with self.lock:
            if self.is_guiding['e'] or self.is_guiding['w']:
                self.is_guiding['e'] = False
                self.is_guiding['w'] = False
                # Cancel any pending stop events
                self.scheduler_east.cancel_all_events()
                self.scheduler_west.cancel_all_events()
                # Resume normal tracking speed
                if self.smc:
                    self.smc.axis_set_speed(1, SIDEREAL_SPEED_DEGREES_PER_SEC)
                print("RA guiding stopped")

    # --- DEC Motor Control (FIXED) ---

    def _send_arduino_command(self, cmd: str) -> str:
        """Sends a command to the Arduino and returns the response."""
        if not self.arduino_dec:
            print("Warning: Arduino is not connected.")
            return ""
        try:
            print(f"Arduino <- {cmd}")  # Debug output
            self.arduino_dec.write(cmd.encode('utf-8'))
            response = self.arduino_dec.readline().decode('utf-8').strip()
            print(f"Arduino -> {response}")  # Debug output
            return response
        except serial.SerialException as e:
            print(f"Error communicating with Arduino: {e}")
            return ""

    def get_dec_coord(self) -> str:
        """Reads the current DEC position from Arduino and formats it."""
        response = self._send_arduino_command(":Gd#")
        if response and response.replace('-', '').isdigit():  # Handle negative numbers
            steps = int(response)
            self.dec_current_steps = steps  # Update cached value
            return self._steps_to_dec_coord(steps)
        return "+00*00:00"  # Default/error value

    def set_dec_target(self, dec_deg: float) -> bool:
        """
        Sets the DEC target position and immediately sends it to Arduino.
        This is what the original code did in the set_dec method.
        """
        self.dec_target_deg = dec_deg
        
        # Apply meridian flip transformation if needed
        working_dec = dec_deg
        if self.is_meridian_flipped:
            working_dec = 180 - dec_deg
        
        steps = self._dec_deg_to_steps(working_dec)
        print(f"Setting DEC target: {dec_deg:.3f}° -> {steps} steps")
        response = self._send_arduino_command(f":Sd{steps}#")
        
        # Validate response (like original code)
        return response == "#"

    def goto_dec(self) -> bool:
        """
        Commands the Arduino to slew to the stored DEC target.
        Assumes set_dec_target was called first.
        """
        print("Starting DEC slew")
        response = self._send_arduino_command(":MS#")
        return response == "#"

    def guide_dec(self, direction: str, duration_ms: int = 0):
        """Handles guiding pulses for the DEC axis via Arduino."""
        with self.lock:
            if direction == 'n':
                self.is_guiding['n'] = True
                cmd = ":Mn#"
                print("Guiding North")
            elif direction == 's':
                self.is_guiding['s'] = True
                cmd = ":Ms#"
                print("Guiding South")
            else:
                return
            
            self._send_arduino_command(cmd)

        # Schedule automatic stop if duration specified
        if duration_ms > 0:
            if direction == 'n':
                self.scheduler_north.schedule_event(duration_ms, lambda: self.stop_guide_dec('n'))
            else:
                self.scheduler_south.schedule_event(duration_ms, lambda: self.stop_guide_dec('s'))

    def stop_guide_dec(self, direction: str = None):
        """Stops any active DEC guiding."""
        with self.lock:
            if direction:
                self.is_guiding[direction] = False
            else:
                # Stop all DEC guiding
                self.is_guiding['n'] = False
                self.is_guiding['s'] = False
                self.scheduler_north.cancel_all_events()
                self.scheduler_south.cancel_all_events()
        
        self._send_arduino_command(":Q#")
        print("DEC guiding stopped")

    def is_dec_slewing(self) -> bool:
        """Checks if the DEC axis is currently slewing."""
        response = self._send_arduino_command(":D#")
        return response == "1"

    def sync_dec(self) -> bool:
        """
        Synchronizes the Arduino's current position to the target.
        This replicates the original sync behavior.
        """
        print("Syncing DEC")
        # The target should already be set via set_dec_target
        # Just send the sync command
        response = self._send_arduino_command(":CM#")
        return response == "#"

    def stop_all_motion(self):
        """Stops all motion on both axes."""
        print("Emergency stop - all motion")
        with self.lock:
            # Stop RA
            if self.is_ra_slewing:
                self.is_ra_slewing = False
            
            self.stop_guide_ra()
            
            # Stop DEC  
            self.stop_guide_dec()
            
            # Resume tracking if not guiding
            if not any(self.is_guiding.values()):
                self.set_tracking(True)

    # --- Coordinate Conversion Utilities (FIXED) ---

    @staticmethod
    def _hms_to_degrees(h: int, m: int, s: int) -> float:
        """Converts HH:MM:SS to decimal degrees."""
        total_hours = h + m / 60.0 + s / 3600.0
        return (total_hours / 24.0) * 360.0

    @staticmethod
    def _degrees_to_hms_str(deg: float) -> str:
        """Converts decimal degrees to an HH:MM:SS formatted string."""
        deg = deg % 360.0
        total_hours = deg / 15.0
        h = int(total_hours)
        rem_min = (total_hours - h) * 60
        m = int(rem_min)
        s = int((rem_min - m) * 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    def _dec_deg_to_steps(self, dec_deg: float) -> int:
        """
        Converts a declination in degrees to motor steps.
        This matches the original dec_to_steps function.
        """
        # Convert to centiarcseconds (original used this unit)
        dec_centi_arcsec = abs(dec_deg) * 3600.0
        
        # Map to step range (original algorithm)
        steps = int(((dec_centi_arcsec + 3600.0 * 180.0) / (3600.0 * 360.0)) * DEC_STEPS_PER_180_DEG)
        
        return steps

    def _steps_to_dec_coord(self, steps: int) -> str:
        """
        Converts motor steps back to a sDD*MM:SS formatted string.
        This matches the original steps_to_coord function.
        """
        # Reverse the steps to degrees (original algorithm)
        normalized_dec = (steps / DEC_STEPS_PER_180_DEG) * 360.0 - 180.0
        
        # Apply meridian flip correction if needed
        if self.is_meridian_flipped:
            normalized_dec = ((180 - normalized_dec) + 180) % 360 - 180
        
        # Determine sign and make positive for formatting
        sign = '-' if normalized_dec < 0 else '+'
        normalized_dec = abs(normalized_dec)
        
        # Convert to DMS format
        deg = int(normalized_dec)
        minutes = int((normalized_dec - deg) * 60)
        seconds = round(((normalized_dec - deg) * 60 - minutes) * 60)
        
        # Handle seconds overflow
        if seconds >= 60:
            seconds = 0
            minutes += 1
        if minutes >= 60:
            minutes = 0
            deg += 1
        
        return f"{sign}{deg:02d}*{minutes:02d}:{seconds:02d}"


# --- LX200 Protocol Proxy Class ---

class LX200Proxy:
    """
    A TCP server that emulates the LX200 protocol and translates commands
    for the TelescopeController.
    """

    def __init__(self, host: str, port: int, controller: TelescopeController):
        """Initializes the proxy server."""
        self.host = host
        self.port = port
        self.controller = controller
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.client_socket = None

    def start(self):
        """Binds the server socket and listens for client connections."""
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        print(f"LX200 Proxy listening on {self.host}:{self.port}")

        try:
            while True:
                self.client_socket, client_address = self.sock.accept()
                print(f"Connection from {client_address}")
                self._handle_client()
        except KeyboardInterrupt:
            print("\nShutting down server.")
        finally:
            self.sock.close()

    def _handle_client(self):
        """Handles communication with a connected client in a loop."""
        try:
            while True:
                data = self.client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break # Client disconnected
                
                print(f"<- Received: {data}")
                response = self._process_command(data)
                if response is not None:
                    print(f"-> Sending: {response}")
                    self.client_socket.sendall(response.encode('utf-8'))
        except ConnectionResetError:
            print("Client disconnected abruptly.")
        except Exception as e:
            print(f"Error handling client: {e}")
        finally:
            print("Closing client connection.")
            if self.client_socket:
                self.client_socket.close()

    def _deg_to_degmin(self, dec_deg):
        """Converts decimal degrees to a tuple of (degrees, minutes)."""
        d = int(dec_deg)
        m = abs((dec_deg - d) * 60)
        return d, m
        
    def _process_command(self, command: str) -> str:
        """
        Parses an LX200 command and calls the appropriate controller method.
        CORRECTED version with proper DEC handling.
        """
        # --- Get Commands ---
        if command == ':GR#':  # Get RA
            with self.controller.lock:
                ra_deg = self.controller.ra_current_deg
                if self.controller.is_meridian_flipped:
                    ha_deg = self.controller.get_hour_angle_deg(ra_deg)
                    ra_deg = self.controller.get_right_ascension_deg((ha_deg - 180) % 360)
            return f"{self.controller._degrees_to_hms_str(ra_deg)}#"

        if command == ':GD#':  # Get DEC
            return f"{self.controller.get_dec_coord()}#"

        if command == ':D#':  # Get slewing status
            with self.controller.lock:
                ra_slewing = self.controller.is_ra_slewing
                ra_guiding = any([self.controller.is_guiding['e'], self.controller.is_guiding['w']])
                dec_guiding = any([self.controller.is_guiding['n'], self.controller.is_guiding['s']])
            
            dec_slewing = self.controller.is_dec_slewing()
            
            if ra_slewing or dec_slewing or ra_guiding or dec_guiding:
                return chr(127) + "#"  # Slewing (matches original)
            else:
                return "#"  # Not slewing

        # --- Set Commands (CORRECTED) ---
        if command.startswith(':Sr'):  # Set target RA
            match = re.match(r':Sr(\d{2}):(\d{2}):(\d{2})#', command)
            if match:
                h, m, s = map(int, match.groups())
                target_ra = self.controller._hms_to_degrees(h, m, s)
                
                with self.controller.lock:
                    ha_target = self.controller.get_hour_angle_deg(target_ra)
                    print(f"Setting RA target: {h:02d}:{m:02d}:{s:02d} = {target_ra:.3f}° (HA={ha_target:.3f}°)")
                    
                    if ha_target > 180:
                        print("Setting meridian flip for RA target")
                        self.controller.is_meridian_flipped = True
                        flipped_ha = (ha_target - 180) % 360
                        self.controller.ra_target_deg = self.controller.get_right_ascension_deg(flipped_ha)
                    else:
                        self.controller.is_meridian_flipped = False
                        self.controller.ra_target_deg = target_ra
                
                return "1"
            return "0"

        if command.startswith(':Sd'):  # Set target DEC (FIXED)
            match = re.match(r':Sd([+-])(\d{2})\*(\d{2}):(\d{2})#', command)
            if match:
                sign_str, d_str, m_str, s_str = match.groups()
                sign = -1 if sign_str == '-' else 1
                deg = int(d_str) + int(m_str)/60 + int(s_str)/3600
                dec_deg = deg * sign
                
                print(f"Setting DEC target: {sign_str}{d_str}*{m_str}:{s_str} = {dec_deg:.3f}°")
                
                # FIXED: Actually send the target to Arduino (like original)
                success = self.controller.set_dec_target(dec_deg)
                return "1" if success else "0"
            return "0"

        # --- Action Commands (CORRECTED) ---
        if command == ':MS#':  # Slew to target coordinates
            print("Slew command received. Starting GOTO operation.")
            
            # Start RA slew in background thread
            if hasattr(self.controller, 'ra_target_deg'):
                threading.Thread(target=self.controller.goto_ra, daemon=True).start()
            
            # Start DEC slew (target should already be set by :Sd command)
            dec_success = self.controller.goto_dec()
            
            return "0" if dec_success else "1"  # 0 = success in LX200

        if command == ':CM#':  # Sync to target coordinates (FIXED)
            print("Sync command received.")
            
            # Sync RA
            with self.controller.lock:
                if hasattr(self.controller, 'ra_target_deg'):
                    self.controller.smc.axis_stop_motion(1, True)
                    target_ha = self.controller.get_hour_angle_deg(self.controller.ra_target_deg)
                    self.controller.smc.axis_set_pos(1, target_ha)
                    self.controller.ra_current_deg = self.controller.ra_target_deg
                    self.controller.set_tracking(True)
            
            # Sync DEC (target should already be set by :Sd command)
            dec_success = self.controller.sync_dec()
            
            return "Coordinates matched.        #" if dec_success else "Sync failed.#"

        if command.startswith(':M'):  # Manual move (guiding) (IMPROVED)
            direction = None
            duration_ms = 0
            
            # Check for timed guiding commands like ':Mgw150#' (PHD2 style)
            timed_match = re.match(r':Mg([nsew])(\d+)#', command)
            if timed_match:
                direction, duration_str = timed_match.groups()
                duration_ms = int(duration_str)
            else:
                # Simple move command like ':Mn#'
                simple_match = re.match(r':M([nsew])#', command)
                if simple_match:
                    direction = simple_match.group(1)
            
            if direction:
                if direction in ['n', 's']:
                    self.controller.guide_dec(direction, duration_ms)
                elif direction in ['e', 'w']:
                    self.controller.guide_ra(direction, duration_ms)
                return ""  # No response for move commands
            
            return "0"  # Invalid direction

        if command.startswith(':Q'):  # Quit/Stop commands (IMPROVED)
            print("Stop command received.")
            self.controller.stop_all_motion()
            return ""  # No response

        # --- Static/Boilerplate Responses ---
        if command == ':GVP#': return "Proxino#"
        if command == ':GVN#': return "1.0#"
        if command == ':GVR#': return "001012023#" # RA Motor Version
        if command == ':GVD#': return "01.1#"      # DEC Motor Version
        if command == ':GVF#': return "11#"        # Firmware Info
        if command == ':GM#': return "Site 1#"    # Site Name
        if command == ':GT#': return "60.0#"      # Tracking Rate
        if command == ':Gc#': return "24#"        # Calendar Format (24-hour)

        # --- Time and Date Commands ---
        if command == ':GVT#': # Get Telescope Time
            return datetime.now().strftime("%H:%M:%S#")
        if command == ':GL#': # Get Local Time (same as GVT for this implementation)
            return datetime.now().strftime("%H:%M:%S#")
        if command == ':GC#': # Get Calendar Date
            return datetime.now().strftime("%m/%d/%y#")
        if command == ':GG#': # Get UTC Offset
            # The offset in hours from UTC. Positive for zones west of UTC.
            # time.timezone gives seconds west of UTC.
            return f"{-time.timezone / 3600.0:+.1f}#"

        # --- Location Commands ---
        if command == ':Gg#': # Get Longitude
            # LX200 longitude is degrees *west* of Greenwich.
            lon_deg = float(self.controller.observer.lon)
            return f"{lon_deg * -1:+04.0f}*00#" 
        
        if command == ':Gt#': # Get Latitude
            lat_deg, lat_min = self._deg_to_degmin(float(self.controller.observer.lat))
            return f"{lat_deg:+03d}*{lat_min:02.0f}#"
            
        if command.startswith(':Sg'): # Set Longitude, e.g., :SgDDD*MM#
             match = re.match(r":Sg([+-]?\d{2,3})\*(\d{2})#", command)
             if match:
                 degrees_str, minutes_str = match.groups()
                 degrees = int(degrees_str)
                 minutes = int(minutes_str)
                 # LX200 longitude is degrees WEST, so negate
                 self.controller.observer.lon = str(-(degrees + minutes / 60.0))
                 return "1"
             return "0"
             
        if command.startswith(':St'):  # Set Latitude, e.g., :St+DD*MM#
                match = re.match(r":St([+-]?\d{2})\*(\d{2})#", command)
                if match:
                    degrees_str, minutes_str = match.groups()
                    degrees = int(degrees_str)
                    minutes = int(minutes_str)
                    self.controller.observer.lat = str(degrees + minutes / 60.0)
                    return "1"
                return "0"

        # Acknowledge other set commands to prevent client errors
        if command.startswith((':SC', ':SL', ':So', ':Sh', ':SG')):
            return "1"
        
        if command.startswith(':R'):
            return ""  # Rate commands
            
        # Default response for unknown commands
        print(f"Unknown command: {command}")
        return "0"


# --- Main Execution ---
if __name__ == "__main__":
    print("Starting LX200 Proxy for Custom Mount")
    print("=====================================")
    
    # 1. Initialize the controller that manages all hardware
    print("Initializing telescope controller...")
    controller = TelescopeController()
    
    # Give hardware time to initialize
    time.sleep(2)
    
    print("Hardware initialization complete.")
    print(f"Current RA: {controller._degrees_to_hms_str(controller.ra_current_deg)}")
    print(f"Current DEC: {controller.get_dec_coord()}")
    print(f"Sidereal Time: {controller._degrees_to_hms_str(controller.get_sidereal_time_deg())}")
    
    # 2. Start the LX200 proxy server, passing it the controller instance
    print("\nStarting LX200 proxy server...")
    proxy = LX200Proxy(SERVER_HOST, SERVER_PORT, controller)
    
    # 3. The server will run until manually stopped (Ctrl+C)
    try:
        proxy.start()
    except KeyboardInterrupt:
        print("\n\nShutting down gracefully...")
        controller.stop_all_motion()
        print("Goodbye!")
