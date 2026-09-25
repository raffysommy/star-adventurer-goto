import socket
import re
import time
import synscan
import serial
from threading import Thread, RLock
import threading
from datetime import datetime
import ephem
from typing import Callable

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs
# ========================== CONFIGURATION ============================
WEB_HOST = "0.0.0.0"
WEB_PORT = 8080

SERVER_HOST         = '127.0.0.1'
SERVER_PORT         = 5001

SERIAL_PORT = '/dev/ttyUSB0'

OBSERVER_LON        = 14.4377
OBSERVER_LAT        = 40.8728

DEC_AXIS_REVERSED = False ## This is a new flag, PIER May not be accurate (or work only for guiding, need to test).

FLIP_RA_GUIDING_ON_MERIDIAN = False ## Flippa a true se non va, GPT dice che non c'è bisogno di invertire RA in beyond the pole ma non mi fido

PIER_EAST_SIDE = False ## Side of the lights on the star adventurer, with camera pointing in the opposite direction of north pole

# ========================== CONFIGURATIONEND ============================

pierEastSide = PIER_EAST_SIDE 

meridian_flipped = False

my_mount = ephem.Observer()
my_mount.lon, my_mount.lat = OBSERVER_LON, OBSERVER_LAT
def get_siderial_time():
    global my_mount
    my_mount.date = datetime.now()
    return my_mount.sidereal_time()

def get_hour_angle(ra):
    global my_mount
    my_mount.date = datetime.now()
    return (my_mount.sidereal_time() / ephem.pi * 180 - ra) % 360.0 + offset_star_adventurer

def get_right_ascension(ha):
    global my_mount
    my_mount.date = datetime.now()
    ha = ha - offset_star_adventurer
    return (my_mount.sidereal_time() / ephem.pi * 180 - ha) % 360.0

arduinodec = serial.serial_for_url('socket://DEC-MOUNT-CONTROLLER.local:7777', timeout=2)
smc=synscan.motors(serial_dev=SERIAL_PORT)
smc.get_values({},True)
offset_star_adventurer = 2 #degrees: This compensate the fact that the start adventurer is not able to go to zero RA, so this is the 0 position
smc.set_pos(offset_star_adventurer,0)
ra_current = 0
ra_target = 0
ra_slewing = False
ra_tracking = True
ra_guiding_east = False
ra_guiding_west = False
ra_guiding_north = False
ra_guiding_south = False
dec_current_steps = None
dec_current_coord = "Not read yet"
    
ra_locker = RLock()
step_per_rev_dec=585600/2
sidereal_speed_refracted=0.004176


## OK what the mount ""really"" understand is the hour angle, that increases over time in tracking while the Siderial time increases
## I need location and time for calculating the siderial time
## the RA = (HA - ST)%360 //Check the modulo here
## this means than when I want a target HA = RA - ST
## Now if the target (in HA) is > 180° (181, so it never navigates to 0) what I do is subtract 180 and do 180-Dec for the declination
## Then the mount should keep a meridian flipped flag, where in that case RA is returned as (RA + 180)%360 and DEC as Dec=180-FDec


def tracking():
    global ra_tracking
    ra_tracking = True
    smc.axis_stop_motion(1,True)
    smc.axis_set_speed(1, sidereal_speed_refracted)
    smc.axis_set_motion_mode(1, True, True, False)
    smc.axis_start_motion(1)
    
def update_ra_current():
    global ra_current, ra_locker, ra_slewing, ra_tracking, ra_target, meridian_flipped, ra_guiding_east, ra_guiding_west
    ah_previous = -2000 
    
    while True:
        # Acquire the lock BEFORE talking to the mount to prevent collisions with guiding
        with ra_locker:
            try:
                # 1. Physical Serial Read (Protected by Lock)
                raw_val = smc.axis_get_pos(1)

                # 2. VALIDATION: Check if it's a number
                try:
                    ah_read = float(raw_val)
                except (ValueError, TypeError):
                    # If we got junk, we don't crash, we just exit the 'with' block and try later
                    print(f"[WARNING] Junk data received: {raw_val}. Skipping cycle.")
                    ah_read = None 

                # 3. Process the data if it's valid
                if ah_read is not None:
                    # Stall detection logic
                    if ((ah_read == ah_previous) or (ah_read < ah_previous)) and (ra_tracking or ra_guiding_east or ra_guiding_west):
                        tracking() 

                    ra_current = get_right_ascension(ah_read)
                    ah_previous = ah_read

            except Exception as e:
                print(f"[ERROR] Communication exception: {e}")

        # 4. Release pressure on the CPU and the Serial Port
        # Increased to 0.5s to give PhD2 more 'airtime' to send guide pulses
        time.sleep(0.5)

update_ra_current_thread = threading.Thread(target=update_ra_current)            
update_ra_current_thread.start()

with ra_locker:
    tracking()
    time.sleep(1)

def goto_ra():
    global ra_slewing, ra_current, ra_target, ra_locker, ra_tracking
    with ra_locker:
        ra_tracking = False
        ra_slewing = True
        CW0 = get_hour_angle(ra_current) - get_hour_angle(ra_target)
        smc.axis_stop_motion_hard(1,True)
        #smc.axis_set_goto_target(1, ra_target) #goto is bugged
        while(not smc.get_status(1)['Stopped']):
            time.sleep(0.2)
            print(smc.update_current_values())
        direction = (CW0<=0)
        smc.axis_set_motion_mode(1,True, direction ,False) ##repetita aiuvant
        smc.axis_set_speed(1, 128 * sidereal_speed_refracted)
        smc.axis_start_motion(1)
        print(smc.update_current_values())
    print(CW0)
    LAST_CW = CW0 #keep track of motor moving
    print("Current:"+str(ra_current)+", Target "+ str(ra_target))
    slow = False
    while ra_slewing:
        with ra_locker:
            CW1 =   get_hour_angle(ra_current) - get_hour_angle(ra_target)
            if abs(LAST_CW-CW1)<=0.01:
                smc.axis_start_motion(1)
        LAST_CW=CW1
        print("Current:"+str(get_hour_angle(ra_current))+", Target "+ str(get_hour_angle(ra_target)))
        if(abs(CW1)<0.1) and not slow:
            with ra_locker:
                 smc.axis_stop_motion(1,True)
                 smc.axis_set_speed(1, 8 * sidereal_speed_refracted) # Slow down, we are getting there
                 smc.axis_start_motion(1)
                 slow = True
               
        if CW0*CW1 <= 0: # changed sign = overshot, or wrong direction
            with ra_locker:
                smc.axis_stop_motion(1, True)
                smc.axis_set_speed(1, sidereal_speed_refracted) # reset speed
                ra_slewing= False
                break
        time.sleep(0.2)

                
    with ra_locker:
        ra_slewing = False
        ra_tracking = True
        tracking()
        smc.update_current_values()

    print(smc.values[1]['Status'])


goto_ra_thread = threading.Thread(target=goto_ra)

class EventScheduler:
    def __init__(self):
        """Initialize the event scheduler."""
        self.events = []
        self.lock = threading.Lock()

    def schedule_event(self, delay_ms: int, callback: Callable):
        """
        Schedule an event to run in the future.

        Args:
            delay_ms (int): Delay in milliseconds before the event runs.
            callback (Callable): The function to execute when the event is triggered.
        """
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

scheduler_slew_north = EventScheduler()
scheduler_slew_east = EventScheduler()
scheduler_slew_west = EventScheduler()
scheduler_slew_south = EventScheduler()



def slew_ra_east():
    global ra_locker
    with ra_locker:
        smc.axis_set_speed(1, 1.5 * sidereal_speed_refracted)
        
def slew_ra_west():
    global ra_locker
    with ra_locker:
        smc.axis_set_speed(1, 0.5 * sidereal_speed_refracted)

def stop_guiding_skywatcher():
    global ra_guiding_east, ra_guiding_west, ra_locker
    with ra_locker:
        ra_guiding_east = False
        ra_guiding_west = False
        smc.axis_set_speed(1, sidereal_speed_refracted)


def stop_guiding_arduino():
    global ra_guiding_north, ra_guiding_south
    ra_guiding_north = False
    ra_guiding_south = False
    arduinodec.write(":Q#".encode())
    arduinodec.readline()

def time_to_degrees(hours, minutes, seconds):
    """
    Converts a time (hours, minutes, seconds) in the range 00:00:00-23:59:59
    (and beyond, with circular wrapping) to a float in the range 0.0 to +360.0 degrees.

    Args:
        hours (int): Hours (can exceed 23 or go below 0 for circular wrapping).
        minutes (int): Minutes (can exceed 59 or go below 0 for circular wrapping).
        seconds (int): Seconds (can exceed 59 or go below 0 for circular wrapping).

    Returns:
        float: A value in the range 0.0 to +360.0 degrees.
    """
    # Convert the time into total seconds (allow for overflow)
    total_seconds = hours * 3600 + minutes * 60 + seconds

    # Normalize total seconds to the 24-hour clock (0 <= seconds < 86400)
    seconds_in_day = 24 * 3600  # Total seconds in a day
    normalized_seconds = total_seconds % seconds_in_day

    # First, normalize to [0, 360) degrees
    degrees = (normalized_seconds / seconds_in_day) * 360.0

    return degrees

def degrees_to_hms(degrees: float) -> tuple:
    """
    Converts degrees to time (hours, minutes, seconds).

    Args:
        degrees (float): Degrees to be converted (0 to 360).

    Returns:
        tuple: (hours, minutes, seconds)
    """
    degrees = degrees % 360
    if degrees < 0:
        degrees += 360  # Ensure it's in the range [0, 360)
    
    total_hours = degrees / 15
    hours = int(total_hours)
    remaining_minutes = (total_hours - hours) * 60
    minutes = int(remaining_minutes)
    seconds = int((remaining_minutes - minutes) * 60)

    return hours, minutes, seconds

def dec_to_steps(degrees):
    dec_in_centi_degrees = degrees * 3600.0 ##GPT ha detto di togliere l'abs, non è che mi fido ma proviamo
    #dec_in_centi_degrees = abs(degrees) * 3600.0
    # Map the declination to the range 0 to step_per_rev_dec steps
    steps = int(((dec_in_centi_degrees + 3600.0 * 180.0) / (3600.0 * 360.0)) * step_per_rev_dec)

    return steps

def steps_to_coord(steps):
    global meridian_flipped
    reverse_dec = meridian_flipped ^ DEC_AXIS_REVERSED ## If this doesn't work remove the XOR
    """Converts stepper coordinates (steps) back to DD:MM:SS format, handling sign separately."""

    # Reverse the steps to degrees by scaling back to the -180 to +180 range
    normalized_dec = (steps / step_per_rev_dec) * 360.0 - 180.0

    if reverse_dec:
        normalized_dec = ((180 - normalized_dec) + 180) % 360 -180 #flip back the meridian dec coordinates

    # Determine the sign: If the normalized value is negative, sign is -1; otherwise, +1
    sign = -1 if normalized_dec < 0 else 1
    normalized_dec = abs(normalized_dec)  # Make the normalized value positive for processing

    # Convert back to degrees, minutes, and seconds
    deg = int(normalized_dec)  # Degrees
    minutes = int((normalized_dec - deg) * 60)  # Minutes
    seconds = round(((normalized_dec - deg) * 60 - minutes) * 60)  # Seconds
    
    # Format the result as DD*MM:SS, including the sign
    formatted_coord = f"{sign * deg:02d}*{minutes:02d}:{seconds:02d}"
    
    return formatted_coord
    

# LX200 Proxy Class
class LX200Proxy:
    def __init__(self, host=SERVER_HOST, port=SERVER_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self.client_socket = None
        self.client_address = None

    def start(self):
        """Starts the TCP server to listen for incoming connections."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind((self.host, self.port))
        self.sock.listen(1)
        print(f"Listening on {self.host}:{self.port}")
        while True:
            self.client_socket, self.client_address = self.sock.accept()
            print(f"Connection from {self.client_address}")
            self.handle_client()

    def handle_client(self):
        """Handles the communication with the connected client."""
        try:
            while True:
                data = self.client_socket.recv(1024).decode('utf-8').strip()
                if not data:
                    break
                print(f"Received: {data}")
                try:
                    response = self.process_command(data)
                    print(f"Response: {response}")
                except Exception as e:
                    print(f"[ERROR] Command '{data}' caused exception: {e}")
                    response = "#"
                if response:
                    self.client_socket.sendall(response.encode('utf-8'))
        finally:
            self.client_socket.close()

    def process_command(self, command):
        """Processes incoming LX200 protocol commands."""
        # Handle different commands
        if command.startswith(':GR'):
            return self.get_ra()
        elif command.startswith(':GD'):
            return self.get_dec()
        elif command.startswith(':GVP'):
            return self.get_version()
        elif command.startswith(':GVN'):
            return self.get_version_details()
        elif command.startswith(':GVR'):
            return "001012023#"
        elif command.startswith(':GVD'):
            return "01.1#"
        elif command.startswith(':GVT'):
            return datetime.now().strftime("%H:%M:%S#")
        elif command.startswith(':GVF'):
            return datetime.now().strftime("11#")
        elif command.startswith(':Sr'):
            return self.set_ra(command)
        elif command.startswith(':Sd'):
            return self.set_dec(command)
        elif command.startswith(':MS'):
            return self.slew()
        elif command.startswith(':M'):
            return self.slow_move(command)
        elif command.startswith(':Q'):
            return self.stop_all()
        elif command.startswith(':CM'):
            return self.sync()
        elif command.startswith(':D'):
            return self.report_slewing()
        elif command.startswith(':GM'):
            return "Site1Name#"  # Replace with dynamic site name if necessary
        elif command.startswith(':GT'):
            return "60.0#"  # Replace with actual tracking rate calculation if necessary
        elif command.startswith(':Gt'):
            degrees = int(my_mount.lat)
            minutes = abs(my_mount.lat % 1 * 60)
            return f"{degrees:+03d}*{minutes:02.0f}#" # Latitude, positive for North
        elif command.startswith(':Gg'):
             degrees = -int(my_mount.lon) # East is negative as per meade specification
             minutes = abs(my_mount.lon % 1 * 60)
             return f"{degrees:+03d}*{minutes:02.0f}#"
        elif command.startswith(':Sg'):
             ## Set the mount location
             match = re.match(r":Sg([+-]?\d{2,3})\*(\d{2})#", command)
             if match:
                 sign = 1 if command.startswith(':Sg-') else -1
                 deg, minutes = map(int, match.groups())
                 my_mount.lon = sign * (deg + minutes / 60)
                 return "1"
             return "0"
        elif command.startswith(':St'):
                ## Set the mount location
                match = re.match(r":St([+-]?\d{2})\*(\d{2})#", command)
                if match:
                    sign = -1 if command.startswith(':St-') else 1
                    deg, minutes = map(int, match.groups())
                    my_mount.lat = sign * (deg + minutes / 60)
                    return "1"
                return "0"
        elif command.startswith(':So') or command.startswith(":Sh") or command.startswith(":SG") or command.startswith(":SL") or command.startswith(":SC"):
             return "1" # Set commands for time and limits not supported but we don't complain
        elif command.startswith(':R'):
             return "" # R commands not supported but we don't complain
        elif command.startswith(':GG'):
             return f"{-time.timezone / 3600.0:+.1f}#"  # Timezone offset
        elif command.startswith(':Gc'):
             return "24#" # Calender format 24
        elif command.startswith(':GL'):
             utc_time = datetime.now().strftime("%H:%M:%S#")
             return utc_time  # Current UTC time
        elif command.startswith(':GC'):
             utc_date = datetime.now().strftime("%m/%d/%y#")
             return utc_date  # Current UTC date    
        else:
            return "Invalid Command"

    # Boilerplate functions
    def get_version(self):
        """Responds with version."""
        return "Proxino#"
    def get_version_details(self):
        """Responds with version details."""
        return "1.0#"
        
    def get_ra(self):
        """Handles RA getting in HH:MM:SS format."""
        global ra_slewing, ra_current, ra_locker, meridian_flipped
        with ra_locker:
                pos = ra_current # Update value in bg from the bg thread
        if meridian_flipped:
            pos = get_right_ascension((get_hour_angle(pos) - 180) % 360)
            print("Meridian Flipped")
        hours, minutes, seconds = degrees_to_hms(pos)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}#"
        

    def get_dec(self):
        """Handles DEC getting in DD*MM:SS format"""
        global dec_current_steps, dec_current_coord

        arduinodec.write(":Gd#".encode())
        dec_read = arduinodec.readline()

        try:
            steps = int(dec_read)
            self._last_valid_dec_steps = steps
        except ValueError:
            # If it fails, use the last valid steps if available
            steps = getattr(self, '_last_valid_dec_steps', 0)

        coord = steps_to_coord(steps)
        dec_current_steps = steps
        dec_current_coord = coord
        return f"{coord}#"
        
    def set_ra(self, command):
        """Handles RA setting in HH:MM:SS"""
        global ra_slewing, ra_target, ra_locker, meridian_flipped
        match = re.match(r":Sr(\d{2}):(\d{2}):(\d{2})#", command)
        if match:
            hours, minutes, seconds = map(int, match.groups()[:3])
            ra_target_temp = time_to_degrees(hours, minutes, seconds)
            with ra_locker:
                print("Hour Angle Target: " +str(get_hour_angle(ra_target_temp)))
                if get_hour_angle(ra_target_temp) > 180:
                    meridian_flipped = True
                    ra_target = get_right_ascension(get_hour_angle(ra_target_temp + 180) % 360)
                else:
                    meridian_flipped = False
                    ra_target = ra_target_temp

            return "#"

    def set_dec(self, command):
        global meridian_flipped, DEC_AXIS_REVERSED
        reverse_dec = meridian_flipped ^ DEC_AXIS_REVERSED ## If this doesn't work remove the XOR
        """Handles DEC setting in DD*MM:SS or DD*MM format."""
        match = re.match(r":Sd([+-]?\d{2})\*(\d{2}):(\d{2})#", command)
        if match:
            sign = -1 if command.startswith(':Sd-') else 1
            deg, minutes, seconds = map(int, match.groups())
            degrees = (abs(deg) + abs(minutes)/60 + abs(seconds)/3600) * sign
            if reverse_dec:
                degrees = 180 - degrees
            print("Degrees: " + str(degrees))
            steps = dec_to_steps(degrees)
            arduinodec.write((":Sd" + str(steps) + "#").encode())
            if arduinodec.readline()== "#":
               return "#"
            return "#"


    def slew(self):
        global ra_slewing, ra_current, ra_target, ra_locker, goto_ra_thread
        if not goto_ra_thread.is_alive():
            goto_ra_thread = None
            goto_ra_thread = threading.Thread(target=goto_ra)
            goto_ra_thread.start()
        arduinodec.write(":MS#".encode())
        if arduinodec.readline()== "#":
            return"0"   
        return "0"

    def slow_move(self, command):
        """Handles slow move command (N, S, E, W)."""
        global ra_guiding_north, ra_guiding_east, ra_guiding_south, ra_guiding_west
        direction = command[2:]
        duration_flag = False
        if direction.startswith("g"):
            direction = direction[1:] # Skip the g part
            duration_flag = True
            
        reverse_dec = meridian_flipped ^ DEC_AXIS_REVERSED ## If this doesn't work remove the XOR
        if direction.startswith("n"):
            ra_guiding_north = True
            duration = re.match(r"n(\d+)", direction)
            if duration and duration_flag:
                value = int(duration.group(1))
                if not reverse_dec:
                    scheduler_slew_north.schedule_event(value, lambda:  stop_guiding_arduino())
                else:
                    scheduler_slew_south.schedule_event(value, lambda:  stop_guiding_arduino())    
            if not reverse_dec:
                arduinodec.write(":Mn#".encode())
            else:
                arduinodec.write(":Ms#".encode())                
            arduinodec.readline()
        elif direction.startswith("s"):
            ra_guiding_south = True
            duration = re.match(r"s(\d+)", direction)
            if duration and duration_flag:
                value = int(duration.group(1))
                if not reverse_dec:
                    scheduler_slew_south.schedule_event(value, lambda:  stop_guiding_arduino())
                else:
                    scheduler_slew_north.schedule_event(value, lambda:  stop_guiding_arduino())    
            if not reverse_dec:
                arduinodec.write(":Ms#".encode())
            else:
                arduinodec.write(":Mn#".encode())  
            arduinodec.readline()
        elif direction.startswith("w"):
            ra_guiding_west = True
            duration = re.match(r"w(\d+)", direction)
            if duration and duration_flag:
                value = int(duration.group(1))
                if not (meridian_flipped and FLIP_RA_GUIDING_ON_MERIDIAN):
                    scheduler_slew_west.schedule_event(value, lambda:  stop_guiding_skywatcher())
                else:
                    scheduler_slew_east.schedule_event(value, lambda:  stop_guiding_skywatcher())   
            if not (meridian_flipped and FLIP_RA_GUIDING_ON_MERIDIAN):
                slew_ra_west()
            else:
                slew_ra_east()
        elif direction.startswith("e"):
            ra_guiding_east = True
            duration = re.match(r"e(\d+)", direction)
            if duration and duration_flag:
                value = int(duration.group(1))
                if not (meridian_flipped and FLIP_RA_GUIDING_ON_MERIDIAN):
                    scheduler_slew_east.schedule_event(value, lambda:  stop_guiding_skywatcher())
                else:
                    scheduler_slew_west.schedule_event(value, lambda:  stop_guiding_skywatcher())   
            if not (meridian_flipped and FLIP_RA_GUIDING_ON_MERIDIAN):
                slew_ra_east()
            else:
                slew_ra_west()
        return "0"

    def stop_all(self):
        """Stops all movement."""
        global ra_slewing, ra_current, ra_target, ra_locker, goto_ra_thread, ra_guiding_north, ra_guiding_east, ra_guiding_south, ra_guiding_west
        with ra_locker:
            ra_slewing = False
        if goto_ra_thread.is_alive():
            goto_ra_thread.join()
        with ra_locker:
            if ra_guiding_east or ra_guiding_west:
                scheduler_slew_east.cancel_all_events()
                scheduler_slew_west.cancel_all_events()
                stop_guiding_skywatcher()
            elif ra_guiding_south or ra_guiding_north:
                scheduler_slew_south.cancel_all_events()
                scheduler_slew_north.cancel_all_events()
                stop_guiding_arduino()
            else:
                tracking()                    
        arduinodec.write(":Q#".encode())
        if arduinodec.readline()== "#":
            return ""
        return ""

    def sync(self):
        """Synchronize telescope."""
        global ra_slewing, ra_current, ra_target, ra_locker
        with ra_locker:
            if ra_current!=ra_target:
                smc.axis_stop_motion(1,True)
                smc.axis_set_pos(1, get_hour_angle(ra_target))
                ra_current = ra_target
                tracking()
        arduinodec.write(":CM#".encode())
        arduinodec.readline()
        return ":Coordinates matched #"

    def report_slewing(self):
        """Reports if the telescope is slewing."""
        global ra_slewing, ra_current, ra_target, ra_locker
        arduinodec.write(":D#".encode())
        dec_slewing = (arduinodec.readline()=="1")
        with ra_locker:
            if ra_slewing | dec_slewing | ra_guiding_south | ra_guiding_north | ra_guiding_east | ra_guiding_west :
               return chr(127) + "#"
            else:
               return "#" # not slewing or goto

###WEBUI
DASHBOARD_HTML = """
<!doctype html>
<html lang="en">
<head>
    <meta charset="utf-8">
    <meta
        name="viewport"
        content="width=device-width, initial-scale=1"
    >

    <title>Star Adventurer</title>

    <style>
        body {
            font-family: sans-serif;
            max-width: 900px;
            margin: 30px auto;
            padding: 0 15px;
            background: #111;
            color: #eee;
        }

        .grid {
            display: grid;
            grid-template-columns:
                repeat(auto-fit, minmax(180px, 1fr));
            gap: 12px;
        }

        .card {
            background: #222;
            padding: 16px;
            border-radius: 8px;
        }

        .value {
            margin-top: 6px;
            font-family: monospace;
            font-size: 1.3em;
        }

        label {
            display: block;
            margin: 15px 0;
        }

        button {
            padding: 10px 18px;
            cursor: pointer;
        }

        .true {
            color: #ffb74d;
        }

        .false {
            color: #81c784;
        }

        #save-result {
            margin-left: 10px;
        }
    </style>
</head>

<body>
    <h1>Star Adventurer</h1>

    <div class="grid">
        <div class="card">
            <div>Current RA</div>
            <div class="value" id="ra">...</div>
        </div>

        <div class="card">
            <div>Current DEC</div>
            <div class="value" id="dec">...</div>
        </div>

        <div class="card">
            <div>DEC steps</div>
            <div class="value" id="dec-steps">...</div>
        </div>

        <div class="card">
            <div>Mount state</div>
            <div class="value" id="mount-state">...</div>
        </div>

        <div class="card">
            <div>Meridian flipped</div>
            <div class="value" id="meridian-flipped">...</div>
        </div>

        <div class="card">
            <div>Effective DEC flip</div>
            <div class="value" id="effective-dec-flip">...</div>
        </div>
    </div>

    <h2>Runtime settings</h2>

    <div class="card">
        <label>
            <input
                type="checkbox"
                id="dec-axis-reversed"
            >
            DEC_AXIS_REVERSED
        </label>

        <label>
            <input
                type="checkbox"
                id="flip-ra-guiding"
            >
            FLIP_RA_GUIDING_ON_MERIDIAN
        </label>

        <button onclick="saveSettings()">
            Apply settings
        </button>

        <span id="save-result"></span>
    </div>

    <script>
        let settingsInitialized = false;

        function showBoolean(id, value) {
            const element = document.getElementById(id);

            element.textContent = value ? "TRUE" : "FALSE";
            element.className =
                "value " + (value ? "true" : "false");
        }

        async function refreshStatus() {
            try {
                const response = await fetch(
                    "/api/status",
                    {cache: "no-store"}
                );

                const status = await response.json();

                document.getElementById("ra").textContent =
                    status.ra;

                document.getElementById("dec").textContent =
                    status.dec;

                document.getElementById("dec-steps").textContent =
                    status.dec_steps === null
                        ? "unknown"
                        : status.dec_steps;

                document.getElementById(
                    "mount-state"
                ).textContent = status.state;

                showBoolean(
                    "meridian-flipped",
                    status.meridian_flipped
                );

                showBoolean(
                    "effective-dec-flip",
                    status.effective_dec_flip
                );

                if (!settingsInitialized) {
                    document.getElementById(
                        "dec-axis-reversed"
                    ).checked = status.dec_axis_reversed;

                    document.getElementById(
                        "flip-ra-guiding"
                    ).checked = status.flip_ra_guiding;

                    settingsInitialized = true;
                }
            } catch (error) {
                document.getElementById(
                    "mount-state"
                ).textContent = "Web connection error";
            }
        }

        async function saveSettings() {
            const body = new URLSearchParams({
                dec_axis_reversed:
                    document.getElementById(
                        "dec-axis-reversed"
                    ).checked ? "1" : "0",

                flip_ra_guiding:
                    document.getElementById(
                        "flip-ra-guiding"
                    ).checked ? "1" : "0"
            });

            const result = document.getElementById(
                "save-result"
            );

            try {
                const response = await fetch(
                    "/api/settings",
                    {
                        method: "POST",
                        headers: {
                            "Content-Type":
                                "application/x-www-form-urlencoded"
                        },
                        body: body
                    }
                );

                if (!response.ok) {
                    throw new Error("HTTP error");
                }

                result.textContent = "Saved";
                settingsInitialized = false;
                await refreshStatus();

                setTimeout(() => {
                    result.textContent = "";
                }, 2000);
            } catch (error) {
                result.textContent = "Error";
            }
        }

        refreshStatus();
        setInterval(refreshStatus, 1000);
    </script>
</body>
</html>
"""


def dashboard_ra_string():
    pos = ra_current

    if meridian_flipped:
        pos = get_right_ascension(
            (get_hour_angle(pos) - 180) % 360
        )

    hours, minutes, seconds = degrees_to_hms(pos)

    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def dashboard_mount_state():
    if ra_slewing:
        return "SLEWING"

    directions = []

    if ra_guiding_north:
        directions.append("N")

    if ra_guiding_south:
        directions.append("S")

    if ra_guiding_east:
        directions.append("E")

    if ra_guiding_west:
        directions.append("W")

    if directions:
        return "GUIDING " + "/".join(directions)

    if ra_tracking:
        return "TRACKING"

    return "IDLE"


def dashboard_status():
    return {
        "ra": dashboard_ra_string(),
        "dec": dec_current_coord,
        "dec_steps": dec_current_steps,
        "state": dashboard_mount_state(),

        "meridian_flipped":
            meridian_flipped,

        "dec_axis_reversed":
            DEC_AXIS_REVERSED,

        "flip_ra_guiding":
            FLIP_RA_GUIDING_ON_MERIDIAN,

        "effective_dec_flip":
            meridian_flipped ^ DEC_AXIS_REVERSED,
    }


class DashboardHandler(BaseHTTPRequestHandler):

    def send_body(
        self,
        status_code,
        content_type,
        body
    ):
        if isinstance(body, str):
            body = body.encode("utf-8")

        self.send_response(status_code)

        self.send_header(
            "Content-Type",
            content_type
        )

        self.send_header(
            "Content-Length",
            str(len(body))
        )

        self.send_header(
            "Cache-Control",
            "no-store"
        )

        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]

        if path == "/":
            self.send_body(
                200,
                "text/html; charset=utf-8",
                DASHBOARD_HTML
            )
            return

        if path == "/api/status":
            self.send_body(
                200,
                "application/json; charset=utf-8",
                json.dumps(dashboard_status())
            )
            return

        self.send_body(
            404,
            "text/plain; charset=utf-8",
            "Not found"
        )

    def do_POST(self):
        global DEC_AXIS_REVERSED
        global FLIP_RA_GUIDING_ON_MERIDIAN

        path = self.path.split("?", 1)[0]

        if path != "/api/settings":
            self.send_body(
                404,
                "text/plain; charset=utf-8",
                "Not found"
            )
            return

        content_length = int(
            self.headers.get("Content-Length", "0")
        )

        request_body = self.rfile.read(
            content_length
        ).decode("utf-8")

        values = parse_qs(request_body)

        DEC_AXIS_REVERSED = (
            values.get(
                "dec_axis_reversed",
                ["0"]
            )[0] == "1"
        )

        FLIP_RA_GUIDING_ON_MERIDIAN = (
            values.get(
                "flip_ra_guiding",
                ["0"]
            )[0] == "1"
        )

        self.send_body(
            200,
            "application/json; charset=utf-8",
            json.dumps({
                "ok": True,
                "dec_axis_reversed":
                    DEC_AXIS_REVERSED,
                "flip_ra_guiding":
                    FLIP_RA_GUIDING_ON_MERIDIAN,
            })
        )

    def log_message(self, format, *args):
        pass


def start_dashboard():
    server = ThreadingHTTPServer(
        (WEB_HOST, WEB_PORT),
        DashboardHandler
    )

    print(
        f"Dashboard listening on "
        f"http://{WEB_HOST}:{WEB_PORT}"
    )

    server.serve_forever()
    
##ENDWEBUI

# Run the proxy server
if __name__ == "__main__":
    arduinodec.flushInput()
    arduinodec.flushOutput()
    time.sleep(2)
    arduinodec.write(":Y#".encode())
    arduinodec.readline()
    # Invia il target dei passi per 0°
    arduinodec.write((f":Sd146400#").encode())
    arduinodec.readline()
    # Sincronizza la posizione attuale a quei passi
    arduinodec.write(":CM#".encode())
    arduinodec.readline()
    if pierEastSide:
        arduinodec.write(":CP#".encode())
        arduinodec.readline()
    dashboard_thread = threading.Thread(
        target=start_dashboard,
        daemon=True
    )
    dashboard_thread.start()
    proxy = LX200Proxy()
    proxy.start()
