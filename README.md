# Star Adventurer Classic and 2i GoTo Mod

This project transforms a Sky-Watcher Star Adventurer or Star Adventurer 2i into a fully functional GoTo mount with guiding capabilities. It achieves this by adding a custom, Arduino-powered Declination (DEC) axis that works in tandem with the mount's native Right Ascension (RA) motor.

The system is controlled via a Python-based proxy server that emulates the Meade LX200 protocol, allowing you to control your modified mount from most popular planetarium or astrophotography software (like Stellarium, Cartes du Ciel) using INDI LX200 GPS driver (it may work on Windows too).

The core of this project relies on the [**advanced firmware**](https://skywatcher.com/post/resource/star-adventurer-advanced-firmware/) for the Star Adventurer (not needed for the 2i), which enables serial communication. This firmware is a bit buggy, and this project includes several software workarounds to ensure stable and reliable operation despite those quirks.

## Features

-   **Full GoTo Capability:** Slew to any celestial object from your favorite software.
    
-   **LX200 Protocol Emulation:** Wide compatibility with astrophotography software.
    
-   **Full Guiding Support:** Pulse guiding on both RA and DEC axes (via direct driving or  ST4).

![Setup](setup.png?raw=true "Setup")
    

## 1. Hardware Requirements

### Electronic Components

The heart of the DEC axis is a simple circuit controlled by an Arduino Nano.

-   **Arduino Nano:** The microcontroller that runs the DEC motor logic.
    
-   **EasyDriver v4.5 (or similar):** A simple stepper motor driver.
    
-   **NEMA 17 Bipolar Stepper Motor:** The motor that drives the DEC axis.
    
-   **DC Female Power Jack:** To power the EasyDriver and motor (9-12V recommended). A step-up converter can be used if you are powering from a 5V source.
    
-   **(Optional) RJ12 Socket:** For a standard ST4 guiding port. The system also supports direct pulse guiding commands from your control software.
    

### 3D Printed Parts (`/stl` folder)

You will need to print the following parts to assemble the DEC axis.

-   `Nema17MotorHolder.stl`: Mounts the NEMA 17 motor to the Star Adventurer's DEC bracket.
    
    -   Requires: 4x M3 screws (for the motor) and 2x M5 screws with bolts (for the bracket).
        
-   `StarAdventurerDeclinationKnobMod.stl`: Replaces the original DEC knob on the Star Adventurer and connects to the timing belt.
    
    -   Requires: 1x small M2 screw to secure it.
        
-   `Nema17Knob.stl`: Attaches to the NEMA 17 motor shaft.
    
    -   Requires: 1x M3 screw and nut for tightening.
        

### Mechanical Parts

-   **GT2 Timing Belt (400mm loop, 6mm width):** Connects the motor to the declination knob.
    

## 2. Wiring and Assembly

The circuit connects the Arduino to the EasyDriver to control the stepper motor. It's crucial to ensure all ground connections (GND) are common between the Arduino, the EasyDriver, and the DC power supply.

-   **Power:** Connect your 9-12V DC power supply to the EasyDriver's power input (M+ and GND).
    
-   **Arduino to EasyDriver:**
    
    -   Connect Arduino **Pin D2** to EasyDriver **STEP**.
        
    -   Connect Arduino **Pin D3** to EasyDriver **DIR**.
        
    -   Connect Arduino **GND** to EasyDriver **GND** (the one near the power input).
        
-   **Motor to EasyDriver:** Connect the four wires of the NEMA 17 motor to the EasyDriver's motor outputs (OUT1A, OUT1B, OUT2A, OUT2B).
    

#### Circuit Diagram

![Setup](wiring.png?raw=true "Setup")

## 3. Software Setup and Installation

### Step 1: Clone the Repository

First, clone this repository to your local machine.

```
git clone https://github.com/raffysommy/star-adventurer-goto
cd star-adventurer-goto

```

### Step 2: Set up the Arduino

1.  Open the Arduino IDE.
    
2.  Navigate to the `arduino` folder in this project.
    
3.  Open the `.ino` sketch file.
    
4.  Connect your Arduino Nano to your computer.
    
5.  Select the correct board (Arduino Nano) and COM/TTY port from the `Tools` menu.
    
6.  Upload the sketch to the Arduino.
    

### Step 3: Install Python Dependencies

It is highly recommended to use a Python virtual environment.

```
# Create a virtual environment (optional but recommended)
python -m venv venv
source venv/bin/activate  # On Windows use `venv\Scripts\activate`

# Install the required packages
pip install -r requirements.txt

```

## 4. Configuration

Before running the server, you must configure the Python script with your specific setup.

Open the `lx200.py` file and edit the following constants at the top:

-   **`SERIAL_PORT`**: Change this to the serial port your Arduino is connected to (e.g., `COM3` on Windows, `/dev/ttyUSB1` on Linux).
    
-   **`OBSERVER_LON`**: Set your observer's longitude in decimal degrees (East is positive).
    
-   **`OBSERVER_LAT`**: Set your observer's latitude in decimal degrees (North is positive).

**Initial Pier Side**: You may need to set your mount's starting pier side. In `new_lx200.py`, find the `TelescopeController` class and its `__init__` method. Set the `self.is_pier_east_side` variable according to your physical setup.
-- **Example:** Set `pier_east_side = False` (the default) if the  camera is on the side of the Star Adventurer with the mode dial and lights, with your camera pointing in the opposite direction of the celestial pole. Set it to `True` if the lights are on the other side or you are pointing towards the celestial pole. If you make it wrong the mount will move in the opposite direction, so just try to flip it :)

You may also need to adjust `step_per_rev_dec` if your motor or gearing setup is different.

## 5. Advanced Topics & Explanation

### The RA Slew Limitation and the "Virtual Meridian Flip"

The advanced firmware for the Star Adventurer has a critical limitation: the RA motor cannot perform a full 360-degree slew. It operates reliably only within a ~180-degree range of motion in software counters. If a GoTo command requires it to move beyond this range (i.e., to cross a "virtual bar"), the command will fail, and the mount will get stuck.

This script implements a "hack" to overcome this. We call it a **Virtual Meridian Flip**.

Here's how it works:

1.  When your planetarium software (e.g., KStarsr) sends a GoTo command, our Python proxy receives the target's RA and DEC coordinates.
    
2.  The script calculates the target's Hour Angle (HA), which determines its position relative to the meridian.
    
3.  The script checks if slewing to this HA would force the RA motor to cross its "forbidden zone".
    
4.  If it does, the script performs a "virtual meridian flip":
    
    -   It calculates a new RA target that is 180 degrees away from the original target's HA. This new target is within the motor's "safe" operational range.
        
    -   It simultaneously inverts the target DEC coordinate.
        
    -   It sends these _new, corrected_ coordinates to the mount's motors.
        

The result is that the telescope ends up pointing at the correct celestial object, but with the mount on the opposite side of the pier, completely avoiding the firmware limitation. This entire process is transparent to your control software, which believes it's communicating with a standard, fully-functional LX200 mount.

### Networking the Star Adventurer (for non-2i models)

The original Star Adventurer communicates via a serial (USB) port. The Star Adventurer 2i has built-in WiFi. For this system to work, the Python proxy script needs to communicate with the mount over a network connection.

-   **Star Adventurer 2i users:** You can skip this step. Simply connect to your mount's WiFi network.
    
-   **Original Star Adventurer users:** You must make the mount's serial port available on the network. The recommended tool for this on Linux-based systems (like a Raspberry Pi) is `ser2net`.
    

`ser2net` is a daemon that allows you to connect to a serial port over a TCP or UDP network socket.

**Example `ser2net` Configuration:**

You need to create or edit the `/etc/ser2net.yaml` file. Here is a sample configuration that exposes a serial device on `UDP port 1880`, which is what the `synscan` library expects.

```
%YAML 1.1
---
# This is a ser2net configuration file.
# Find detailed documentation in ser2net.yaml(5)

define: &banner \r\nser2net port \p device \d [\B] (Debian GNU/Linux)\r\n\r\n

connection: &con0096
    accepter: udp,1880
    enable: on
    options:
      kickolduser: true
      telnet-brk-on-sync: false
    connector: serialdev,
              /dev/ttyUSB0,
              115200n81,local

```

**Breakdown of the configuration:**

-   `accepter: udp,1880`: Listens for incoming UDP connections on port 1880.
    
-   `connector: serialdev, /dev/ttyUSB0, 115200n81,local`: Forwards all traffic from the network port to the serial device located at `/dev/ttyUSB0` with a baud rate of 115200. **You must change `/dev/ttyUSB0`** to match the actual device name of your Star Adventurer's USB-to-Serial adapter.
    

After configuring `ser2net`, you can enable and start the service using `systemctl`:

```
sudo systemctl enable ser2net
sudo systemctl start ser2net

```

## 6. Running the Proxy Server

Once everything is configured, run the main script from your terminal:

```
python lx200.py

```

If successful, you will see a message indicating that the proxy is listening for connections:

LX200 Proxy listening on 127.0.0.1:5000

The server is now running. You can now connect to it from your planetarium or astrophotography software by configuring it to use a **Meade** LX200 **GPS** telescope at **IP Address `127.0.0.1`** and **Port `5000`**.

Once connected, you can select objects and issue GoTo commands, and the mount will slew to the target
