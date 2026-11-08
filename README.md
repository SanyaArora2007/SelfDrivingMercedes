# Self-Driving Mercedes

<img width="512" height="289" alt="FullCarPicture" src="https://github.com/user-attachments/assets/a09e6237-1174-4de6-a0fb-83b8f9404a5e" />

This self-driving Mercedes handles obstacle avoidance and collision detection.
The car is powered by A **BuWizz 3.0 Pro**, which is a brick that powers and controls
the car's drive and steering motors using Bluetooth. I have mounted a **Raspberry Pi**
connected to a **distance sensor** on the front of the car, with its primary goal of
detecting obstacles. Built on a LEGO platform, this Mercedes has several custom aftermarket mods
to accommodate the motors and their connections to the steering and drivetrain.

`DriveCar.py` is the program that runs the car. When started, the car calibrates its steering to find
the center point between mechanical end stops. Then, using that calibration, the car cruises forward in a straight
line. As the car moves, the front sensor checks the distance to obstacles in the
front at 20Hz. When there is an obstacle within a specified range, the car backs up and then turns in a
random direction, which allows it to avoid the obstacle. Since I have only mounted one sensor,
there is still a possibility of the car crashing, so the same logic is applied when the
drive motor stalls, indicating a collision.

```
Raspberry Pi Zero 2 W (on the car)
   ├── I²C ───────────► VL53L0X distance sensor (front)
   └── Bluetooth LE ──► BuWizz 3.0 Pro ──► XL drive motor + L steering motor
```

Click [here](https://youtu.be/4X625ZH-nY0) to see a video.

---

## Repository layout

| File | What it is |
|---|---|
| `DriveCar.py` | Main program: Controlled through Bluetooth, steering calibration, obstacle avoidance |
| `DriveServer.py` | Web server: starts the same obstacle-avoidance run over HTTP (`POST /run`, `POST /recenter`), and drives the car by hand from the web page |
| `car_control.py` | Shared library used by `DriveCar.py` and `DriveServer.py` |
| `manual_control.py` | Remote-control driving for `DriveServer.py`: speed and steering levels, safety stops |
| `web/index.html` | Web page served by `DriveServer.py` at `/`: remote-control buttons, live telemetry and the car's log |
| `sensor-test.py` | Continuously getting VL52L0X distance sensor readings |
| `steer_cal.json` | Steering calibration |
| `requirements.txt` | Python packages to install into the venv |
| `restart.sh` | Starts (or restarts) `DriveServer.py` in the background on the Pi; also run at boot |

### Setup

On the Raspberry Pi, create the virtual environment inside the project folder:

```bash
cd ~/SelfDrivingMercedes
sudo apt install python3-lgpio python3-rpi-lgpio python3-libgpiod   # usually preinstalled on Raspberry Pi OS
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

`--system-site-packages` is required: the sensor library talks to the Pi's GPIO
through the Debian `python3-*gpio` packages installed by `apt`, which a plain venv
can't see. A venv also can't be moved or renamed once created (its scripts store
their absolute path), so if the folder moves, delete `venv/` and run these steps again.

On a laptop the same steps work without the `apt` line; the sensor library is
skipped and the car uses stall detection only.

### Running it

On the Raspberry Pi, use command-line arguments to run a specific file:

```bash
cd ~/SelfDrivingMercedes           # wherever the code lives on the Pi
source venv/bin/activate           # the venv with bleak + the sensor libs
sudo hciconfig hci0 up             # only if the Bluetooth adapter is "DOWN"
python3 DriveCar.py --duration 60  # 60-second obstacle-avoidance run
```

Useful command-line arguments:

| Flag | Effect |
|---|---|
| `--duration SECONDS` | How long to drive (default 120 s = 2 min) |
| `--sensor-mm MM` | Distance that triggers avoidance (default 250 mm) |
| `--recalibrate` | Re-run the full steering calibration and exit |

To drive the car from a browser instead, run the web server with `restart.sh`. It stops the
server if it's already running, then starts it in the background, logging to `server.log`;
the page is at `http://<pi-address>:8000/`. To start it whenever the Pi boots, add this
line with `crontab -e`:

```
@reboot /home/sanyaarora/SelfDrivingMercedes/restart.sh
```

---

## LEGO Modifications

I have added three main things to this LEGO Technic car:

- **Drive motor** — A LEGO **XL PoweredUp motor** that controls the movement of the car
- **Steering motor** — A LEGO **L PoweredUp motor** that controls the sterring
- **Electronics platform** — Mounting points for the **BuWizz brick**, the
  **Raspberry Pi**, and a **forward-facing distance sensor** at the front of the
  car using a laser to detect objects
  
One important thing about the steering motors is that it does **not** connect 1:1 to the
wheel. The motor has to turn approximately 6 degrees for every degree the wheels move,
yielding a **6:1 ratio**. This is important because the motor's encoder measures the motor's
shaft, not the wheels, which is a big distinction when it comes to the software.

---

## BuWizz vs. other smart bricks

I chose the **BuWizz 3.0 Pro** over other smart-brick options because of what this project
needed:

- **PoweredUp motor support with encoders.** The BuWizz reads each PoweredUp
  motor's built-in rotation encoder and reports position/velocity back over
  Bluetooth. This is what makes closed-loop steering (drive to a specific angle)
  and stall-based obstacle detection possible at all.
- **On-board position/speed servo control.** The brick has a built-in PID
  controller per port, so I can command "steer to X degrees" and the brick
  holds it at that specific degree.
- **A documented BLE API.** This allows me to use Python to control the BuWizz using `bleak`.
- **Plenty of current** for the drive and steering motors.

---

## Steering and driving

Both motors are plugged into separate BuWizz PowerUp ports, and they are driven in completely
different ways:

| | Drive motor (XL) | Steering motor (L) |
|---|---|---|
| Port | PU port 1 | PU port 2 |
| Mode | Simple PWM (speed) | **Position servo** (angle) |
| Command | "go this fast" | "go to this angle" |

Everything talks to the brick using Bluetooth LE with the [`bleak`](https://github.com/hbldh/bleak)
Python library, using the BuWizz 3.0 command set.

---

## Steering with degrees and calibration

The steering motor runs in the Buwizz's **position-servo mode**, and you can give it
a specific angle in degrees, and its internal PID (Proportional Integral Derivative) drives
the wheels there and holds.
Two things to consider:

1. **The angle is in *motor* degrees, not wheel degrees.** Because of the ~6:1
   steering gearing, "45 degrees" at the motor is only ~7-8 degrees at the  wheels. The
   encoder can only see the motor shaft.
2. **The encoder's zero is arbitrary each session** — it resets when the motor
   powers up, so the same "0 degrees" doesn't mean "wheels straight" from run to run.

At the beginning, the car has to **calibrate its own steering** so it knows which direction
is straight. The calibration works by:

- Running the servo to each mechanical end stop (all the way left and all the way right)
- Using the two end stops, it records the **center** (midpoint between the two) and the
  **half-lock span** (how far full lock is from center, in motor degrees, which is about
  +/-137 degrees on this car).

The steering motor also reports an **absolute angle** (-180 to 179 degrees) that does *not*
reset at power-up. Because the whole steering range is less than one full turn of the
motor, each absolute angle matches exactly one steering position. So the calibration also
saves the center's absolute angle in `steer_cal.json`, and after that the car finds
straight-ahead just by reading the motor, without driving the wheels into the end stops.
The full calibration only runs again when there's no saved center, when the reading
doesn't make sense, or when you ask for it (`POST /recenter`, the page's **Recenter
steering** button, or `DriveCar.py --recalibrate`).

---

## Distance sensor

<img width="403" height="227" alt="DistanceSensor" src="https://github.com/user-attachments/assets/878e1417-0608-4d66-b931-889980dd6d92" />

The **VL53L0X laser distance sensor** is a small chip that measures distance by timing a reflected
infrared laser pulse. It reads around **5 cm when something is touching it**, and its maximum distance is around **80 cm**.

In `DriveCar.py`, while driving forward, the sensor is read every control tick.
If it sees an obstacle within **`SENSOR_TRIGGER_MM`** (default **250 mm** ≈ 25 cm),
the car triggers the *same* reverse-and-turn recovery as a physical hit, but
**before** actually touching the obstacle.

`sensor-test.py` is a standalone script that prints the live distance, which was useful
to test if the sensor was working correctly (all the hardware was correctly in place).

---

## Soldering

The distacne sensor is connected to the Raspberry Pi using fine wires. To complete the circuit
and all the connections, I had to solder the sensor to the head (containing pins for the wires to attach).
This was essential for the I²C to work

I tried hard to use XSHUT so that multiple sensors could be used on the same I²C bus. But either due to
bad soldering or a bad part, I couldn't get it to work.

---

## Raspberry Pi

<img width="403" height="227" alt="RaspberryPi" src="https://github.com/user-attachments/assets/dfd49956-c2ec-4539-bd1f-3f7fadb2f2b1" />


A **Raspberry Pi** rides on the car and is the actual "self-driving" computer. It
does two jobs at once:

- **Reads the distance sensor** over its I²C bus.
- **Controls the BuWizz** over Bluetooth LE, running `DriveCar.py`.

The only way to run the sensor-based avoidance script is to run it **on the Pi** because
the sensor is physically wired to the Pi's GPIO. I can still run the same script using my
laptop; however, the car will run using collision detections instead of obstacle avoidance.

---

## I²C

**I²C** is the two-wire bus (a data line **SDA** and a clock line **SCL**, plus
power and ground) that the Pi uses to talk to the distance sensor.

The VL53L0X sensor is wired to the Raspberry Pi's 40-pin header like this:

| VL53L0X pin | Raspberry Pi pin | Function |
|---|---|---|
| **VCC** / VIN | Pin 1 — **3.3 V** | Power |
| **GND** | Pin 6 — **GND** | Ground |
| **SDA** | Pin 3 — **GPIO2** | I²C data |
| **SCL** | Pin 5 — **GPIO3** | I²C clock |
| **XSHUT** | Pin 11 — **GPIO17** *(optional)* | Shutdown/enable — only needed to run more than one sensor |

With a single sensor, XSHUT can be left unconnected — the sensor comes up at its
default I²C address `0x29`.

I found pin 1 by probing the powered Raspberry Pi's header with a multimeter until the corner pin read 3.3V. I knew 3.3V was important because pin 1 is always defined as a 3.3V power pin on the Pi's layout.

---

## Future: multiple sensors with an I²C multiplexer (TCA9548A)

Right now the car drives using **one** forward-facing sensor. In the future, I want to mount multiple sensors in different directions (front, back, side, etc.) For this to be possible, I need multiple VL53L0X sensors. I tried the XSHUT method to constantly turn each sensor on and off and cycle between them to get readings from all sensors. Since this didn't work, next time I would like to try and use an I²C multiplexer, so multiple sensors can be read together.

## Future: camera that detects my dog

Every time I test my self-driving car, my dog always runs away from it. I would like to try
and add a camera that uses an OpenCV model (compatible with this Raspberry Pi) to detect and
avoid my scared dog. Turns out using a VLM (Vision Language Model) is not possible on this
Raspberry Pi because of limited memory.

---

## References

- **Raspberry Pi Zero 2 W** — https://www.amazon.com/dp/B0DRRDJKDV?ref_=ppx_hzod_title_dt_b_fed_asin_title_1_1
- **VL53L0X distance sensor** — https://www.amazon.com/dp/B0DP6893DS?ref_=ppx_hzod_title_dt_b_fed_asin_title_0_0&th=1
- **LEGO model** — Otrans' *Mercedes-Benz G500 Professional Line — RC mod for Powered Up
  motors + LED light* (MOC-203945), published on Rebrickable:
  <https://rebrickable.com/mocs/MOC-203945/otrans/42177-mercedes-benz-g500-professional-line-rc-mod-for-powered-up-motors-led-light/#parts>
- **LEGO parts** — sourced from [BrickOwl](https://www.brickowl.com/) and
  [ToyPro](https://www.toypro.com/). Rebrickable offers an easy way to differentiate parts
  between multiple LEGO sets, so you know exactly what to buy.
- **BuWizz 3.0 Pro** — <https://buwizz.com/shop/buwizz-3-0-pro/>
