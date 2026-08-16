# Self-Driving Mercedes

An autonomous, obstacle-avoiding LEGO Mercedes. A **BuWizz 3.0 Pro** smart brick
powers the car's drive and steering motors over Bluetooth; a **Raspberry Pi**
mounted on the car reads a forward-facing **distance sensor** and steers the car
around obstacles on its own — all in Python (`DriveCar.py`).

On start-up the car calibrates its steering to the mechanical end stops, then
cruises forward. When the front sensor sees something within range (or the drive
motor stalls against something the sensor missed), it backs up, turns a random
direction, and drives around — repeating for the length of the run.

```
Raspberry Pi (on the car)
   ├── I²C ───────────► VL53L0X distance sensor (front)
   └── Bluetooth LE ──► BuWizz 3.0 Pro ──► XL drive motor + L steering motor
```

---

## Repository layout

| File | What it is |
|---|---|
| `DriveCar.py` | Main program: BLE control, steering calibration, obstacle-avoidance mission |
| `sensor-test.py` | Standalone continuous read-out of the VL53L0X distance sensor |
| `steer_cal.json` | Cached steering calibration (lock-to-lock travel) |

### Running it

On the Raspberry Pi (which is mounted on the car and does both the sensing and
the driving):

```bash
cd ~/SelfDrivingMercedes           # wherever the code lives on the Pi
source venv/bin/activate           # the venv with bleak + the sensor libs
sudo hciconfig hci0 up             # only if the Bluetooth adapter is "DOWN"
python3 DriveCar.py --duration 60  # 60-second obstacle-avoidance run
```

Useful flags:

| Flag | Effect |
|---|---|
| `--duration SECONDS` | How long to drive (default 120 s = 2 min) |
| `--sensor-mm MM` | Distance that triggers avoidance (default 250 mm) |
| `--no-sensor` | Ignore the sensor, use only motor-stall detection |
| `--recalibrate` | Re-run the full steering calibration and exit |
| `--selftest` | Fixed drive + steer sweep instead of the mission |

---

## Lego Mods

The car is a LEGO Technic car body reworked so it can drive itself. Three things
had to be added or adapted to the stock build:

- **Drive motor** — a LEGO **XL PoweredUp motor** geared into the rear
  drivetrain to move the car.
- **Steering motor** — a LEGO **L PoweredUp motor** geared into the steering
  rack so the software can turn the front wheels.
- **Electronics platform** — mounting points for the **BuWizz brick**, the
  **Raspberry Pi**, and a **forward-facing distance sensor** at the front of the
  car with a clear line of sight ahead.

The steering motor does **not** connect 1:1 to the wheels — there is reduction
gearing between them (roughly **6:1**), so the motor has to turn about six
degrees for every degree the road wheels move. This gearing matters a lot for
the software (see *Steering with degrees and calibration*), because the motor's
encoder measures the *motor* shaft, not the wheels.

---

## BuWizz vs. other smart bricks — the control brick decision

The "brain-to-motor" layer is the smart brick that receives commands over
Bluetooth and drives the LEGO motors. I chose the **BuWizz 3.0 Pro** over other
smart-brick options because of what this project needed:

- **PoweredUp motor support with encoders.** The BuWizz reads each PoweredUp
  motor's built-in rotation encoder and reports position/velocity back over
  Bluetooth. This is what makes closed-loop steering (drive to a specific angle)
  and stall-based obstacle detection possible at all.
- **On-board position/speed servo control.** The brick has a built-in PID
  controller per port, so I can command "steer to X degrees" and the brick
  holds it — no servo loop needed on my side.
- **A documented BLE API.** This allows me to use Python to control the BuWizz using `bleak`.
- **Plenty of current** for the drive and steering motors, with configurable
  per-port current limits.

The trade-off: the BuWizz is a closed, connect-one-host-at-a-time BLE device, so
only one computer (the Pi) can control it at a time, and everything goes through
its command protocol rather than raw motor pins.

---

## Steering and driving — two motors, two control modes

Both motors plug into the BuWizz's PoweredUp ports, but they are driven in
completely different ways:

| | Drive motor (XL) | Steering motor (L) |
|---|---|---|
| Port | PU port 1 | PU port 2 |
| Mode | Simple PWM (speed) | **Position servo** (angle) |
| Command | "go this fast" | "go to this angle" |

Everything talks to the brick over Bluetooth LE with the [`bleak`](https://github.com/hbldh/bleak)
Python library, using the BuWizz 3.0 command set.

---

## Steering with degrees and calibration

The steering motor runs in the BuWizz's **position-servo mode**: you give it a
target angle in degrees and its internal PID drives the wheels there and holds.
But there are two catches:

1. **The angle is in *motor* degrees, not wheel degrees.** Because of the ~6:1
   steering gearing, "45°" at the motor is only ~7–8° at the road wheels. The
   encoder can only see the motor shaft.
2. **The encoder's zero is arbitrary each session** — it resets when the motor
   powers up, so the same "0°" doesn't mean "wheels straight" from run to run.

To make steering meaningful, the car **calibrates its own steering range** on
start-up:

- It ramps the servo target outward in small steps and watches the encoder. While
  the wheels can follow, each step advances the shaft; once the wheels hit a
  mechanical **end stop**, the shaft stops advancing. That's how it finds full
  lock on each side, without needing to know the gear ratio.
- From the two end stops it records the **center** (midpoint of travel) and the
  **half-lock span** (how far full lock is from center, in motor degrees — about
  ±137° on this build).
---

## Driving with PoweredUp

The drive motor runs in **simple PWM mode**: the BuWizz applies a duty cycle
(−127…+127) to the motor, and reports the motor's **velocity** back in the status
stream. The car uses that velocity two ways:

- **Speed control** — cruising forward at a fixed PWM, backing up at reverse PWM,
  and a slightly higher PWM while turning around an obstacle.
- **Stall = obstacle** — if the car is commanding forward power but the drive
  velocity stays near zero (after a brief spin-up grace period), it has physically
  run into something. This is the fallback obstacle detector for anything the
  distance sensor's beam misses (low objects, glancing hits).

The obstacle-avoidance behavior itself: cruise forward (capped at ~10 s of
straight driving before a gentle random turn, so it doesn't beeline into walls
or out of Bluetooth range); on an obstacle, reverse, stop, turn the wheels a
**random** left/right, drive around, straighten, and resume. Turns are done
*while moving* for a smooth arc, and the recovery is symmetric — if it bumps
something while backing up, it does the same maneuver forward instead.

---

## Distance sensor

The forward-looking sensor is a **VL53L0X time-of-flight (ToF) laser distance
sensor** — a small chip that measures distance by timing a reflected infrared
laser pulse. It reads roughly **5 cm when something is touching it** up to about
**80 cm** at the far end, over I²C.

In `DriveCar.py`, while driving forward the sensor is read every control tick.
If it sees an obstacle within **`SENSOR_TRIGGER_MM`** (default **250 mm** ≈ 25 cm),
the car triggers the *same* reverse-and-turn recovery as a physical hit — but
**before** actually touching the obstacle.

`sensor-test.py` is a standalone script that just prints the live distance, handy
for aiming/mounting the sensor and confirming it works.

---

## Soldering

Getting the sensor onto the car meant some soldering and a lot of connection
debugging — the sensor breakouts connect to the Pi with fine wires, and every
joint has to be solid for I²C to work.

Hard-won lessons from wiring the sensors: I tried very hard to use XSHUT, so
that multiple sensors could be used on the same I²C bus. But either due to bad
soldering or a bad part I couldn't get it to work.

---

## Raspberry Pi

A **Raspberry Pi** rides on the car and is the actual "self-driving" computer. It
does two jobs at once:

- **Reads the distance sensor** over its I²C bus.
- **Controls the BuWizz** over Bluetooth LE (BlueZ), running `DriveCar.py`.

Because the sensor is physically wired to the Pi's GPIO, the driving program has
to run **on the Pi** for sensor-based avoidance to work. Running the same script
on a laptop still works — it just falls back to stall-only detection, since the
sensor libraries/hardware aren't there.

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

---

## Future: multiple sensors with an I²C multiplexer (TCA9548A)

Right now the car drives on **one** forward sensor. To sense in more directions
(front + sides, or a wider forward arc), it needs multiple VL53L0X sensors. I tried
using the XSHUT method to constantly turn on and off each sensor and cycle between them
to get readings from all sensors. This didn't work because of either soldering issues, or
a bad part. Next, I would like to use an I²C multiplexer, so multiple sensors can be read
together. This would allow me to use different types of sensors, like, color, sound, or 
motion.

## Future: camera that detects my dog

Every time I test my self-driving car my dog always runs away from it. I would like to try
and add a camera that uses an OpenCV model (compatible with this Raspberry Pi) to detect and
avoid my sacred dog. Turns out using a VLM (Vision Language Model) is not possible on this
Raspberry Pi because of limited memory. 
