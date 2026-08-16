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
- **A documented BLE API.** BuWizz publishes the exact packet format (motor
  commands, status reports, watchdog, port configuration), so everything could
  be driven directly from Python instead of reverse-engineered.
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
Python library, using the BuWizz 3.0 command set (e.g. `0x30`/`0x31` set motor,
`0x50` set port function, `0x52` set servo reference, `0x35` watchdog).

Two protocol lessons that were essential to get right:

- **The watchdog is fed by its own command, not by motor writes.** The BuWizz
  drops the connection if it doesn't receive a watchdog (`0x35`) command in time.
  Sending motor commands does *not* reset it — you must re-send `0x35`. Missing
  this made the car disconnect a second into every drive.
- **BLE writes here are unacknowledged**, so a single dropped packet can strand a
  command. Motor and steering targets are re-sent every control tick so a lost
  packet is harmless.
- **Driving and steering share one packet.** Command `0x31` sets every port in a
  single message (PWM for the drive port, degrees for the steering servo), so the
  two never overwrite each other — an earlier version that sent them as two
  separate packets had them zeroing each other out and starving the drive motor.

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
- Steering is then commanded as a **fraction of full lock** (`-1.0` = full left,
  `+1.0` = full right, `0.0` = center), with a small safety margin so it never
  slams the hard stops. The code never has to think in raw motor degrees.

**Caching:** the *span* (half-lock) is a fixed physical property of the build, so
it's saved to `steer_cal.json` and reused. The *center*, however, depends on the
per-session encoder zero, so each normal start does a quick **re-home** — it taps
just one end stop and derives center from the cached span (about half the time of
a full two-stop calibration). `--recalibrate` forces a fresh full calibration and
re-saves the span (use it if you rebuild the steering).

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
laser pulse. It reads roughly **5 mm when something is touching it** up to about
**8 m** at the far end, over I²C.

In `DriveCar.py`, while driving forward the sensor is read every control tick.
If it sees an obstacle within **`SENSOR_TRIGGER_MM`** (default **250 mm** ≈ 25 cm),
the car triggers the *same* reverse-and-turn recovery as a physical hit — but
**before** actually touching the obstacle. That's the difference between the old
"bump and recover" behavior and true **avoidance**:

- 250 mm gives the car room to stop and turn before contact.
- Lower it (e.g. `--sensor-mm 100`) to react later/closer; raise it to react
  sooner.
- The velocity-stall detector stays active underneath as a safety net.

`sensor-test.py` is a standalone script that just prints the live distance, handy
for aiming/mounting the sensor and confirming it works.

---

## Soldering

Getting the sensor onto the car meant some soldering and a lot of connection
debugging — the sensor breakouts connect to the Pi with fine wires, and every
joint has to be solid for I²C to work.

Hard-won lessons from wiring the sensors:

- **A distance sensor with power but a bad ground or data line looks "powered
  but silent."** The chip's XSHUT/enable pin pulls up to VCC regardless of
  ground, so it can appear alive while never answering on the bus. A cold solder
  joint on **GND, SDA, or SCL** produces exactly this — the fix is at the joint,
  not the code.
- **A truly dead/defective sensor can *jam* the whole I²C bus** — it holds a bus
  line during transactions so *every* device (including good ones) goes
  unreachable and the bus "hangs." If a scan hangs, suspect a bad sensor holding
  the bus.
- **Isolate faults with swaps.** Moving a wire between two header pins, or moving
  a board between two harnesses, tells you whether a fault follows the *wire*, the
  *pin*, or the *board* — much faster than staring at solder joints.
- A defective chip can also **wedge the Pi's I²C controller** after a bus hang; a
  **reboot** clears it.

The upshot: connections were validated end-to-end with a known-good sensor before
trusting any result, and each suspect part (wire, pin, harness, board) was ruled
out one at a time.

---

## Raspberry Pi

A **Raspberry Pi** rides on the car and is the actual "self-driving" computer. It
does two jobs at once:

- **Reads the distance sensor** over its I²C bus.
- **Controls the BuWizz** over Bluetooth LE (BlueZ), running `DriveCar.py`.

Because the sensor is physically wired to the Pi's GPIO, the driving program has
to run **on the Pi** for sensor-based avoidance to work. Running the same script
on a laptop still works — it just falls back to stall-only detection, since the
sensor libraries/hardware aren't there (the imports are optional and degrade
gracefully).

Practical Pi notes:

- The Python dependencies live in a **virtualenv** (`bleak` for Bluetooth,
  `adafruit-circuitpython-vl53l0x` + `board`/`busio` for the sensor). Activate it
  before running: `source venv/bin/activate`.
- The **Bluetooth adapter can come up `DOWN`** after a reboot; bring it up with
  `sudo hciconfig hci0 up` before the first run.
- One host at a time: while the Pi is connected to the BuWizz, nothing else can
  connect to the brick.
- Range: a stationary Pi + a car that drives away will eventually lose the BLE
  link. The 10-second straight-driving cap helps keep the car from wandering out
  of range.

---

## I²C

**I²C** is the two-wire bus (a data line **SDA** and a clock line **SCL**, plus
power and ground) that the Pi uses to talk to the distance sensor. Each device on
the bus has a 7-bit **address**; the Pi is the controller and the sensor is a
peripheral.

On this build:

- SDA → Pi **pin 3 (GPIO2)**, SCL → Pi **pin 5 (GPIO3)**, on I²C bus `i2c-1`.
- The VL53L0X answers at its default address **`0x29`**.
- You can see it with a bus scan (`i2c.scan()` in Python, or `i2cdetect`), which
  is the first thing to check when a sensor "isn't working": if it shows up at
  `0x29`, it's alive on the bus; if the bus is empty, it's a wiring/power problem;
  if the scan *hangs*, a device is jamming the bus.

The catch that shapes the whole multi-sensor story below: **every VL53L0X powers
up at the same address, `0x29`.** One sensor is no problem. Two on the same bus
**collide** — you can't address them independently as-is.

The standard workaround (and what the code is set up for) uses each sensor's
**XSHUT** (shutdown/enable) pin: bring the sensors up one at a time, and use the
sensor library's `set_address()` to move the first one to a new address (e.g.
`0x30`) before enabling the next. Note that this reassignment is **volatile** — it
resets to `0x29` whenever the sensor loses power.

---

## Future: multiple sensors with an I²C multiplexer (TCA9548A)

Right now the car drives on **one** forward sensor. To sense in more directions
(front + sides, or a wider forward arc), it needs multiple VL53L0X sensors — and
that runs straight into the address collision above.

The XSHUT trick works for two sensors but gets fragile fast (every sensor needs
its own GPIO enable line, the address reassignment is lost on power loss, and one
flaky enable line breaks the sequence). The clean solution is an **I²C
multiplexer**:

- **TCA9548A** — an 8-channel I²C switch. All the sensors keep their identical
  `0x29` address, but each one lives on its **own downstream channel** of the mux.
  The Pi talks to the mux (at its own address, typically `0x70`), selects a
  channel, then talks to "the `0x29` on that channel."
- **Why it's better here:** no XSHUT wiring, no address juggling, no volatile
  reassignment to redo on every power-up. Adding a sensor is just "plug it into
  the next channel." Up to 8 sensors on one mux (and multiple muxes if ever
  needed).
- **Software change:** read a sensor by (1) writing the channel-select byte to the
  TCA9548A, then (2) reading the VL53L0X at `0x29` as usual — wrap that in a small
  helper and the rest of the avoidance logic stays the same, just fed by whichever
  direction is most relevant.

That's the planned path from a single-sensor "avoid what's dead ahead" car to a
multi-sensor one that can pick gaps and steer toward open space.
