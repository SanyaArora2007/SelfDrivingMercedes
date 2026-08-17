# Self-Driving Mercedes

This self-driving LEGO Mercedes handles obstacle avoidance and collision detection.
The car is powered by A **BuWizz 3.0 Pro**, which is a brick that powers and controls
the car's drive and steering motors using Bluetooth. I have mounted a **Raspberry Pi**
connected to a **distance sensor** on the front of the car, with its primary goal of
detecting obstacles.

`DriveCar.py` is the program that runs the car. When started, the car calibrates its steering
using mechanical end stops. Then, using that calibration, the car cruises forward in a straight
line. As the car moves, the front sensor is constantly checking the distance to obstacles in the
front, and when there is an obstacle within its range, the car backs up and then turns a
random direction, which allows the car to avoid the obstacle. Since I have only mounted one sensor,
there is still a possibility of the car crashing, so there is the same logic applies when the
drive motor stalls (indicating a collision).

```
Raspberry Pi Zero 2 W (on the car)
   ├── I²C ───────────► VL53L0X distance sensor (front)
   └── Bluetooth LE ──► BuWizz 3.0 Pro ──► XL drive motor + L steering motor
```

---

## Repository layout

| File | What it is |
|---|---|
| `DriveCar.py` | Main program: Controlled through bluetooth, steering calibration, obstacle-avoidance |
| `sensor-test.py` | Continuously getting VL52L0X distance sensor readings |
| `steer_cal.json` | Steering calibration |

### Running it

On the Raspberry Pi use comman line arguments to run a specific file:

```bash
cd ~/SelfDrivingMercedes           # wherever the code lives on the Pi
source venv/bin/activate           # the venv with bleak + the sensor libs
sudo hciconfig hci0 up             # only if the Bluetooth adapter is "DOWN"
python3 DriveCar.py --duration 60  # 60-second obstacle-avoidance run
```

Useful command line arguments:

| Flag | Effect |
|---|---|
| `--duration SECONDS` | How long to drive (default 120 s = 2 min) |
| `--sensor-mm MM` | Distance that triggers avoidance (default 250 mm) |
| `--recalibrate` | Re-run the full steering calibration and exit |

---

## Lego Modifications

I have added three main things to this LEGO technic car:

- **Drive motor** — A LEGO **XL PoweredUp motor** that controls the movement of the car
- **Steering motor** — A LEGO **L PoweredUp motor** that controls the sterring
- **Electronics platform** — Mounting points for the **BuWizz brick**, the
  **Raspberry Pi**, and a **forward-facing distance sensor** at the front of the
  car using a laser to detect objects
  
One important thing about the steering motors is that it does **not** connect 1:1 to the
wheel. The motos has to turn approximetely 6 degreees for every degree the wheels move,
yielding a **6:1 ratio**. This is important because the moto's encoder measures the motors
shaft, not the wheels, which is a big distcintion when it comes to the software.

---

## BuWizz vs. other smart bricks — the control brick decision

I chose the **BuWizz 3.0 Pro** over othersmart-brick optionsbecause of what this project
needed:

- **PoweredUp motor support with encoders.** The BuWizz reads each PoweredUp
  motor's built-in rotation encoder and reports position/velocity back over
  Bluetooth. This is what makes closed-loop steering (drive to a specific angle)
  and stall-based obstacle detection possible at all.
- **On-board position/speed servo control.** The brick has a built-in PID
  controller per port, so I can command "steer to X degrees" and the brick
  holds it at that speciic degree.
- **A documented BLE API.** This allows me to use Python to control the BuWizz using `bleak`.
- **Plenty of current** for the drive and steering motors.

---

## Steering and driving — two motors, two control modes

Both motos are plugges into seperate BuWizz pPowerUp ports, anf they are dirven in completely
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

The steering motor runs in the Buwizz's **position-servo mode** and you can give it
a specific angle in degreed, and its interned PID (Proportional Integral Derivative) drives
the wheels there and holds.
Two thing to consider:

1. **The angle is in *motor* degrees, not wheel degrees.** Because of the ~6:1
   steering gearing, "45°" at the motor is only ~7–8° at the  wheels. The
   encoder can only see the motor shaft.
2. **The encoder's zero is arbitrary each session** — it resets when the motor
   powers up, so the same "0°" doesn't mean "wheels straight" from run to run.

At the begining the car has to **calibrate its own steering** so it knows which direction
is straight. The calibrations works by:

- Running the servo to each mechaincal end stop (all the way left and all the way right)
- Using the two end stops it records the **center** (midpoint between the two) and the
  **half-lock span** (how far full lock is from center, in motor degrees which is about
  ±137° on this build).
---

## Distance sensor

The **VL53L0X laser distacne sensor** is a small chip that measure distance by timing a relfected
infrared laser pulse. It reads around **5 cm when something is touching it**, and its maximum disatice is around **80 cm**.

In `DriveCar.py`, while driving forward the sensor is read every control tick.
If it sees an obstacle within **`SENSOR_TRIGGER_MM`** (default **250 mm** ≈ 25 cm),
the car triggers the *same* reverse-and-turn recovery as a physical hit, but
**before** actually touching the obstacle.

`sensor-test.py` is a standalone script that just prints the live distance, which was useful
to test if the sensor was working correctly (all the hardware was correctly in place).

---

## Soldering

The distacne sensor is connected to the Raspbery Pi using fine wires. To complete the cricuit
and all the connections I has to solder the sesnro to the head (cotaining pins for the wires to attach). This was essential for the I²C to work

I tried very hard to use XSHUT, so that multiple sensors could be used on the same I²C bus. But either due to bad soldering or a bad part I couldn't get it to work.

---

## Raspberry Pi

A **Raspberry Pi** rides on the car and is the actual "self-driving" computer. It
does two jobs at once:

- **Reads the distance sensor** over its I²C bus.
- **Controls the BuWizz** over Bluetooth LE, running `DriveCar.py`.

The only way to run the sensor-based avoidance script is to run it **on the Pi** because
the sensor is physically wired to the Pi'2 GPIO. I can still run the same script using my
laptop, however, the car will run using collision detections insetad of obstacle avoidance.

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

I found pin 1 by probing the powered Raspberry Pi's header with a multimeter until the corner pin read 3.3V. I knew 3.3V was important because pin 1 is always defined as a 3.3V power oin on the Pi's layout.

---

## Future: multiple sensors with an I²C multiplexer (TCA9548A)

Right now the car drives using **one** forward facing sensor. In the future I want to mount multiple sensors in different directions (front, back, side etc.) For this to be possible, I need multiple VL53L0X sensors. I tried the XSHUT methos to constantly turn on and off each sensor and cycle between them to get reading from all sensor. Since this didn't work, next time I would like to try and use an I²C multiplexer, so multiple sensors can be read together.

## Future: camera that detects my dog

Every time I test my self-driving car my dog always runs away from it. I would like to try
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
