# Self-Driving Mercedes

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

---

## Repository layout

| File | What it is |
|---|---|
| `DriveCar.py` | Main program: Controlled through Bluetooth, steering calibration, obstacle avoidance |
| `sensor-test.py` | Continuously getting VL52L0X distance sensor readings |
| `steer_cal.json` | Steering calibration |

### Running it

On the Raspberry Pi, use command-line arguments to run a specific file:

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
---

## Distance sensor

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

I tried hard to use XSHUT, so that multiple sensors could be used on the same I²C bus. But either due to
bad soldering or a bad part, I couldn't get it to work.

---

## Raspberry Pi

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
