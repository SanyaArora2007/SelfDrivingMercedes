"""
telemetry.py  -  Self-Driving Mercedes: telemetry log

Appends events to telemetry.log next to the scripts, and hands each line to a
callback (DriveServer.py streams them to the web page). The second column says
which kind of event a line is:

    CONTROL  something the user did, e.g. a button on the page's control pad,
             with the speed and steering levels before and after
    STATUS   what the car is actually doing, as measured: the drive speed
             (and current) and the steering angle whenever either changes,
             the front distance while the car is moving, STALL when drive
             power is on but the motor isn't turning, and SAFETY_STOP when the
             car stops itself under manual control (see manual_control.py)

    2026-11-08 16:02:11.123  CONTROL  FORWARD_LEFT  speed +1 -> +2, steer 0 -> -1
    2026-11-08 16:02:11.180  STATUS   SPEED         vel +28 (moving), drive 0.84 A
    2026-11-08 16:02:11.230  STATUS   STEERING      -61 deg from straight (-45% lock)
    2026-11-08 16:02:11.380  STATUS   DISTANCE      412 mm
    2026-11-08 16:02:13.020  STATUS   SAFETY_STOP   front sensor 231 mm; speed +2 -> 0
    2026-11-08 16:02:20.410  STATUS   STALL         pwm +92 but vel 0 for 0.4 s; drive 2.31 A
"""

import asyncio
import os
import time

import car_control
from car_control import DRIVE_PORT, STALL_VEL, STEER_PORT

LOG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "telemetry.log")

SAMPLE_HZ = 20          # how often the status monitor looks at the car
DISTANCE_PERIOD = 0.2   # log the front distance this often while moving (s)
# Measured values jitter by a degree or two between reports, so a change only
# counts once it's at least this big (steering: motor degrees; speed: the
# brick's raw velocity units).
STEER_STEP = 3
SPEED_STEP = 3
OUT_OF_RANGE_MM = 8190  # the VL53L0X reads ~8190+ when nothing is in range


def _signed(n):
    """A whole number with its sign, but plain 0: +28, 0, -61."""
    n = round(n)
    return f"{n:+d}" if n else "0"


def _port(key, port):
    v = car_control.latest_status.get(key, [])
    return v[port - 1] if len(v) >= port else None


class Telemetry:
    def __init__(self, path=LOG_FILE, publish=None):
        self.file = open(path, "a", buffering=1)    # line-buffered
        self.publish = publish

    def _write(self, kind, event, detail):
        now = time.time()
        stamp = (time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(now))
                 + f".{int(now % 1 * 1000):03d}")
        line = f"{stamp}  {kind:<7}  {event:<12}  {detail}".rstrip()
        self.file.write(line + "\n")
        if self.publish is not None:
            self.publish(line)

    def control(self, event, detail=""):
        self._write("CONTROL", event, detail)

    def status(self, event, detail=""):
        self._write("STATUS", event, detail)

    async def monitor(self, active, watch_stalls):
        """Log STATUS events for as long as `active()` says the car's readings
        mean something (connected, homed and not homing). STALL events are only
        looked for while `watch_stalls()` -- the manual driver watches for
        stalls itself, as it also stops the car."""
        last_vel = last_steer = None
        last_distance = 0.0
        stall = car_control.StallDetector()
        while True:
            await asyncio.sleep(1 / SAMPLE_HZ)
            if not active():
                last_vel = last_steer = None       # report afresh when back
                continue
            vel = _port("vel", DRIVE_PORT)
            pos = _port("pos", STEER_PORT)
            moving = vel is not None and abs(vel) >= STALL_VEL

            if vel is not None and (
                    last_vel is None or abs(vel - last_vel) >= SPEED_STEP
                    or moving != (abs(last_vel) >= STALL_VEL)):
                self.status("SPEED", f"vel {_signed(vel)} "
                                     f"({'moving' if moving else 'stopped'})"
                                     + _current_detail())
                last_vel = vel

            if not watch_stalls():
                stall = car_control.StallDetector()
            elif stall.update(car_control.drive_command["pwm"], vel):
                self.status("STALL", stall_detail(car_control.drive_command["pwm"], vel))

            if pos is not None and (last_steer is None
                                    or abs(pos - last_steer) >= STEER_STEP):
                self.status("STEERING", _steer_detail(pos))
                last_steer = pos

            now = time.monotonic()
            if moving and now - last_distance >= DISTANCE_PERIOD:
                self.status("DISTANCE", _distance_detail())
                last_distance = now


def stall_detail(pwm, vel):
    return (f"pwm {_signed(pwm)} but vel {_signed(vel)} for "
            f"{car_control.STALL_TIME:g} s" + _current_detail("; "))


def _current_detail(sep=", "):
    amps = car_control.drive_current()
    return "" if amps is None else f"{sep}drive {amps:.2f} A"


def _steer_detail(pos):
    cal = car_control.steer_cal
    off = pos - cal["center"]
    lock = cal["lock_right"] if off >= 0 else cal["lock_left"]
    frac = f" ({_signed(100 * off / lock)}% lock)" if lock else ""
    return f"{_signed(off)} deg from straight{frac}"


def _distance_detail():
    if car_control.distance_sensor is None:
        return "no sensor"
    mm = car_control.recent_distance()
    if mm is None:
        return "no reading"
    if mm >= OUT_OF_RANGE_MM:
        return f"{mm} mm (nothing in range)"
    return f"{mm} mm"
