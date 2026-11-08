"""
manual_control.py  -  Self-Driving Mercedes: manual (remote-control) driving

Drives the car from discrete user commands -- faster/slower, steer left/right,
all stop -- on an already-connected, homed BuWizz. Used by DriveServer.py's
/ws/control WebSocket.

Speed and steering are levels rather than raw values: speed runs from -6 (full
reverse) through 0 (stopped) to +6 (full forward), steering from -2 (full left)
through 0 (straight) to +2 (full right). Each command moves one level. The
methods only change those targets; one loop sends them to the brick 20 times a
second (a single 0x31 packet carrying drive PWM and steering degrees) and feeds
the watchdog, so a button press never waits on Bluetooth.

The loop also enforces three safety stops, all of which drop the speed to 0:
    - dead-man: nothing heard from the controlling page for DEADMAN_S seconds
    - front sensor: an obstacle within stop_distance() while going forward --
      further away the faster the car is going (reverse still works, so the
      car can back away)
    - stall: drive power on but the drive motor not turning (see
      car_control.StallDetector), e.g. pushing against something
"""

import asyncio
import time

import car_control
from car_control import (DRIVE_PORT, STEER_PORT, WATCHDOG_FEED,
                         feed_watchdog, motor_ext_packet, motor_packet, send,
                         steer_target)

# PWM for each forward speed level 1-6 (reverse uses the same values, negated).
# Level 0 is stopped. The car barely moves much below ~84, so the levels start
# there and climb in steps of 8, topping out at 120.
SPEED_PWM = (0, 84, 92, 100, 108, 116, 120)
SPEED_LEVELS = len(SPEED_PWM) - 1
STEER_LEVELS = 2       # levels each side of center; the last one is full lock

DEADMAN_S = 1.0        # stop driving if the page goes quiet this long
LOOP_HZ = 20

# The front-sensor stop distance grows with the measured drive speed: the car
# covers more ground between sensor readings, and coasts or brakes further,
# when it's going faster. Linear from STOP_MM_SLOW at the slowest speed (vel
# about 25 at PWM 84) to STOP_MM_FAST at full speed (vel about 110 at PWM 120).
STOP_MM_SLOW, STOP_MM_FAST = 200, 350
STOP_VEL_SLOW, STOP_VEL_FAST = 25, 110
# Brake flag for the drive port in motor packets: with it set, zero power
# brakes the motor instead of letting it coast. Used after a safety stop.
DRIVE_BRAKE = 1 << (DRIVE_PORT - 1)


def speed_pwm(level):
    """Drive PWM for a signed speed level."""
    pwm = SPEED_PWM[abs(level)]
    return pwm if level >= 0 else -pwm


def stop_distance(vel):
    """Front-sensor stop distance (mm) for a measured drive velocity; the
    longest if the velocity isn't known."""
    if vel is None:
        return STOP_MM_FAST
    frac = (abs(vel) - STOP_VEL_SLOW) / (STOP_VEL_FAST - STOP_VEL_SLOW)
    return round(STOP_MM_SLOW + max(0.0, min(1.0, frac)) * (STOP_MM_FAST - STOP_MM_SLOW))


def steer_frac(level):
    """Fraction of full lock (-1 left .. +1 right) for a signed steering level."""
    return level / STEER_LEVELS


class ManualDriver:
    """Remote-control driving on one BuWizz connection. Call run() as a task
    (holding the server's car lock), change the targets with the command
    methods, and end it with request_stop(). If given, `on_stall(pwm, vel)` is
    called when the drive stalls, and `on_safety_stop(reason, speed)` when a
    safety stop drops the speed from `speed` to 0."""

    def __init__(self, client, char, on_safety_stop=None, on_stall=None):
        self.client = client
        self.char = char
        self.on_safety_stop = on_safety_stop
        self.on_stall = on_stall
        self._stall = car_control.StallDetector()
        self._braking = False          # brake (not coast) while the drive is off
        self.speed = 0                 # -SPEED_LEVELS .. +SPEED_LEVELS
        self.steer = 0                 # -STEER_LEVELS .. +STEER_LEVELS
        self.last_heard = time.time()
        self.homing = False            # True while recenter() probes the stops
        self._stop = asyncio.Event()
        # Held for each send, and for the whole of a re-home, so the two
        # never interleave packets.
        self._link = asyncio.Lock()

    # ---- commands (no awaits: they only change targets) ---------------------
    def heard_from(self):
        """The controlling page is still there (any message counts)."""
        self.last_heard = time.time()

    def speed_up(self):
        self.set_speed(self.speed + 1)

    def slow_down(self):
        self.set_speed(self.speed - 1)

    def turn_right(self):
        self.set_steer(self.steer + 1)

    def turn_left(self):
        self.set_steer(self.steer - 1)

    def all_stop(self):
        self.set_speed(0)

    def straighten(self):
        self.set_steer(0)

    def request_stop(self):
        """End run(): it stops the motors and straightens the wheels."""
        self._stop.set()

    def set_speed(self, level):
        """Jump straight to a speed level (clamped to the range)."""
        self.speed = max(-SPEED_LEVELS, min(SPEED_LEVELS, level))
        if self.speed != 0:
            self._braking = False

    def set_steer(self, level):
        """Jump straight to a steering level (clamped to the range)."""
        self.steer = max(-STEER_LEVELS, min(STEER_LEVELS, level))

    # ---- the send loop ------------------------------------------------------
    async def run(self):
        print("Manual control engaged.")
        last_feed = 0.0
        try:
            while not self._stop.is_set():
                async with self._link:
                    self._safety_stops()
                    await send(self.client, self.char, motor_ext_packet({
                        DRIVE_PORT: speed_pwm(self.speed),
                        STEER_PORT: int(steer_target(steer_frac(self.steer)))},
                        brake=DRIVE_BRAKE if self._braking else 0))
                    now = time.time()
                    if now - last_feed > WATCHDOG_FEED:
                        await feed_watchdog(self.client, self.char)
                        last_feed = now
                try:
                    await asyncio.wait_for(self._stop.wait(), 1 / LOOP_HZ)
                except asyncio.TimeoutError:
                    pass
        except ConnectionError as e:
            print(f"\nManual control aborted: {e}")
        finally:
            self._stop.set()
            await car_control.stop_and_straighten(self.client, self.char)
            print("Manual control released.")

    def _safety_stops(self):
        if self.speed != 0 and time.time() - self.last_heard > DEADMAN_S:
            self._safety_stop(f"no word from the page for {DEADMAN_S:.0f} s")
        if self.speed > 0:
            d = car_control.read_distance()
            vel = car_control.drive_vel()
            limit = stop_distance(vel)
            if d is not None and d <= limit:
                self._safety_stop(f"front sensor {d} mm (limit {limit} mm "
                                  f"at vel {vel})")
        pwm, vel = speed_pwm(self.speed), car_control.drive_vel()
        if self._stall.update(pwm, vel):
            if self.on_stall is not None:
                self.on_stall(pwm, vel)
            self._safety_stop("drive stalled")

    def _safety_stop(self, reason):
        """Cut the drive and brake it, until the next Forward or Reverse."""
        print(f"    ! {reason} -> manual stop")
        speed = self.speed
        self.set_speed(0)
        self._braking = True
        if self.on_safety_stop is not None:
            self.on_safety_stop(reason, speed)

    async def recenter(self):
        """Re-calibrate the steering (as POST /recenter does) without ending
        manual control. Drive and steering targets go back to zero. Returns
        True if homing succeeded; raises ConnectionError if the link is lost.

        The wheels are straightened first (as a mission does when it ends):
        re-homing from full lock starts the probe against a stop."""
        async with self._link:
            self.set_speed(0)
            self.set_steer(0)
            await send(self.client, self.char, motor_packet({}))   # all stop
            await car_control.servo_to(self.client, self.char, steer_target(0.0))
            self.homing = True
            try:
                return await car_control.home_steering(self.client, self.char,
                                                       full=True)
            finally:
                self.homing = False

    def state(self):
        return {"speed_level": self.speed, "speed_pwm": speed_pwm(self.speed),
                "steer_level": self.steer}


def _port(values, port):
    return values[port - 1] if len(values) >= port else None


def telemetry(connected):
    """What the brick and sensor last reported, for showing to the user. The
    brick fields are None when not connected (the last report would be stale)."""
    s = car_control.latest_status if connected else {}
    vbat = s.get("vbat")
    drive_a = _port(s.get("currents", []), DRIVE_PORT)
    steer_a = _port(s.get("currents", []), STEER_PORT)
    pos = _port(s.get("pos", []), STEER_PORT)
    cal = car_control.steer_cal
    lock = None if pos is None else cal["lock_right"] if pos >= cal["center"] else cal["lock_left"]
    mm = car_control.recent_distance()
    return {
        "drive": {
            "vel": _port(s.get("vel", []), DRIVE_PORT),    # brick's raw units
            "current_a": drive_a,
            "power_w": round(vbat * drive_a, 1) if None not in (vbat, drive_a) else None,
        },
        "steer": {
            "pos_deg": pos,
            "abs_deg": _port(s.get("abs_pos", []), STEER_PORT),   # -180..179, absolute
            "type": _port(s.get("type", []), STEER_PORT),         # motor type ID
            # Actual angle as a fraction of the usable lock (-1 left .. +1 right).
            "actual_frac": round((pos - cal["center"]) / lock, 2) if pos is not None and lock else None,
            "current_a": steer_a,
            "trim_deg": car_control.steer_trim(),    # straight ahead vs. midpoint
        },
        "battery": {"vbat": vbat, "temp_c": s.get("temp"), "error": s.get("error")},
        "sensor": {
            "present": car_control.distance_sensor is not None,
            "mm": mm,
            "trigger_mm": car_control.SENSOR_TRIGGER_MM,
            "blocked": mm is not None and mm <= car_control.SENSOR_TRIGGER_MM,
        },
    }
