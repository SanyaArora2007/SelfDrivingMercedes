#!/opt/anaconda3/bin/python3

"""
car_control.py  -  Self-Driving Mercedes: shared car-control library

Controls a LEGO car powered by a BuWizz 3.0 Pro (XL drive motor + L steering
motor) over Bluetooth LE. On start it finds the steering center (from the
steering motor's absolute angle, or by probing its end stops), then cruises forward and avoids obstacles on its own: a front VL53L0X distance
sensor reacts before contact, and a velocity-stall check catches anything the
beam misses. On an obstacle it backs up, turns a random direction, and drives
around, then keeps cruising for the mission duration.

Runs on the Raspberry Pi mounted on the car (the pi reads the I2C sensor and
drives the BuWizz over BLE). Without the sensor libraries/hardware (e.g. on a
laptop) it falls back to velocity-stall detection only.

Protocol values are taken from BuWizz's official 3.0 API document.

This module has no entry point of its own; it is used by:
    DriveCar.py     - command-line front end: calls main(), which does one
                      complete connect / home / run / disconnect cycle
    DriveServer.py  - FastAPI web server front end: holds one connection open
                      and calls connect(), home_steering(), drive_session()
                      and disconnect() separately, so the steering is homed
                      once per connection, not per run
    manual_control  - remote-control driving for DriveServer.py

Notes:
    - macOS: first run will prompt for Bluetooth permission for your terminal.
    - Linux/Pi: uses BlueZ; connect from here, don't pair in system settings.
    - Do NOT name this file bleak.py.
"""

import asyncio
import json
import os
import random
import struct
import time
from bleak import BleakScanner, BleakClient

# VL53L0X front distance sensor (I2C). Only available on the Pi; on a laptop the
# imports fail and we fall back to velocity-stall obstacle detection only.
try:
    import board
    import adafruit_vl53l0x
    SENSOR_LIBS_OK = True
except Exception:
    SENSOR_LIBS_OK = False

# ---- Device / protocol constants (from BuWizz 3.0 API) ----------------------
DEVICE_NAME    = "BuWizz3"                                   # advertised name
BUWIZZ_SERVICE = "500592d1-74fb-4481-88b3-9919b1676e93"      # main service UUID
APP_CHAR_HINT  = "2901"   # the application characteristic's distinguishing bytes

CMD_SET_MOTOR      = 0x30  # 6x signed-8bit PWM (-127..127), + brake + lut bytes
CMD_SET_MOTOR_EXT  = 0x31  # 4x int32 refs (ports 1-4, per PU mode) +2x int8 +brake+lut
CMD_XFER_PERIOD    = 0x32  # status report period (ms, 20-255)
CMD_MOTOR_TIMEOUT  = 0x34  # what to do when watchdog trips
CMD_WATCHDOG       = 0x35  # auto-stop if no command within N seconds
CMD_SET_PU_FUNC    = 0x50  # set PU port function (0x10 = simple PWM)
CMD_SET_SERVO_REF  = 0x52  # set servo reference: 4x signed-32bit, PU ports 1-4
STATUS_REPORT_ID   = 0x01

# PU port function bytes (command 0x50)
PU_SIMPLE_PWM = 0x10  # raw PWM, driven via 0x30 motor data
PU_POS_SERVO  = 0x15  # position servo: reference is degrees (cumulative from zero)

# ---- Your build --------------------------------------------------------------
DRIVE_PORT = 1    # PU port the XL drive motor is plugged into (1-4)
STEER_PORT = 2    # PU port the L steering motor is plugged into (1-4)
STEER_MOTOR_TYPE = 46  # Technic Large motor: reports an absolute shaft angle

MAX_DRIVE   = 45  # crawl cap out of 127 (mirrors your power-curve ceiling)
STEER_TIMEOUT = 2.0  # max seconds to wait for the steering servo to converge
STEER_TOL = 3  # consider the target reached within this many degrees

# ---- Steering end-stop calibration ------------------------------------------
# The position servo commands MOTOR-shaft degrees; gearing to the road wheels
# means full lock is many motor degrees away and bounded by hard end stops.
# We find that range by ramping the servo target outward in steps until the
# shaft can no longer follow (it's pressed against a stop), then steer as a
# fraction of the measured lock-to-lock travel. Using the servo (not raw PWM)
# keeps full torque and never leaves servo mode.
CAL_STEP     = 20    # motor-degree increment when probing outward for a stop
CAL_MAX      = 500   # never probe further than this many degrees from center
CAL_LAG      = 10    # if the shaft trails the target by more than this -> at a stop
CAL_BACKOFF  = 60    # back away this far before probing, for a run-up to the stop
CAL_STOP_STEPS = 3   # steps in a row that barely move before we call it a stop
CAL_ABS_SLACK = 20   # how far past full lock an absolute reading may still be
TRIM_MAX = 60        # how far straight ahead may be trimmed from the stops' midpoint
STEER_MARGIN = 0.85  # with straight ahead at the midpoint, use this fraction of
                     # the half-lock (the rest is clearance from the stops)

# The lock-to-lock travel is a fixed property of the build, so we cache the
# half-lock span. The cumulative position's zero is wherever the motor was at
# brick power-up, so angles in those degrees can't be cached -- but the motor's
# ABSOLUTE angle (abs_pos, -180..179) keeps its meaning across power cycles, so
# we cache the stops' midpoint (`mid_abs`) and straight ahead (`center_abs`) as
# absolute angles and re-derive both on each connect, with no probing (see
# abs_home()). Straight ahead isn't necessarily the midpoint: on this car the
# left side has more travel. Their difference is the steering trim, which a
# full calibration keeps; set it with trim_steering() (the page's trim buttons).
CACHE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "steer_cal.json")

WATCHDOG_S    = 3     # auto-stop if no 0x35 command within this many seconds
WATCHDOG_FEED = 0.5   # re-arm the watchdog at least this often while driving

# ---- Obstacle-avoidance mission ---------------------------------------------
REVERSE_SPEED = 85    # PWM used when backing away from an obstacle
DRIVE_SPEED   = REVERSE_SPEED  # forward PWM (cruise) -- same as reverse
TURN_SPEED    = 95    # PWM while going around with wheels turned
BACKUP_TIME   = 5.0   # seconds to escape an obstacle (reverse, or forward for a
                      # rear hit) -- 5 s is enough at the higher reverse speed
TURN_DRIVE_TIME = 5.0 # seconds to drive with wheels turned to go around
MAX_RECOVER_DEPTH = 2 # cap on chained forward<->reverse recoveries (boxed in)
MISSION_TIME  = 120.0 # default overall run length (2 min); see --duration
STRAIGHT_MAX  = 10.0  # max seconds of straight driving before a gentle turn
SMOOTH_TURN_FRAC = 0.5  # how far to steer for that turn (fraction of full lock)
SMOOTH_TURN_TIME = 3.0  # seconds to hold the gentle turn before straightening

# Cruise acceleration: every ACCEL_INTERVAL seconds of clear straight driving,
# bump the speed by ACCEL_STEP (up to ACCEL_MAX). Resets to DRIVE_SPEED after
# each obstacle, so the car speeds up only while it knows the path is clear.
ACCEL_INTERVAL = 5.0  # seconds of clear driving before each speed bump
ACCEL_STEP     = 5    # PWM added per interval
ACCEL_MAX      = 120  # top cruise PWM (out of 127)

# Obstacle = commanding forward power but the drive motor isn't turning.
STALL_VEL   = 4       # |drive velocity| below this counts as "not moving"
STALL_TIME  = 0.4     # must stay stalled this long (while powered) -> obstacle
DRIVE_GRACE = 0.6     # ignore stalls this long after starting to drive (spin-up)

# ---- Status report sanity check ---------------------------------------------
# Now and then the brick sends a status report with some bytes zeroed: seen in
# the steering position, e.g. -14 arriving as 0, -256, 65522 or -16776974, one
# report at a time. A report is dropped (the previous one stays current) when it
# disagrees with what the steering motor can physically do. A real change would
# persist, so after STATUS_MAX_DROPS dropped reports in a row the next is taken.
STATUS_MAX_STEER_STEP = 60  # deg; the servo can't turn further in one report
STATUS_ABS_TOL = 10         # deg; pos and abs_pos keep a fixed offset (mod 360)
STATUS_MAX_DROPS = 3
STATUS_DROP_LOG_S = 10      # report dropped reports at most this often

# ---- Front distance sensor (VL53L0X) ----------------------------------------
# When driving forward, an obstacle seen within this distance triggers the same
# recovery as a physical hit. 250 mm gives the car room to stop/turn before
# contact (real avoidance); lower it toward "touching" only if you want it to
# react later.
SENSOR_TRIGGER_MM = 250  # obstacle if the front sensor reads <= this (and > 0)
SENSOR_MIN_MM     = 1    # readings at/below this are treated as invalid/noise

latest_status = {}
# The status filter's state: the steering motor's pos-to-abs_pos offset, the
# drops in a row, and the drops not yet reported (see on_status()).
status_filter = {"offset": None, "drops": 0, "unreported": 0, "logged": 0.0}
# The drive PWM most recently sent to the brick, whoever sent it (see send()).
drive_command = {"pwm": 0}
disconnect_info = {"reason": None}
distance_sensor = None    # set in main() when the VL53L0X is present
last_distance = {"mm": None, "t": 0.0}   # last read_distance() result and when
# Filled in by calibrate_steering()/abs_home() (see set_steering()): straight
# ahead and the half-lock in motor degrees, each side's usable lock from
# straight ahead, and the cached absolute angles.
steer_cal = {"center": 0.0, "mid": 0.0, "half": 0.0, "lock_left": 0.0,
             "lock_right": 0.0, "center_abs": None, "mid_abs": None}


def load_cache():
    """Return the cached steering calibration as {"half": float, "center_abs":
    int or None, "mid_abs": int or None, "trim": int}, or None if unavailable
    or unusable. `trim` is how far straight ahead is from the stops' midpoint
    (0 if unknown); a cache without mid_abs is taken to have no trim."""
    try:
        with open(CACHE_FILE) as f:
            data = json.load(f)
        half = float(data["half"])
        center_abs = data.get("center_abs")
        center_abs = None if center_abs is None else int(center_abs)
        mid_abs = data.get("mid_abs", center_abs)
        mid_abs = None if mid_abs is None else int(mid_abs)
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if half < CAL_STEP:
        return None
    trim = 0 if None in (center_abs, mid_abs) else wrap180(center_abs - mid_abs)
    return {"half": half, "center_abs": center_abs, "mid_abs": mid_abs,
            "trim": trim}


def save_cache():
    """Persist the half-lock span and the absolute angles of straight ahead and
    the stops' midpoint (from steer_cal) next to the script."""
    try:
        with open(CACHE_FILE, "w") as f:
            json.dump({"half": round(steer_cal["half"], 1),
                       "mid_abs": steer_cal["mid_abs"],
                       "center_abs": steer_cal["center_abs"]}, f)
        print(f"  saved steering calibration to {CACHE_FILE}")
    except OSError as e:
        print(f"  [!] could not save calibration: {e}")


def _sb(v):
    """clamp to signed int8 range and return the raw byte value"""
    v = max(-127, min(127, int(v)))
    return v & 0xFF


def motor_packet(values, brake=0, lut=0):
    """values: dict of port(1-6) -> signed PWM (-127..127). Others default 0."""
    m = [0, 0, 0, 0, 0, 0]
    for port, val in values.items():
        m[port - 1] = val
    return bytes([CMD_SET_MOTOR] + [_sb(x) for x in m] + [brake, lut])


def pu_func_packet(functions):
    """functions: dict of PU port(1-4) -> function byte. Others = simple PWM."""
    f = [PU_SIMPLE_PWM] * 4
    for port, fn in functions.items():
        f[port - 1] = fn
    return bytes([CMD_SET_PU_FUNC] + f)


def servo_ref_packet(refs):
    """refs: dict of PU port(1-4) -> signed 32-bit reference. For a position-
    servo port the reference is the target angle in degrees. Others default 0."""
    r = [0, 0, 0, 0]
    for port, val in refs.items():
        r[port - 1] = int(val)
    return bytes([CMD_SET_SERVO_REF]) + b"".join(struct.pack("<i", x) for x in r)


def motor_ext_packet(refs, brake=0, lut=0):
    """Command 0x31: one packet that sets every PU port 1-4 by its own mode --
    PWM (-127..127) for a simple-PWM port, degrees for a position-servo port.
    Lets us drive one motor while holding another as a servo without the two
    fighting over a shared 0x30/0x52 packet. refs: dict of port(1-4) -> ref."""
    r = [0, 0, 0, 0]
    for port, val in refs.items():
        r[port - 1] = int(val)
    body = b"".join(struct.pack("<i", x) for x in r)
    return bytes([CMD_SET_MOTOR_EXT]) + body + bytes([0, 0, brake, lut])


def parse_status(data):
    if not data or data[0] != STATUS_REPORT_ID:
        return None
    flags = data[1]
    out = {
        "error": bool(flags & 0x01),
        "vbat": round(9.0 + data[2] * 0.05, 2),
        "currents": [round(data[3 + i] * 0.015, 2) for i in range(6)],
        "temp": data[9],
        "type": [],     # PU motor type per port (0 = none/unknown), ports 1-4
        "vel": [],      # PU port velocities (signed), ports 1-4
        "abs_pos": [],  # absolute shaft angle -180..179 deg, ports 1-4
        "pos": [],      # cumulative position in degrees (signed), ports 1-4
    }
    # PoweredUp motor data: 8 bytes/port -> type(u8) vel(s8) abs_pos(s16) pos(s32)
    for i in range(4):
        base = 22 + i * 8
        if base + 8 <= len(data):
            mtype, vel, abs_pos, pos = struct.unpack_from("<Bbhi", data, base)
            out["type"].append(mtype)
            out["vel"].append(vel)
            out["abs_pos"].append(abs_pos)
            out["pos"].append(pos)
    return out


def on_status(_sender, data):
    s = parse_status(data)
    if not s:
        return
    f = status_filter
    if _plausible(s) or f["drops"] >= STATUS_MAX_DROPS:
        if f["drops"] >= STATUS_MAX_DROPS:
            f["offset"] = None                     # relearn from this report
        f["drops"] = 0
        latest_status.update(s)
        if f["offset"] is None:
            f["offset"] = _abs_offset(s)
    else:
        f["drops"] += 1
        f["unreported"] += 1
        now = time.monotonic()
        if now - f["logged"] >= STATUS_DROP_LOG_S:
            pos = s["pos"][STEER_PORT - 1] if len(s["pos"]) >= STEER_PORT else None
            print(f"  [!] dropped {f['unreported']} corrupt status report(s) "
                  f"(latest: steering pos={pos} deg, was {steer_pos()} deg)")
            f.update(unreported=0, logged=now)


def _abs_offset(s):
    """The steering motor's abs_pos minus pos (mod 360) in a report, or None
    if it doesn't report an absolute angle."""
    i = STEER_PORT - 1
    if len(s["pos"]) <= i or s["type"][i] != STEER_MOTOR_TYPE:
        return None
    return wrap180(s["abs_pos"][i] - s["pos"][i])


def _plausible(s):
    """Whether a status report is consistent with the last one kept."""
    if not latest_status:
        return True
    if s["type"] != latest_status.get("type"):
        return False                               # motors don't change mid-run
    i = STEER_PORT - 1
    old = latest_status.get("pos", [])
    if len(s["pos"]) > i and len(old) > i:
        if abs(s["pos"][i] - old[i]) > STATUS_MAX_STEER_STEP:
            return False
        offset, expected = _abs_offset(s), status_filter["offset"]
        if None not in (offset, expected) and abs(wrap180(offset - expected)) > STATUS_ABS_TOL:
            return False
    return True


async def find_app_char(client):
    for service in client.services:
        if service.uuid.lower() == BUWIZZ_SERVICE:
            for c in service.characteristics:
                if APP_CHAR_HINT in c.uuid.lower():
                    return c
    return None


def on_disconnect(_client):
    # A reason is already set if we disconnected on purpose (see disconnect()).
    if disconnect_info["reason"] is None:
        disconnect_info["reason"] = "peripheral dropped the BLE connection"
        print("  [!] BuWizz disconnected unexpectedly")


async def send(client, char, packet):
    """Write a packet, converting a lost connection into a clear error. Notes
    the drive PWM of motor packets, for stall detection."""
    if not client.is_connected:
        raise ConnectionError(
            disconnect_info["reason"] or "BuWizz is no longer connected"
        )
    await client.write_gatt_char(char, packet, response=False)
    if packet[0] == CMD_SET_MOTOR_EXT:
        drive_command["pwm"] = struct.unpack_from("<i", packet, 1 + 4 * (DRIVE_PORT - 1))[0]
    elif packet[0] == CMD_SET_MOTOR:
        drive_command["pwm"] = struct.unpack_from("b", packet, DRIVE_PORT)[0]


async def feed_watchdog(client, char):
    """Re-arm the connection watchdog. Per the BuWizz 3.0 API, only the 0x35
    command resets the watchdog timer -- motor (0x30) writes do NOT."""
    await send(client, char, bytes([CMD_WATCHDOG, WATCHDOG_S]))


async def hold(client, char, values, seconds, hz=20):
    """Send a motor command repeatedly for `seconds`, re-arming the watchdog."""
    pkt = motor_packet(values)
    end = time.time() + seconds
    last_print = 0.0
    last_feed = 0.0
    while time.time() < end:
        await send(client, char, pkt)
        now = time.time()
        if now - last_feed > WATCHDOG_FEED:
            await feed_watchdog(client, char)
            last_feed = now
        if now - last_print > 0.5 and latest_status:
            c = latest_status.get("currents", [0] * 6)
            v = latest_status.get("vel", [])
            drive_v = v[DRIVE_PORT - 1] if len(v) >= DRIVE_PORT else "?"
            print(f"    vbat={latest_status.get('vbat')}V  "
                  f"I(drive)={c[DRIVE_PORT-1]}A  drive_vel={drive_v}")
            last_print = now
        await asyncio.sleep(1 / hz)


def steer_field(key):
    """The steering port's latest value of a per-port status field ("pos",
    "abs_pos", "type", ...), or None if not reported yet."""
    v = latest_status.get(key, [])
    return v[STEER_PORT - 1] if len(v) >= STEER_PORT else None


def steer_pos():
    """Current steering motor position in degrees, or None if not reported yet."""
    return steer_field("pos")


def wrap180(deg):
    """An angle or angle difference folded into -180..179."""
    return (deg + 180) % 360 - 180


def log_steer_angles(label):
    """Print the steering motor's type, cumulative position and absolute
    (-180..179) angle."""
    print(f"  steering {label}: type={steer_field('type')}  pos={steer_pos()} deg  "
          f"abs_pos={steer_field('abs_pos')} deg")


async def servo_to(client, char, target_deg, timeout=STEER_TIMEOUT, tol=STEER_TOL,
                   hz=20):
    """Command the steering position servo to `target_deg` (motor degrees) and
    wait until the shaft reaches it (within `tol`) or `timeout` elapses. The
    command is re-sent every tick because BLE writes here are unacknowledged and
    a lone packet can be dropped. The drive motor is explicitly held at zero so
    the car stays put while the wheels turn. Returns the angle actually reached;
    the onboard PID keeps holding the target after this returns."""
    target = int(target_deg)
    pkt = motor_ext_packet({DRIVE_PORT: 0, STEER_PORT: target})
    end = time.time() + timeout
    last_feed = 0.0
    actual = None
    while time.time() < end:
        await send(client, char, pkt)
        now = time.time()
        if now - last_feed > WATCHDOG_FEED:
            await feed_watchdog(client, char)
            last_feed = now
        actual = steer_pos()
        if actual is not None and abs(actual - target) <= tol:
            break
        await asyncio.sleep(1 / hz)
    return actual


async def probe_stop(client, char, start, direction):
    """Ramp the servo target outward from `start` in CAL_STEP steps until the
    shaft stops advancing (pressed against an end stop). A real stop is detected
    when a step advances the shaft by much less than it was commanded to move --
    not merely when the shaft trails the target (which also happens in motion).
    Returns the furthest angle actually reached.

    It first backs CAL_BACKOFF degrees away from the stop: the last stretch
    before a stop is stiff, and starting inside it from standstill the shaft
    barely moves on the first step, which would be mistaken for the stop itself
    (e.g. when a run ended with the wheels turned). The run-up carries it
    through to the real stop.

    The run-up can't help when the back-off itself lands against the OTHER stop
    (wheels already at the far lock), so a stall only counts as the stop after
    CAL_STOP_STEPS steps in a row: in the stiff zone the target keeps pulling
    ahead and the shaft breaks free, while at a real stop it never moves. The
    stop is where the stall began, before the extra steps pressed into it."""
    backed = await servo_to(client, char, start - direction * CAL_BACKOFF,
                            timeout=1.5, tol=CAL_LAG)
    if backed is not None:
        start = backed
    reached = prev = start
    stalls = 0
    step = 0
    while step < CAL_MAX:
        step += CAL_STEP
        target = start + direction * step
        actual = await servo_to(client, char, target, timeout=1.2, tol=STEER_TOL)
        if actual is None:
            break
        progress = (actual - prev) * direction     # advance this step (deg)
        print(f"    probe {'+' if direction > 0 else '-'}: target={target} deg  "
              f"actual={actual} deg  (advanced {progress:+d})")
        prev = actual
        if progress < CAL_STEP * 0.4:              # barely moved: stop or stiff zone
            if stalls == 0:
                reached = actual
            stalls += 1
            if stalls >= CAL_STOP_STEPS:
                break
        else:
            stalls = 0
            reached = actual
    return reached


def set_steering(center, mid, half):
    """Record the steering geometry in this session's motor degrees: `center`
    is straight ahead and `mid` the midpoint between the end stops. Each side's
    usable lock runs from straight ahead to that side's stop, short by the same
    clearance STEER_MARGIN leaves when straight ahead is the midpoint -- so a
    trimmed center never steers into a stop."""
    clearance = (1 - STEER_MARGIN) * half
    steer_cal.update(center=center, mid=mid, half=half,
                     lock_right=max(0.0, mid + half - center - clearance),
                     lock_left=max(0.0, center - (mid - half) - clearance))


def steer_trim():
    """How far straight ahead is from the stops' midpoint (+ = right), in
    motor degrees."""
    return round(steer_cal["center"] - steer_cal["mid"])


def trim_steering(delta):
    """Move straight ahead by `delta` motor degrees (+ = right), keeping it
    within TRIM_MAX of the stops' midpoint, and cache the new trim so homing
    and recalibration keep it. Doesn't move the servo: the caller re-sends the
    steering target. Returns the new trim."""
    trim = max(-TRIM_MAX, min(TRIM_MAX, steer_trim() + delta))
    set_steering(steer_cal["mid"] + trim, steer_cal["mid"], steer_cal["half"])
    if steer_cal["mid_abs"] is not None:
        steer_cal["center_abs"] = wrap180(steer_cal["mid_abs"] + trim)
        save_cache()
    print(f"Steering trim {trim:+d} deg (straight ahead abs_pos="
          f"{steer_cal['center_abs']} deg)")
    return trim


async def calibrate_steering(client, char, trim=0):
    """Find both steering end stops by servo probing and record the half-lock
    span and straight ahead -- `trim` degrees from the stops' midpoint (the
    cached trim, so recalibrating keeps it) -- plus their absolute angles for
    abs_home(). Leaves the steer port as a position servo at straight ahead.
    Stays in servo mode throughout."""
    print("Calibrating steering travel...")
    await send(client, char, pu_func_packet({STEER_PORT: PU_POS_SERVO}))
    await asyncio.sleep(0.3)
    start = steer_pos()
    if start is None:
        print("  [!] No steering position feedback -- is the steer motor a "
              "PoweredUp motor on the right port?")
        return False
    await servo_to(client, char, start, timeout=0.5, tol=CAL_LAG)  # hold, settle

    right = await probe_stop(client, char, start, +1)
    await servo_to(client, char, start, timeout=3.0, tol=CAL_LAG)  # back to start
    left = await probe_stop(client, char, start, -1)

    mid = (left + right) / 2.0
    half = abs(right - left) / 2.0
    center = mid + trim
    print(f"  left={left} deg  right={right} deg  ->  midpoint={mid:.0f} deg, "
          f"half-lock={half:.0f} deg, straight ahead={center:.0f} deg "
          f"(trim {trim:+d}) (motor)")
    set_steering(center, mid, half)
    await servo_to(client, char, center, timeout=3.0)
    if half < CAL_STEP:
        print("  [!] Steering travel looks tiny -- calibration may be off.")
        return False
    # The midpoint's absolute angle: where the shaft is now, corrected by how
    # far it is from the midpoint.
    pos, abs_deg = steer_pos(), steer_field("abs_pos")
    if (pos is not None and abs_deg is not None
            and steer_field("type") == STEER_MOTOR_TYPE):
        mid_abs = wrap180(abs_deg + round(mid - pos))
        steer_cal.update(mid_abs=mid_abs, center_abs=wrap180(mid_abs + trim))
        print(f"  abs_pos: midpoint={mid_abs} deg, "
              f"straight ahead={steer_cal['center_abs']} deg")
    else:
        steer_cal.update(mid_abs=None, center_abs=None)
    return True


async def abs_home(client, char, half, center_abs, mid_abs):
    """Find straight ahead without probing, from the steering motor's absolute
    angle: it is where abs_pos equals the cached `center_abs` (and the stops'
    midpoint where it equals `mid_abs`). Lock to lock is under a full turn, so
    the shaft's offset from either -- folded into -180..179 -- is unambiguous,
    and each in this session's cumulative degrees is pos minus that offset.
    Leaves the steer port as a position servo holding straight ahead. Returns
    True on success; on failure (no absolute angle, a reading past the stops,
    or a center the shaft can't reach) steer_cal is left as it was."""
    print("Homing steering from its absolute angle...")
    await send(client, char, pu_func_packet({STEER_PORT: PU_POS_SERVO}))
    await asyncio.sleep(0.3)
    pos, abs_deg, mtype = steer_pos(), steer_field("abs_pos"), steer_field("type")
    if pos is None or abs_deg is None or mtype != STEER_MOTOR_TYPE:
        print(f"  [!] No usable absolute angle (type={mtype}, abs_pos={abs_deg}).")
        return False
    from_mid = wrap180(abs_deg - mid_abs)
    if abs(from_mid) > half + CAL_ABS_SLACK:
        print(f"  [!] abs_pos={abs_deg} deg is {from_mid:+d} deg from the cached "
              f"midpoint ({mid_abs} deg), past the stops -- cache looks stale.")
        return False
    center = pos - wrap180(abs_deg - center_abs)
    mid = pos - from_mid
    print(f"  abs_pos={abs_deg} deg, cached straight ahead abs_pos={center_abs} "
          f"deg  ->  center={center} deg (motor)")
    actual = await servo_to(client, char, center, timeout=3.0)
    if actual is None or abs(actual - center) > CAL_LAG:
        print(f"  [!] Couldn't reach that center (stuck at {actual} deg).")
        return False
    set_steering(center, mid, half)
    steer_cal.update(center_abs=center_abs, mid_abs=mid_abs)
    return True


async def steer_to_fraction(client, char, frac):
    """Steer to a fraction of full lock: -1.0 = full left, +1.0 = full right,
    0.0 = center. A safety margin keeps it off the hard stops."""
    frac = max(-1.0, min(1.0, frac))
    target = steer_target(frac)
    actual = await servo_to(client, char, target)
    shown = actual if actual is not None else "?"
    print(f"    steer {frac:+.0%} lock  ->  target={target:.0f} deg  "
          f"actual={shown} deg (motor)")


# ---- Obstacle-avoidance behavior --------------------------------------------
def steer_target(frac):
    """Motor-degree servo target for a steering fraction (-1 left .. +1 right)
    of that side's usable lock."""
    frac = max(-1.0, min(1.0, frac))
    lock = steer_cal["lock_right"] if frac >= 0 else steer_cal["lock_left"]
    return steer_cal["center"] + frac * lock


def drive_vel():
    """Current drive-motor velocity (signed), or None if not reported yet."""
    v = latest_status.get("vel", [])
    return v[DRIVE_PORT - 1] if len(v) >= DRIVE_PORT else None


def drive_current():
    """Current drawn by the drive motor (A), or None if not reported yet."""
    c = latest_status.get("currents", [])
    return c[DRIVE_PORT - 1] if len(c) >= DRIVE_PORT else None


class StallDetector:
    """Spots a drive stall: power commanded but the drive motor not turning
    (|vel| < STALL_VEL) for STALL_TIME, ignoring the first DRIVE_GRACE seconds
    after the drive starts or changes direction (spin-up). The same test as
    drive() makes during a mission."""

    def __init__(self):
        self.direction = 0
        self.since = 0.0             # when the drive started in this direction
        self.stalled_since = None
        self.reported = False

    def update(self, pwm, vel, now=None):
        """Feed in the commanded PWM and measured velocity. Returns True once
        per stall, when it has lasted STALL_TIME."""
        now = time.monotonic() if now is None else now
        direction = (pwm > 0) - (pwm < 0)
        if direction != self.direction:
            self.direction, self.since = direction, now
            self.stalled_since, self.reported = None, False
        if direction == 0 or vel is None or now - self.since < DRIVE_GRACE:
            return False
        if abs(vel) >= STALL_VEL:
            self.stalled_since, self.reported = None, False
            return False
        if self.stalled_since is None:
            self.stalled_since = now
        if not self.reported and now - self.stalled_since >= STALL_TIME:
            self.reported = True
            return True
        return False


def init_sensor():
    """Bring up the VL53L0X front sensor if the libraries and hardware are
    present. Sets the module-level `distance_sensor`. Returns True on success."""
    global distance_sensor
    if not SENSOR_LIBS_OK:
        print("Front sensor: libraries not available -> velocity-stall detection only.")
        return False
    try:
        i2c = board.I2C()
        distance_sensor = adafruit_vl53l0x.VL53L0X(i2c)
        print(f"Front sensor: VL53L0X ready (trigger <= {SENSOR_TRIGGER_MM} mm).")
        return True
    except Exception as e:
        print(f"Front sensor: not detected ({e}) -> velocity-stall detection only.")
        distance_sensor = None
        return False


def read_distance():
    """Front obstacle distance in mm, or None if no sensor / bad read. Values at
    or below SENSOR_MIN_MM are treated as invalid (error/no-return)."""
    if distance_sensor is None:
        return None
    try:
        mm = distance_sensor.range
    except Exception:
        mm = None
    if mm is not None and mm <= SENSOR_MIN_MM:
        mm = None
    last_distance.update(mm=mm, t=time.time())
    return mm


def recent_distance(max_age=0.25):
    """The last distance reading if it's fresh (driving reads the sensor every
    tick), otherwise a new reading. For display; avoids doubling sensor reads."""
    if time.time() - last_distance["t"] <= max_age:
        return last_distance["mm"]
    return read_distance()


async def drive(client, char, pwm, seconds, steer_deg, detect=True, accelerate=False,
                hz=20):
    """Drive the drive motor at `pwm` for up to `seconds` while holding the
    steering servo at `steer_deg` (motor degrees), via a single 0x31 packet so
    the drive and steer commands never zero each other out.

    If `accelerate` is set, the (forward) speed ramps up by ACCEL_STEP every
    ACCEL_INTERVAL seconds of clear driving, capped at ACCEL_MAX -- used for
    cruising, where a long stretch without a hit means the way is clear.

    If `detect` is set and we are commanding motion (pwm != 0) but the drive
    motor's velocity stays near zero past the spin-up grace period, we treat it
    as an obstacle -- in whichever direction we're going -- stop the drive
    (steering held) and return True. Otherwise return False after the full
    duration."""
    hold_pkt = motor_ext_packet({DRIVE_PORT: 0, STEER_PORT: int(steer_deg)})
    start = time.time()
    end = start + seconds
    last_feed = last_print = 0.0
    stalled_since = None
    cur_pwm = pwm
    while time.time() < end:
        now = time.time()
        if accelerate:
            steps = int((now - start) // ACCEL_INTERVAL)
            cur_pwm = min(ACCEL_MAX, pwm + steps * ACCEL_STEP)
        await send(client, char,
                   motor_ext_packet({DRIVE_PORT: cur_pwm, STEER_PORT: int(steer_deg)}))
        if now - last_feed > WATCHDOG_FEED:
            await feed_watchdog(client, char)
            last_feed = now

        v = drive_vel()
        # Proactive: front sensor sees an obstacle close ahead while driving
        # forward -> react before contact (same recovery as a physical hit).
        if detect and pwm > 0:
            d = read_distance()
            if d is not None and d <= SENSOR_TRIGGER_MM:
                await send(client, char, hold_pkt)  # stop drive, hold steer
                print(f"    ! sensor {d} mm -> obstacle ahead")
                return True
        # Fallback: commanding motion but the drive motor isn't turning -> hit.
        if detect and pwm != 0 and now - start > DRIVE_GRACE:
            if v is not None and abs(v) < STALL_VEL:
                stalled_since = stalled_since or now
                if now - stalled_since >= STALL_TIME:
                    await send(client, char, hold_pkt)  # stop drive, hold steer
                    where = "ahead" if pwm > 0 else "behind"
                    print(f"    ! stalled (vel~{v}) -> obstacle {where}")
                    return True
            else:
                stalled_since = None

        if now - last_print > 0.5:
            c = latest_status.get("currents", [0] * 6)
            print(f"    pwm={cur_pwm:+d}  drive_vel={v}  I(drive)={c[DRIVE_PORT-1]}A")
            last_print = now
        await asyncio.sleep(1 / hz)
    return False


async def obstacle(client, char, depth=0):
    """Recovery after the car hits something while moving FORWARD: back away,
    then try to steer around it. Mirror of reverse_obstacle()."""
    return await _recover(client, char, going_forward=True, depth=depth)


async def reverse_obstacle(client, char, depth=0):
    """Recovery after the car hits something while REVERSING: pull FORWARD away,
    then try to steer around it in reverse. Mirror of obstacle() -- same moves,
    just the opposite escape direction."""
    return await _recover(client, char, going_forward=False, depth=depth)


async def _recover(client, char, going_forward, depth):
    """Shared recovery body. `going_forward` is the direction we were travelling
    when we hit, so we escape the opposite way and probe (try to go around) in
    the original direction. The go-around is a smooth turn -- the wheels swing
    over while the car keeps moving, and the next cruise segment eases them back
    to straight. If we bump a new obstacle while escaping, we recover the other
    way -- bounded by MAX_RECOVER_DEPTH so a boxed-in car can't loop forever."""
    center = steer_target(0.0)
    # Randomly try around to the right or the left each time.
    turn_frac = random.choice([-1.0, 1.0])
    turn = steer_target(turn_frac)
    turn_name = "right" if turn_frac > 0 else "left"
    # Escape opposite to travel; probe (wheels turned) in the travel direction.
    if going_forward:
        escape_pwm, probe_pwm = -REVERSE_SPEED, TURN_SPEED
        escape_name, probe_name = "reverse", "forward"
    else:
        escape_pwm, probe_pwm = DRIVE_SPEED, -TURN_SPEED
        escape_name, probe_name = "forward", "reverse"
    hit_dir = "front" if going_forward else "rear"
    print(f"  ! {hit_dir} obstacle -- running recovery (depth {depth})")

    # 1. Escape straight away from the obstacle. Watch for a new obstacle in the
    #    escape direction (unless we're at the depth cap): if we bump one while
    #    escaping, recover the other way instead (front<->reverse mirror).
    print(f"  recovery: {escape_name}")
    detect_escape = depth < MAX_RECOVER_DEPTH
    if await drive(client, char, escape_pwm, BACKUP_TIME, center, detect=detect_escape):
        print(f"  recovery: bumped something while moving {escape_name}")
        await _recover(client, char, not going_forward, depth + 1)
        return

    # 2. Go around in the travel direction with a SMOOTH turn: no stop -- the
    #    wheels swing to the turn angle while the car keeps moving. They ease
    #    back to straight in the next cruise segment (also while moving).
    print(f"  recovery: {probe_name}, smooth {turn_name} turn")
    await drive(client, char, probe_pwm, TURN_DRIVE_TIME, turn)


async def random_turn(client, char):
    """After a long straight run, ease into a gentle turn WITHOUT stopping: keep
    driving forward while nudging the wheels to a slight random (left/right)
    angle for SMOOTH_TURN_TIME seconds. The wheels aren't straightened here -- the
    next cruise segment eases them back to center, also while moving, for one
    continuous smooth curve. Returns True if an obstacle was hit mid-turn (caller
    recovers)."""
    turn_frac = random.choice([-1.0, 1.0]) * SMOOTH_TURN_FRAC
    turn = steer_target(turn_frac)
    turn_name = "right" if turn_frac > 0 else "left"
    print(f"  straight-line timeout -> gentle {turn_name} turn (while moving)")
    # Drive forward at cruise speed while holding the slight steer angle; the
    # steering servo eases over as the car keeps rolling -> a smooth curve.
    return await drive(client, char, DRIVE_SPEED, SMOOTH_TURN_TIME, turn)


async def run_mission(client, char, duration=MISSION_TIME, accelerate=False):
    """Cruise forward and avoid obstacles for about `duration` seconds. Straight
    driving is capped at STRAIGHT_MAX seconds -- after that we make a random turn
    even with a clear path. If `accelerate` is set, cruising speeds up while the
    path stays clear; each hit triggers the obstacle() recovery, then we
    straighten and cruise again (back at the base speed)."""
    center = steer_target(0.0)
    print(f"\nObstacle-avoidance mission (~{duration:.0f}s"
          f"{', accelerating' if accelerate else ''})...\n")
    await servo_to(client, char, center)          # wheels straight
    deadline = time.time() + duration
    while time.time() < deadline:
        secs = min(STRAIGHT_MAX, deadline - time.time())
        print("  cruise forward")
        hit = await drive(client, char, DRIVE_SPEED, secs, center,
                          accelerate=accelerate)
        # Finished the straight segment without a hit, with time to spare ->
        # the 10 s cap tripped, so make a random turn.
        if not hit and time.time() < deadline:
            hit = await random_turn(client, char)
        if hit:
            await obstacle(client, char)          # recover, then loop and cruise
    print("\nMission time reached.")


async def run_selftest(client, char):
    """Original fixed motion self-test (drive + steer sweep)."""
    print("\nRunning motion self-test...\n")
    print("  drive forward (crawl)")
    await hold(client, char, {DRIVE_PORT: 80}, 2.0)
    print("  stop")
    await hold(client, char, {}, 1.0)
    print("  steer full left")
    await steer_to_fraction(client, char, -1.0)
    print("  steer full right")
    await steer_to_fraction(client, char, 1.0)
    print("  center")
    await steer_to_fraction(client, char, 0.0)
    print("  drive reverse (crawl)")
    await hold(client, char, {DRIVE_PORT: -80}, 2.0)


async def connect():
    """Scan for and connect to the BuWizz, start 20 Hz status reports and arm
    the watchdog. Returns (client, char) on success; raises ConnectionError
    otherwise. The caller owns the connection and must eventually
    `await client.disconnect()`."""
    # Clear state left over from a previous connection in this process.
    latest_status.clear()
    status_filter.update(offset=None, drops=0, unreported=0)
    drive_command["pwm"] = 0
    disconnect_info["reason"] = None

    print(f"Scanning for {DEVICE_NAME} ...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15.0)
    if device is None:
        raise ConnectionError(f"{DEVICE_NAME} not found. Make sure the brick is "
                              f"on (press its button) and in range.")

    print(f"Connecting to {device.address} ...")
    client = BleakClient(device, disconnected_callback=on_disconnect)
    await client.connect()
    try:
        app = await find_app_char(client)
        if app is None:
            print("Could not find the application characteristic. Discovered:")
            for service in client.services:
                print("service", service.uuid)
                for c in service.characteristics:
                    print("   char", c.uuid, c.properties)
            raise ConnectionError("BuWizz application characteristic not found.")

        await client.start_notify(app, on_status)
        await send(client, app, bytes([CMD_XFER_PERIOD, 50]))          # 20 Hz reports
        await send(client, app, bytes([CMD_MOTOR_TIMEOUT, 0]))          # brake on timeout
        await feed_watchdog(client, app)          # arm watchdog (re-armed while driving)
        await asyncio.sleep(0.3)
    except BaseException:
        await disconnect(client)
        raise
    return client, app


async def disconnect(client):
    """Close the connection on purpose (without the 'unexpected' warning)."""
    disconnect_info["reason"] = "disconnected on request"
    await client.disconnect()


async def home_steering(client, char, full=False):
    """Find the steering center for this connection's encoder zero. Normally
    that comes from the motor's absolute angle and the cached center (no wheel
    movement beyond straightening). A full two-stop calibration runs instead,
    re-learning and caching the center, when `full` is set, when nothing is
    cached yet, or when the absolute angle can't be used. Returns True if the
    steering is centered."""
    log_steer_angles("before homing")
    cached = load_cache()
    if full:
        print("Full steering calibration requested...")
    elif cached is not None and cached["center_abs"] is not None:
        if await abs_home(client, char, cached["half"], cached["center_abs"],
                          cached["mid_abs"]):
            log_steer_angles("at center")
            return True
        print("  Falling back to a full calibration...")
    else:
        print("No cached steering center; running full calibration...")
    trim = cached["trim"] if cached is not None else 0
    if await calibrate_steering(client, char, trim):
        save_cache()
        log_steer_angles("at center")
        return True
    return False


async def drive_session(client, char, selftest=False, duration=MISSION_TIME,
                        accelerate=False):
    """Run the mission (or the self-test) on an already-homed connection, then
    stop the motors. The connection is left open."""
    try:
        if selftest:
            await run_selftest(client, char)
        else:
            await run_mission(client, char, duration, accelerate)
    except ConnectionError as e:
        print(f"\nAborting: {e}")
    finally:
        await stop_and_straighten(client, char)


async def stop_and_straighten(client, char):
    """Stop the motors and straighten the wheels at the end of a session."""
    # Only try to stop the motors if we're still connected; otherwise the
    # write would just raise again and mask the real cause.
    if client.is_connected:
        try:
            await send(client, char, motor_packet({}))  # all stop
            await asyncio.sleep(0.2)
            # Don't leave the wheels turned (the drive stays at zero).
            await servo_to(client, char, steer_target(0.0))
            print("\nDone. Motors stopped, wheels straight.")
        except Exception as e:
            print(f"\nCould not send stop command: {e}")
    else:
        print("\nConnection already lost; brick auto-stops via watchdog.")


async def main(recalibrate=False, selftest=False, duration=MISSION_TIME,
               accelerate=False, use_sensor=True, sensor_mm=None):
    """One complete run: connect, home the steering, run the mission (or the
    self-test, or just a recalibration), stop, and disconnect. `sensor_mm`
    overrides the front-sensor trigger distance when given."""
    global SENSOR_TRIGGER_MM
    if sensor_mm is not None:
        SENSOR_TRIGGER_MM = sensor_mm

    try:
        client, app = await connect()
    except ConnectionError as e:
        print(e)
        return
    try:
        if use_sensor:
            init_sensor()                          # front obstacle sensor (Pi only)

        # --recalibrate: do a full two-stop calibration, cache it, and stop.
        if recalibrate:
            cached = load_cache()
            if await calibrate_steering(client, app,
                                        cached["trim"] if cached else 0):
                save_cache()
                print("\nRecalibration complete.")
            else:
                print("\nRecalibration failed; cache left unchanged.")
            return

        await home_steering(client, app)
        await drive_session(client, app, selftest, duration, accelerate)
    finally:
        await disconnect(client)
