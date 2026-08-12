#!/home/sanyaarora/buwizz/venv/bin/python3
"""Continuously report distance from any VL53L0X sensor(s) on the I2C bus.

Scans the I2C bus and streams a live reading for every VL53L0X it finds, each
labeled by its I2C address. Works with a single sensor at the default 0x29, and
will also report a second sensor if one is present at a different address
(e.g. 0x30 after address reassignment).

Note: two un-reassigned VL53L0X boards both sit at 0x29 and collide, so only one
would appear -- separating two sensors needs the XSHUT re-addressing step, which
we'll add once a known-good second sensor is fitted.

Run on the device:  ./sensor-test.py     (Ctrl+C to stop)
"""
import time
import board
import adafruit_vl53l0x


def find_sensors(i2c):
    """Return a list of (address, VL53L0X) for every VL53L0X on the bus."""
    while not i2c.try_lock():
        time.sleep(0.01)
    try:
        addrs = i2c.scan()
    finally:
        i2c.unlock()
    sensors = []
    for a in addrs:
        try:
            sensors.append((a, adafruit_vl53l0x.VL53L0X(i2c, address=a)))
        except Exception:
            pass  # something at this address that isn't a VL53L0X
    return sensors


i2c = board.I2C()

# Wait until at least one sensor is present (so the script is useful even if
# started before a sensor is powered/wired).
print("Looking for VL53L0X sensor(s)...", flush=True)
sensors = []
while not sensors:
    sensors = find_sensors(i2c)
    if not sensors:
        print("  none found; retrying (is a sensor powered and wired?)...", flush=True)
        time.sleep(1.0)

print("Found:", ", ".join(hex(a) for a, _ in sensors), flush=True)
print("Reading... Ctrl+C to stop", flush=True)
try:
    while True:
        parts = []
        for addr, tof in sensors:
            try:
                mm = tof.range
                parts.append(f"{hex(addr)}: {mm:>4} mm ({mm / 10:4.1f} cm)")
            except Exception:
                parts.append(f"{hex(addr)}: --read error--")
        print("   |   ".join(parts), flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopped.")
