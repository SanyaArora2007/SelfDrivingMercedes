#!/opt/anaconda3/bin/python3

"""
DriveCar.py  -  Self-Driving Mercedes: command-line front end

Parses command-line options and runs the car via car_control.main(). All the
BLE, steering-calibration, and obstacle-avoidance logic lives in car_control.py
(shared with the DriveServer.py web server).

Setup / run:
    python3 -m pip install bleak                 # + adafruit-circuitpython-vl53l0x on the Pi
    python3 DriveCar.py                           # 2-minute avoidance mission
    python3 DriveCar.py --duration 60             # shorter run
    python3 DriveCar.py --accelerate              # speed up on a clear path
    python3 DriveCar.py --sensor-mm 250           # sensor trigger distance (mm)
    python3 DriveCar.py --no-sensor               # stall detection only
    python3 DriveCar.py --recalibrate             # recalibrate steering, then exit
    python3 DriveCar.py --selftest                # fixed motion self-test
"""

import argparse
import asyncio

import car_control
from car_control import MISSION_TIME, SENSOR_TRIGGER_MM


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Self-driving BuWizz 3.0 car: obstacle-avoidance mission "
                    "with cached steering calibration.")
    parser.add_argument(
        "--recalibrate", action="store_true",
        help="Only re-run the full steering calibration and save it, then exit "
             "(skip driving).")
    parser.add_argument(
        "--selftest", action="store_true",
        help="Run the fixed motion self-test instead of the obstacle-avoidance "
             "mission.")
    parser.add_argument(
        "--duration", type=float, default=MISSION_TIME, metavar="SECONDS",
        help=f"How long to run the obstacle-avoidance mission, in seconds "
             f"(default {MISSION_TIME:.0f} = 2 minutes).")
    parser.add_argument(
        "--accelerate", action="store_true",
        help="Speed up while cruising a clear path (off by default).")
    parser.add_argument(
        "--no-sensor", action="store_true",
        help="Ignore the front distance sensor (use velocity-stall detection only).")
    parser.add_argument(
        "--sensor-mm", type=int, default=SENSOR_TRIGGER_MM, metavar="MM",
        help=f"Trigger recovery when the front sensor reads this close, in mm "
             f"(default {SENSOR_TRIGGER_MM}; ~150-300 recommended for real "
             f"avoidance).")
    args = parser.parse_args()
    asyncio.run(car_control.main(recalibrate=args.recalibrate,
                                 selftest=args.selftest,
                                 duration=args.duration,
                                 accelerate=args.accelerate,
                                 use_sensor=not args.no_sensor,
                                 sensor_mm=args.sensor_mm))
