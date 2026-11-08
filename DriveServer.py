#!/opt/anaconda3/bin/python3

"""
DriveServer.py  -  Self-Driving Mercedes: web server front end

A small FastAPI server that drives the car over HTTP, built on the same
car_control library as the DriveCar.py CLI.

Unlike the CLI, the server connects to the BuWizz at startup and keeps that
connection open, so the steering center is found once per connection rather
than before every run. The connection can be closed and reopened with
/disconnect and /connect; while there is no connection (including after an
unexpected drop), /run and /recenter are refused.

The web page at / can also drive the car by hand over /ws/control: one page at
a time takes control ("engage"), then steps the speed and steering up and down
(see manual_control.py). While under manual control /run, /connect and
/disconnect are refused, but /recenter still works.

Setup / run:
    python3 -m pip install fastapi uvicorn bleak
    python3 DriveServer.py                        # listen on 0.0.0.0:8000
    python3 DriveServer.py --port 8080

Endpoints:
    GET  /                          web page: control pad, telemetry and the logs
    WS   /ws/logs                   live server log, one text message per line
    WS   /ws/telemetry              live telemetry log (see telemetry.py), likewise
    WS   /ws/control                manual driving (JSON commands) + live car state
    GET  /state                     the car's current state and telemetry
    POST /run                       start a mission with the default duration
    POST /run  {"duration": 60}     same as `DriveCar.py --duration 60`
    POST /recenter                  re-calibrate the steering (both end stops)
    POST /trim {"degrees": 5}       move straight ahead 5 motor degrees right
    POST /connect                   connect to the BuWizz and home the steering
    POST /disconnect                close the BuWizz connection

/run replies 202 straight away and the mission runs in the background; its
progress is printed to this server's console, as with the CLI. The others wait
for their work to finish. Every endpoint replies 409 while the car is busy
(there is only one car), and /run and /recenter reply 409 when not connected.

/ws/control messages, page -> server, as {"cmd": ...}:
    engage, release              take / hand back manual control
    speed_up, slow_down          one speed level (-6 reverse .. 0 .. +6 forward)
    turn_right, turn_left        one steering level (-2 left .. 0 .. +2 right)
    forward_left, forward_right,
    reverse_left, reverse_right  one speed and one steering level together
    all_stop                     speed to 0
    straight                     steering to 0
    set_speed + "level": n       jump straight to speed level n
    set_steer + "level": n       jump straight to steering level n
    ping                         heartbeat; manual driving stops without one
Server -> page: {"type": "state", ...} (as GET /state, plus whether this page
holds control) a few times a second, and {"type": "error", "msg": ...}.

While idle the server keeps feeding the BuWizz watchdog, which otherwise trips
a few seconds after the last command and drops the BLE connection. Only the
watchdog is fed -- no motor commands -- so the car still stops if this process
dies. Console output is timestamped, and also streamed to the web page; this
needs the server started as `python3 DriveServer.py` (not via the uvicorn CLI).

Every driving command is also a CONTROL event in telemetry.log, named after the
page's button (or bar) it comes from; a monitor adds STATUS events for what the
car actually does (see telemetry.py).
"""

import argparse
import asyncio
import json
import os
import sys
import time
from collections import deque
from contextlib import asynccontextmanager
from typing import Optional

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

import car_control
import manual_control
from car_control import MISSION_TIME
from manual_control import ManualDriver
from telemetry import Telemetry

# The one BuWizz connection, and whether the steering has been homed on it.
car = {"client": None, "char": None, "homed": False}
# Held for anything that uses the car, so requests never overlap.
car_lock = asyncio.Lock()
# The in-flight mission, if any. Kept so it isn't garbage-collected and so a
# request can be refused while the car is busy.
mission_task: Optional[asyncio.Task] = None
# Manual control, while engaged: the driver, its send-loop task (which holds
# car_lock) and the /ws/control socket that controls it.
manual = {"driver": None, "task": None, "ws": None}

STATE_PERIOD = 0.2      # seconds between state pushes on /ws/control
# /ws/control driving commands, one per button on the page's control pad: the
# telemetry event each is logged as, and the ManualDriver methods it calls.
PAD_COMMANDS = {
    "speed_up":      ("FORWARD",       ("speed_up",)),
    "slow_down":     ("REVERSE",       ("slow_down",)),
    "turn_left":     ("LEFT",          ("turn_left",)),
    "turn_right":    ("RIGHT",         ("turn_right",)),
    "forward_left":  ("FORWARD_LEFT",  ("speed_up", "turn_left")),
    "forward_right": ("FORWARD_RIGHT", ("speed_up", "turn_right")),
    "reverse_left":  ("REVERSE_LEFT",  ("slow_down", "turn_left")),
    "reverse_right": ("REVERSE_RIGHT", ("slow_down", "turn_right")),
    "all_stop":      ("STOP",          ("all_stop",)),
    "straight":      ("STRAIGHT",      ("straighten",)),
}
# ... and those that take a "level" argument (clicks on the speed and steering
# bars), likewise.
LEVEL_COMMANDS = {"set_speed": "SPEED_BAR", "set_steer": "STEER_BAR"}

INDEX_HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "web", "index.html")


class LogHub:
    """Fans complete console lines out to WebSocket viewers, keeping recent
    history for viewers that join later."""

    def __init__(self, history=500, backlog=1000):
        self.history = deque(maxlen=history)
        self.backlog = backlog
        self.queues = set()

    def publish(self, line):
        self.history.append(line)
        for q in self.queues:
            try:
                q.put_nowait(line)
            except asyncio.QueueFull:
                pass                               # a stalled viewer misses lines

    def subscribe(self):
        """Return (recent history, queue of new lines). No await in here, so no
        line can land in both."""
        q = asyncio.Queue(maxsize=self.backlog)
        self.queues.add(q)
        return list(self.history), q

    def unsubscribe(self, q):
        self.queues.discard(q)


log_hub = LogHub()
telemetry_hub = LogHub()
telemetry = Telemetry(publish=telemetry_hub.publish)


class Timestamped:
    """Wraps a text stream so every line written to it starts with HH:MM:SS,
    and passes each complete line on to the log hub."""

    def __init__(self, stream):
        self.stream = stream
        self.at_line_start = True
        self.partial = ""

    def write(self, text):
        out = []
        for line in text.splitlines(keepends=True):
            if self.at_line_start:
                line = time.strftime("%H:%M:%S ") + line
            out.append(line)
            self.at_line_start = line.endswith("\n")
            if self.at_line_start:
                log_hub.publish(self.partial + line.rstrip("\n"))
                self.partial = ""
            else:
                self.partial += line
        self.stream.write("".join(out))
        return len(text)

    def __getattr__(self, name):
        return getattr(self.stream, name)


def mission_active():
    return mission_task is not None and not mission_task.done()


def manual_active():
    return manual["task"] is not None and not manual["task"].done()


def busy():
    return car_lock.locked() or mission_active() or manual_active()


def connected():
    return car["client"] is not None and car["client"].is_connected


def not_ready_reason():
    """Why a request that needs the car can't have it now, or None."""
    if manual_active():
        return "The car is under manual control."
    if busy():
        return "The car is busy."
    if not connected():
        return "Not connected to the car; POST /connect first."
    return None


def check_ready():
    """Refuse a request that needs the car if it's busy or not connected."""
    reason = not_ready_reason()
    if reason:
        raise HTTPException(status_code=409, detail=reason)


async def connect_and_home():
    """Connect to the BuWizz and find the steering center. A new connection
    may mean a new encoder zero, so it is always homed. Caller holds car_lock.
    Raises ConnectionError if the car can't be reached."""
    car["client"], car["char"] = await car_control.connect()
    car["homed"] = await car_control.home_steering(car["client"], car["char"])
    return car["homed"]


async def disconnect():
    """Close the connection if there is one. Caller holds car_lock."""
    client = car["client"]
    car.update(client=None, char=None, homed=False)
    if client is not None and client.is_connected:
        await car_control.disconnect(client)


async def _run_mission(duration):
    async with car_lock:
        try:
            await car_control.drive_session(car["client"], car["char"],
                                            duration=duration)
        except Exception as e:
            print(f"\nMission failed: {e!r}")


async def _run_manual(driver):
    async with car_lock:
        try:
            await driver.run()
        except Exception as e:
            print(f"\nManual control failed: {e!r}")


async def release_manual():
    """End manual control, if engaged, and wait for the motors to stop."""
    task, driver = manual["task"], manual["driver"]
    manual.update(driver=None, task=None, ws=None)
    if driver is not None:
        driver.request_stop()
    if task is not None:
        await asyncio.gather(task, return_exceptions=True)


def car_state():
    """Link, mode and telemetry, as served by GET /state."""
    if manual_active():
        mode = "manual"
    elif mission_active():
        mode = "mission"
    elif connected():
        mode = "idle"
    else:
        mode = "disconnected"
    driver = manual["driver"] if manual_active() else None
    return {
        "connected": connected(),
        "homed": car["homed"],
        "mode": mode,
        "busy": busy(),
        "manual": driver.state() if driver else None,
        "limits": {"speed_levels": manual_control.SPEED_LEVELS,
                   "steer_levels": manual_control.STEER_LEVELS},
        **manual_control.telemetry(connected()),
    }


def telemetry_active():
    """Whether the car's readings are worth logging as STATUS events: connected
    and homed, and not probing the stops right now (re-homing holds car_lock
    outside a mission or manual control, or flags the manual driver)."""
    if not connected() or not car["homed"]:
        return False
    if manual_active():
        return not manual["driver"].homing
    return mission_active() or not car_lock.locked()


async def keepalive():
    """Feed the watchdog while connected so an idle connection isn't dropped.
    Missions feed it themselves too; the extra feeds are harmless."""
    while True:
        await asyncio.sleep(car_control.WATCHDOG_FEED)
        client = car["client"]
        if client is not None and client.is_connected:
            try:
                await car_control.feed_watchdog(client, car["char"])
            except Exception as e:
                print(f"  [!] keepalive: {e!r}")


@asynccontextmanager
async def lifespan(_app):
    car_control.init_sensor()                      # front obstacle sensor (Pi only)
    async with car_lock:
        try:
            if not await connect_and_home():
                print("Startup: steering homing failed; try POST /recenter.")
        except ConnectionError as e:
            print(f"Startup: {e} Not connected; try POST /connect.")
    keepalive_task = asyncio.create_task(keepalive())
    monitor_task = asyncio.create_task(telemetry.monitor(telemetry_active))
    yield
    # Shutdown: stop any mission (drive_session stops the motors) and disconnect.
    keepalive_task.cancel()
    monitor_task.cancel()
    await release_manual()
    if mission_task is not None and not mission_task.done():
        mission_task.cancel()
        await asyncio.gather(mission_task, return_exceptions=True)
    await disconnect()


app = FastAPI(title="Self-Driving Mercedes", lifespan=lifespan)


class RunRequest(BaseModel):
    duration: float = Field(
        MISSION_TIME, gt=0,
        description=f"Mission length in seconds (default {MISSION_TIME:.0f}), "
                    f"same as `DriveCar.py --duration`.")


class TrimRequest(BaseModel):
    degrees: int = Field(
        ..., ge=-30, le=30,
        description="Motor degrees to move straight ahead by (+ = right); "
                    "about 6 motor degrees per degree at the wheels.")


def steering():
    cal = car_control.steer_cal
    return {"trim": car_control.steer_trim(),
            **{k: cal[k] for k in ("center", "half", "lock_left", "lock_right")}}


@app.get("/", response_class=HTMLResponse, include_in_schema=False)
async def index():
    with open(INDEX_HTML) as f:
        return f.read()


@app.websocket("/ws/logs")
async def logs_ws(ws: WebSocket):
    await stream_hub(ws, log_hub)


@app.websocket("/ws/telemetry")
async def telemetry_ws(ws: WebSocket):
    await stream_hub(ws, telemetry_hub)


async def stream_hub(ws, hub):
    """Send a hub's recent history, then each new line, until the page goes."""
    await ws.accept()
    history, q = hub.subscribe()

    async def pump():
        for line in history:
            await ws.send_text(line)
        while True:
            await ws.send_text(await q.get())

    pump_task = asyncio.create_task(pump())
    try:
        # Waiting on the socket notices a closed page right away; otherwise
        # we'd only find out on the next send. Incoming text is ignored.
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass                                       # viewer closed the page
    finally:
        pump_task.cancel()
        hub.unsubscribe(q)


@app.websocket("/ws/control")
async def control_ws(ws: WebSocket):
    await ws.accept()
    # Only the sender task writes to the socket, so sends never overlap.
    # The receive loop queues replies for it; "state" means send a fresh one.
    outbox = asyncio.Queue()

    async def sender():
        while True:
            try:
                msg = await asyncio.wait_for(outbox.get(), STATE_PERIOD)
            except asyncio.TimeoutError:
                msg = "state"
            if msg == "state":
                msg = {"type": "state", **car_state(),
                       "controller": manual["ws"] is ws and manual_active()}
            await ws.send_json(msg)

    sender_task = asyncio.create_task(sender())
    try:
        while True:
            text = await ws.receive_text()
            try:
                msg = json.loads(text)
                cmd = msg.get("cmd")
            except (ValueError, AttributeError):
                msg, cmd = {}, None
            error = await control_command(ws, cmd, msg)
            if error:
                outbox.put_nowait({"type": "error", "msg": error})
            if cmd != "ping":
                outbox.put_nowait("state")
    except WebSocketDisconnect:
        pass                                       # page closed
    finally:
        sender_task.cancel()
        if manual["ws"] is ws:
            await release_manual()                 # don't drive on unattended


async def control_command(ws, cmd, msg):
    """Carry out one /ws/control command (the parsed message is `msg`).
    Returns an error message or None."""
    driver = manual["driver"] if manual_active() else None
    in_control = driver is not None and manual["ws"] is ws
    if in_control:
        driver.heard_from()                        # any message feeds the dead-man
    if cmd == "ping":
        return None
    if cmd == "engage":
        if in_control:
            return None
        if driver is not None:
            return "Another page has control of the car."
        reason = not_ready_reason()
        if reason:
            return reason
        if not car["homed"]:
            return "The steering isn't homed; POST /recenter first."
        driver = ManualDriver(car["client"], car["char"],
                              on_safety_stop=log_safety_stop)
        manual.update(driver=driver, ws=ws,
                      task=asyncio.create_task(_run_manual(driver)))
        return None
    if cmd == "release":
        if in_control:
            await release_manual()
        return None
    if cmd not in PAD_COMMANDS and cmd not in LEVEL_COMMANDS:
        return f"Unknown command: {cmd!r}"
    if not in_control:
        return "Take control first."
    before = (driver.speed, driver.steer)
    if cmd in LEVEL_COMMANDS:
        level = msg.get("level")
        if type(level) is not int:
            return f"{cmd} needs an integer \"level\"."
        getattr(driver, cmd)(level)                # e.g. driver.set_speed(3)
        event, detail = LEVEL_COMMANDS[cmd], f"level {_lvl(level)}; "
    else:
        event, methods = PAD_COMMANDS[cmd]
        for method in methods:
            getattr(driver, method)()              # e.g. driver.speed_up()
        detail = ""
    telemetry.control(event, detail
                      + f"speed {_lvl(before[0])} -> {_lvl(driver.speed)}, "
                      + f"steer {_lvl(before[1])} -> {_lvl(driver.steer)}")
    return None


def log_safety_stop(reason, speed):
    telemetry.status("SAFETY_STOP", f"{reason}; speed {_lvl(speed)} -> 0")


def _lvl(level):
    """A signed level as shown in the telemetry log: +2, 0, -1."""
    return f"{level:+d}" if level else "0"


@app.get("/state")
async def state():
    return car_state()


@app.post("/run", status_code=202)
async def run(req: Optional[RunRequest] = None):
    global mission_task
    req = req or RunRequest()
    check_ready()
    mission_task = asyncio.create_task(_run_mission(req.duration))
    return {"status": "started", "duration": req.duration}


@app.post("/recenter")
async def recenter():
    try:
        if manual_active():
            # The manual driver holds the car; it pauses itself to re-home.
            car["homed"] = await manual["driver"].recenter()
        else:
            check_ready()
            async with car_lock:
                car["homed"] = await car_control.home_steering(car["client"],
                                                               car["char"], full=True)
    except ConnectionError as e:
        raise HTTPException(status_code=503, detail=str(e))
    if not car["homed"]:
        raise HTTPException(status_code=500, detail="Steering homing failed.")
    return {"status": "recentered", **steering()}


@app.post("/trim")
async def trim(req: TrimRequest):
    """Adjust straight ahead live; the new trim is saved for later sessions."""
    if not connected() or not car["homed"]:
        raise HTTPException(status_code=409,
                            detail="Connect and home the steering first.")
    if manual_active():
        # The manual driver re-sends the steering target 20 times a second,
        # so the wheels follow the new straight ahead on their own.
        car_control.trim_steering(req.degrees)
    else:
        check_ready()
        async with car_lock:
            car_control.trim_steering(req.degrees)
            try:
                await car_control.servo_to(car["client"], car["char"],
                                           car_control.steer_target(0.0))
            except ConnectionError as e:
                raise HTTPException(status_code=503, detail=str(e))
    return {"status": "trimmed", **steering()}


@app.post("/connect")
async def connect():
    if busy():
        raise HTTPException(status_code=409, detail="The car is busy.")
    if connected():
        return {"status": "already connected"}
    async with car_lock:
        await disconnect()                         # clear a dropped connection
        try:
            homed = await connect_and_home()
        except ConnectionError as e:
            raise HTTPException(status_code=503, detail=str(e))
    if not homed:
        raise HTTPException(status_code=500,
                            detail="Connected, but steering homing failed; "
                                   "try POST /recenter.")
    return {"status": "connected", **steering()}


@app.post("/disconnect")
async def disconnect_endpoint():
    if busy():
        raise HTTPException(status_code=409, detail="The car is busy.")
    if not connected():
        return {"status": "already disconnected"}
    async with car_lock:
        await disconnect()
    print("Disconnected on request.")
    return {"status": "disconnected"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Web server for the self-driving BuWizz 3.0 car.")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Interface to listen on (default 0.0.0.0).")
    parser.add_argument("--port", type=int, default=8000,
                        help="Port to listen on (default 8000).")
    args = parser.parse_args()
    sys.stdout = Timestamped(sys.stdout)
    sys.stderr = Timestamped(sys.stderr)
    uvicorn.run(app, host=args.host, port=args.port)
