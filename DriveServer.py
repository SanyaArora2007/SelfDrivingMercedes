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

Setup / run:
    python3 -m pip install fastapi uvicorn bleak
    python3 DriveServer.py                        # listen on 0.0.0.0:8000
    python3 DriveServer.py --port 8080

Endpoints:
    WS   /ws/logs                   live server log, one text message per line
    POST /run                       start a mission with the default duration
    POST /run  {"duration": 60}     same as `DriveCar.py --duration 60`
    POST /recenter                  re-find the steering center now
    POST /connect                   connect to the BuWizz and home the steering
    POST /disconnect                close the BuWizz connection

/run replies 202 straight away and the mission runs in the background; its
progress is printed to this server's console, as with the CLI. The others wait
for their work to finish. Every endpoint replies 409 while the car is busy
(there is only one car), and /run and /recenter reply 409 when not connected.

While idle the server keeps feeding the BuWizz watchdog, which otherwise trips
a few seconds after the last command and drops the BLE connection. Only the
watchdog is fed -- no motor commands -- so the car still stops if this process
dies. Console output is timestamped, and also streamed to /ws/logs; this
needs the server started as `python3 DriveServer.py` (not via the uvicorn CLI).
"""

import argparse
import asyncio
import sys
import time
from collections import deque
from contextlib import asynccontextmanager
from typing import Optional

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field

import car_control
from car_control import MISSION_TIME

# The one BuWizz connection, and whether the steering has been homed on it.
car = {"client": None, "char": None, "homed": False}
# Held for anything that uses the car, so requests never overlap.
car_lock = asyncio.Lock()
# The in-flight mission, if any. Kept so it isn't garbage-collected and so a
# request can be refused while the car is busy.
mission_task: Optional[asyncio.Task] = None

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


def busy():
    return car_lock.locked() or (mission_task is not None and not mission_task.done())


def connected():
    return car["client"] is not None and car["client"].is_connected


def check_ready():
    """Refuse a request that needs the car if it's busy or not connected."""
    if busy():
        raise HTTPException(status_code=409, detail="The car is busy.")
    if not connected():
        raise HTTPException(status_code=409,
                            detail="Not connected to the car; POST /connect first.")


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
    yield
    # Shutdown: stop any mission (drive_session stops the motors) and disconnect.
    keepalive_task.cancel()
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


def steering():
    return {"center": car_control.steer_cal["center"],
            "half": car_control.steer_cal["half"]}


@app.websocket("/ws/logs")
async def logs_ws(ws: WebSocket):
    await ws.accept()
    history, q = log_hub.subscribe()

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
        log_hub.unsubscribe(q)


@app.post("/run", status_code=202)
async def run(req: Optional[RunRequest] = None):
    global mission_task
    req = req or RunRequest()
    check_ready()
    mission_task = asyncio.create_task(_run_mission(req.duration))
    return {"status": "started", "duration": req.duration}


@app.post("/recenter")
async def recenter():
    check_ready()
    async with car_lock:
        try:
            car["homed"] = await car_control.home_steering(car["client"], car["char"])
        except ConnectionError as e:
            raise HTTPException(status_code=503, detail=str(e))
    if not car["homed"]:
        raise HTTPException(status_code=500, detail="Steering homing failed.")
    return {"status": "recentered", **steering()}


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
