#!/bin/sh
# (Re)start DriveServer in the background, appending to server.log. If it's
# already running, stop it first and wait for it to exit (on SIGTERM it stops
# the motors and disconnects). Also run at boot from cron (@reboot), so it
# can't rely on the current directory.

cd /home/sanyaarora/SelfDrivingMercedes || exit 1
PATTERN="^venv/bin/python -u DriveServer.py"
STOP_TIMEOUT=15     # seconds to wait for a clean exit before killing it

if pgrep -f "$PATTERN" > /dev/null; then
    echo "Stopping the running server..."
    pkill -f "$PATTERN"
    waited=0
    while pgrep -f "$PATTERN" > /dev/null; do
        if [ "$waited" -ge "$STOP_TIMEOUT" ]; then
            echo "Still running after ${STOP_TIMEOUT} s; killing it."
            pkill -9 -f "$PATTERN"
            sleep 1
            break
        fi
        sleep 1
        waited=$((waited + 1))
    done
fi

# Just after boot, give Bluetooth time to come up before the server scans.
if [ "$(cut -d. -f1 /proc/uptime)" -lt 60 ]; then
    sleep 10
fi

setsid venv/bin/python -u DriveServer.py >> server.log 2>&1 < /dev/null &
echo "Started DriveServer."
