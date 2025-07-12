#!/usr/bin/env python3
import time
import threading
import collections

import matplotlib
matplotlib.use("MacOSX")  # Use the native MacOSX backend for interactive plotting
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from Driver.DriverFunctions.DVS.angle_pos_client import AnglePositionClient

# ─── Configuration (hardcoded) ────────────────────────────────────────────────
WINDOW = 5.0               # length of rolling window in seconds
ENDPOINT = "tcp://localhost:5556"
TIMEOUT_MS = 5             # ZeroMQ recv timeout (ms)
USE_LOCAL_TIME = True      # if True, ignore server timestamp and use local time.time()
POLL_INTERVAL = 0.010       # seconds between successive get_estimate() calls
PLOT_INTERVAL = 100        # milliseconds between plot redraws


# ─── Data collector thread ─────────────────────────────────────────────────────
def collector(client, buffer):
    """
    Polls client.get_estimate() every POLL_INTERVAL seconds and appends
    (timestamp, angle, position) to the shared deque. If USE_LOCAL_TIME is True
    or server timestamp is None, substitute local time.time().
    """
    while True:
        try:
            angle, pos, server_ts = client.get_estimate()
            # choose timestamp: server-supplied vs. local
            if USE_LOCAL_TIME or server_ts is None:
                ts = time.time()
            else:
                ts = float(server_ts)
            buffer.append((ts, angle, pos))
        except TimeoutError:
            # skip silently until we have at least one sample
            print("No response from server, retrying...")
        time.sleep(POLL_INTERVAL)


# ─── Main plotting routine ─────────────────────────────────────────────────────
def main():
    # shared buffer for timestamps, angles, positions
    data_buffer = collections.deque()

    client = AnglePositionClient()
    # start background polling (daemon so it exits with main thread)
    t = threading.Thread(target=collector,
                         args=(client, data_buffer),
                         daemon=True)
    t.start()

    # create two subplots sharing the time axis
    fig, (ax_angle, ax_pos) = plt.subplots(2, 1, sharex=True)
    line_angle, = ax_angle.plot([], [], lw=1.5)
    line_pos,   = ax_pos.plot([], [], lw=1.5)
    ax_angle.set_ylabel("Angle (°)")
    ax_pos.set_ylabel("Position (px)")
    ax_pos.set_xlabel("Time (s)")

    def update(frame):
        """
        Called every PLOT_INTERVAL ms by FuncAnimation:
        1) Prune samples older than (latest_ts - WINDOW)
        2) Shift times so 0 corresponds to window's left edge
        3) Update line data and explicitly rescale both y-axes,
           filtering out any `None` positions/angles.
        """
        if not data_buffer:
            return line_angle, line_pos

        latest_ts = data_buffer[-1][0]
        cutoff = latest_ts - WINDOW

        # drop old samples efficiently from the left
        while data_buffer and data_buffer[0][0] < cutoff:
            data_buffer.popleft()

        # build relative time axis and raw series (may include None)
        times     = [(ts - latest_ts + WINDOW) for ts, _, _ in data_buffer]
        angles    = [angle for _, angle, _ in data_buffer]
        positions = [pos   for _, _, pos in data_buffer]

        # set data—even if some entries are None, Matplotlib will gap them
        line_angle.set_data(times, angles)
        line_pos.  set_data(times, positions)

        ax_angle.set_xlim(0, WINDOW)
        ax_pos.  set_xlim(0, WINDOW)

        # ─── filter out None before bounding ──────────────────────────────
        valid_angles    = [a for a in angles    if a is not None]
        valid_positions = [p for p in positions if p is not None]

        # dynamic y-scaling for angle
        ax_angle.set_ylim(-180, 180)

        # dynamic y-scaling for position
        if valid_positions:
            pmin, pmax = min(valid_positions), max(valid_positions)
            padp = (pmax - pmin) * 0.1 if pmax != pmin else 1.0
            ax_pos.set_ylim(pmin - padp, pmax + padp)

        return line_angle, line_pos


    ani = FuncAnimation(
        fig,
        update,
        interval=PLOT_INTERVAL,
        blit=False,                # keep full redraw; MacOSX backend
        cache_frame_data=False,
    )

    try:
        plt.tight_layout()
        plt.show()
    finally:
        client.close()


if __name__ == "__main__":
    main()
