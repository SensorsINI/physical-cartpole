#!/usr/bin/env python3
import time
import threading
import collections

import zmq
import json
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ─── Configuration (hardcoded) ────────────────────────────────────────────────
WINDOW = 5.0               # length of rolling window in seconds
ENDPOINT = "tcp://localhost:5556"
TIMEOUT_MS = 5             # ZeroMQ recv timeout (ms)
USE_LOCAL_TIME = True      # if True, ignore server timestamp and use local time.time()
POLL_INTERVAL = 0.010      # seconds between successive get_estimate() calls
PLOT_INTERVAL = 200        # milliseconds between plot redraws

# ─── Your existing client, unchanged ────────────────────────────────────────────
class AnglePositionClient:
    """
    REQ-side ZeroMQ client for fetching the freshest (angle, position) estimate
    from the companion REP server.
    """
    def __init__(self, endpoint: str = ENDPOINT, timeout_ms: int = TIMEOUT_MS):
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.connect(endpoint)
        self._sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._cached = None  # (angle, position, timestamp)

    def get_estimate(self) -> tuple[float, float, float]:
        """
        Returns (angle_deg, position_px, timestamp_s). If server fails to reply
        in time, returns last cached reading to avoid blocking. Raises if
        no cache exists.
        """
        try:
            self._sock.send(b"?")
            raw = self._sock.recv()
            data = json.loads(raw.decode())
            angle = data.get("angle_deg")
            pos   = data.get("position_px")
            ts    = data.get("timestamp")
            # cache fresh reading
            self._cached = (angle, pos, ts)
        except zmq.error.Again:
            if self._cached is None:
                # no cached value to fall back on
                raise TimeoutError("No response and no cached estimate.")
        return self._cached  # guaranteed non-None here

    def close(self) -> None:
        """Cleanly close socket (does not terminate the global Context)."""
        self._sock.close(0)


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
            pass
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
        3) Update line data and keep x-axis [0, WINDOW]
        """
        if not data_buffer:
            return line_angle, line_pos

        latest_ts = data_buffer[-1][0]
        cutoff = latest_ts - WINDOW

        # drop old samples efficiently from the left
        while data_buffer and data_buffer[0][0] < cutoff:
            data_buffer.popleft()

        # build relative time axis
        times     = [(ts - latest_ts + WINDOW) for ts, _, _ in data_buffer]
        angles    = [angle for _, angle, _ in data_buffer]
        positions = [pos   for _, _, pos in data_buffer]

        line_angle.set_data(times, angles)
        line_pos.set_data(times, positions)

        ax_angle.set_xlim(0, WINDOW)
        ax_pos.set_xlim(0, WINDOW)

        # autoscale y-axes to data range
        ax_angle.relim(); ax_angle.autoscale_view(True, True, False)
        ax_pos.relim();   ax_pos.autoscale_view(True, True, False)

        return line_angle, line_pos

    ani = FuncAnimation(fig, update,
                        interval=PLOT_INTERVAL, blit=True)

    try:
        plt.tight_layout()
        plt.show()
    finally:
        client.close()


if __name__ == "__main__":
    main()
