"""angle_pos_zmq.py
===================
Minimal helper that turns *any* producer of `(angle, position)` pairs into a
ZeroMQ REP service without polluting the producer’s code.  Designed so that the
original camera script only needs **three one‑liners**:

```python
from angle_pos_zmq import publish_estimate, start_zmq_server, stop_zmq_server
...
start_zmq_server()                 # once, after your imports
...
publish_estimate(angle, position)  # every time you have a new measurement
...
stop_zmq_server()                  # just before exiting
```

Nothing else changes.
"""

from __future__ import annotations

import json
import threading
from typing import Optional

import zmq

# ──────────────────────────────────────────────────────────────────────────────
#                               INTERNAL STATE
# ──────────────────────────────────────────────────────────────────────────────
_latest = {
    "timestamp": None,   # float | None
    "angle_deg": None,   # float | None
    "cart_x":           None,   # float | None
    "linear_velocity":  None,   # float | None
    "angular_velocity": None,   # float | None
}
_lock = threading.Lock()

_stop = threading.Event()
_thread: Optional[threading.Thread] = None

# ──────────────────────────────────────────────────────────────────────────────
#                               PUBLIC API
# ──────────────────────────────────────────────────────────────────────────────

def publish_estimate(
    angle_deg:         float | None,
    cart_x:            int   | None,
    linear_velocity:   float | None,
    angular_velocity:  float | None,
    timestamp:         float | None = None,
) -> None:
    import time

    with _lock:
        _latest["timestamp"]   = time.time() if timestamp is None else timestamp
        _latest["angle_deg"]   = angle_deg
        _latest["cart_x"]           = cart_x
        _latest["linear_velocity"]  = linear_velocity
        _latest["angular_velocity"] = angular_velocity


# -----------------------------------------------------------------------------

def start_zmq_server(endpoint: str = "tcp://*:5556", recv_timeout_ms: int = 100) -> None:
    """Launch the REP thread **once**.  Subsequent calls are no‑ops."""
    global _thread
    if _thread and _thread.is_alive():
        return  # already running

    def _serve() -> None:
        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.REP)
        sock.bind(endpoint)
        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)
        print(f"[angle_pos_zmq] 🟢  Serving estimates on {endpoint}")

        try:
            while not _stop.is_set():
                events = dict(poller.poll(recv_timeout_ms))
                if sock in events and events[sock] == zmq.POLLIN:
                    sock.recv()  # we ignore client payload
                    with _lock:
                        payload = json.dumps(_latest, default=lambda o: None).encode()
                    sock.send(payload)
        finally:
            sock.close(0)
            print("[angle_pos_zmq] 🔴  Server stopped.")

    _thread = threading.Thread(target=_serve, name="angle-pos-zmq", daemon=True)
    _thread.start()

# -----------------------------------------------------------------------------

def stop_zmq_server() -> None:
    """Signal the REP thread to stop and wait briefly."""
    _stop.set()
    if _thread:
        _thread.join(timeout=1.0)
