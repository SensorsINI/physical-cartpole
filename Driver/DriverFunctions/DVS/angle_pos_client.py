import zmq
import json

class AnglePositionClient:
    """
    REQ-side ZeroMQ client for fetching the freshest (angle, position) estimate
    from the companion REP server.
    """

    def __init__(self, endpoint: str = "tcp://localhost:5556", timeout_ms: int = 5):
        """
        Parameters
        ----------
        endpoint : str
            ZeroMQ address where the server is listening (e.g. "tcp://localhost:5556").
        timeout_ms : int
            How long to wait for a reply before raising a TimeoutError (in milliseconds).
        """
        # Use the global Context to avoid terminating it inadvertently.
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.REQ)
        self._sock.connect(endpoint)
        # Enforce receive timeout so recv() will raise zmq.error.Again if overdue.
        self._sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._cached = None  # (angle, position, timestamp)

    def get_estimate(self) -> tuple[float, float, float]:
        """
        Returns the tuple (angle_deg, position_px, timestamp_s).

        If the server does not reply in time, returns the last cached estimate
        to prevent blocking the caller. If no cached value exists, raises TimeoutError.
        """
        try:
            # Any payload is acceptable; "?" is purely conventional.
            self._sock.send(b"?")
            raw = self._sock.recv()
            data = json.loads(raw.decode())
            angle = data.get("angle_deg")
            position = data.get("position_px")
            ts = data.get("timestamp")
            # Cache the fresh reading
            self._cached = (angle, position, ts)
        except zmq.error.Again:
            if self._cached is None:
                raise TimeoutError("No response from estimator and no cached value.")
        return self._cached  # guaranteed non-None here

    def close(self) -> None:
        """Cleanly close the socket (does not terminate the global Context)."""
        self._sock.close(0)
