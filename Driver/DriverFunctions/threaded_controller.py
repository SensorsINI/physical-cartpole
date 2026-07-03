"""Controller running in a worker thread, its output applied on a fixed clock.

The chip-polling loop calls :meth:`tick` every period. The gate
(``controller.should_trigger``, cheap) is evaluated in the loop thread, and only
while the worker is idle - so gate and computation never run concurrently. On a
trigger the state is snapshotted, the worker computes, and the result is committed
at the first tick with ``time >= t_trigger + window`` (zero-order hold until then;
an overrunning computation is committed as soon as it is ready).

The wrapper does not proxy controller attributes; the driver keeps its direct
controller reference for everything but the per-iteration control.
"""
import threading
import time as _time

import numpy as np

from DriverFunctions.cpu_affinity import set_thread_cpu_affinity


class ThreadedController:
    def __init__(self, controller, window_s, name="controller", cpu_affinity=""):
        """
        :param controller: a template_controller instance. Controllers without
            should_trigger/compute_step fall back to always-trigger + step().
        :param window_s: fixed trigger-to-apply latency in seconds.
        :param name: label used in the status string.
        :param cpu_affinity: core spec (e.g. "2") the worker thread pins itself to.
        """
        self.controller = controller
        self.window_s = float(window_s)
        self.name = name
        self.cpu_affinity = cpu_affinity

        self._has_split = (
            hasattr(controller, "should_trigger")
            and hasattr(controller, "compute_step")
        )

        # Shared state (guarded by _lock)
        self._lock = threading.Lock()
        self._applied_Q = 0.0
        self._snapshot = None
        self._result_Q = 0.0
        self._result_pending = False
        self._busy = False
        self._t_apply = None
        self._last_applied_now = False
        # Bumped on reset so an in-flight computation is discarded when it completes.
        self._generation = 0

        # Statistics
        self._calc_count = 0
        self._calc_time_sum = 0.0
        self._calc_time_max = 0.0
        self._calc_time_last = 0.0
        self._overrun_count = 0

        # Worker control
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = None

    # ------------------------------------------------------------------ lifecycle
    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run_loop, name=f"{self.name}_controller_worker", daemon=True
        )
        self._thread.start()

    def stop(self, join_timeout=2.0):
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout)
        self._thread = None

    def reset(self):
        """Drop any in-flight/pending computation and clear the held output.

        Does not reset the wrapped controller; the driver does that separately.
        """
        with self._lock:
            self._generation += 1
            self._applied_Q = 0.0
            self._result_Q = 0.0
            self._result_pending = False
            self._busy = False
            self._t_apply = None
            self._snapshot = None
            self._last_applied_now = False
        self._wake.clear()

    # ------------------------------------------------------------------ loop side
    def tick(self, s, time=None, updated_attributes=None):
        """Called every polling-loop iteration. Returns the control to apply now."""
        if updated_attributes is None:
            updated_attributes = {}

        applied_now = False
        with self._lock:
            if self._result_pending and time is not None and self._t_apply is not None \
                    and time >= self._t_apply:
                self._applied_Q = self._result_Q
                self._result_pending = False
                applied_now = True
            idle = (not self._busy) and (not self._result_pending)
            q = self._applied_Q

        if idle:
            if self._should_trigger(s, time, updated_attributes):
                with self._lock:
                    self._snapshot = (
                        np.array(s, copy=True),
                        time,
                        dict(updated_attributes),
                    )
                    self._t_apply = (time + self.window_s) if time is not None else None
                    self._busy = True
                self._wake.set()

        with self._lock:
            self._last_applied_now = applied_now
        return q

    def _should_trigger(self, s, time, updated_attributes):
        if self._has_split:
            return bool(
                self.controller.should_trigger(
                    s, time=time, updated_attributes=updated_attributes
                )
            )
        return True

    # ------------------------------------------------------------------ worker
    def _run_loop(self):
        set_thread_cpu_affinity(self.cpu_affinity, thread_label=f"{self.name} worker")
        while not self._stop.is_set():
            self._wake.wait()
            self._wake.clear()
            if self._stop.is_set():
                break

            with self._lock:
                snapshot = self._snapshot
                gen_local = self._generation
            if snapshot is None:
                with self._lock:
                    if gen_local == self._generation:
                        self._busy = False
                continue

            s, time, updated_attributes = snapshot
            t0 = _time.time()
            try:
                if self._has_split:
                    q = self.controller.compute_step(
                        s, time=time, updated_attributes=updated_attributes
                    )
                else:
                    q = self.controller.step(
                        s, time=time, updated_attributes=updated_attributes
                    )
                q = float(q)
                ok = True
            except Exception as exc:  # keep the loop alive; hold previous control
                print(f"[{self.name}] controller computation failed: {exc}", flush=True)
                ok = False
                q = None
            dt = _time.time() - t0

            with self._lock:
                if gen_local != self._generation:
                    continue  # reset happened while computing: discard this result
                self._calc_count += 1
                self._calc_time_last = dt
                self._calc_time_sum += dt
                self._calc_time_max = max(self._calc_time_max, dt)
                if dt > self.window_s:
                    self._overrun_count += 1
                if ok:
                    self._result_Q = q
                    self._result_pending = True
                self._busy = False

    # ------------------------------------------------------------------ reporting
    @property
    def last_applied_now(self):
        with self._lock:
            return self._last_applied_now

    @property
    def applied_Q(self):
        with self._lock:
            return self._applied_Q

    @property
    def calc_time_last(self):
        """Duration of the most recently completed computation, in seconds."""
        with self._lock:
            return self._calc_time_last

    @property
    def calc_count(self):
        with self._lock:
            return self._calc_count

    @property
    def overrun_count(self):
        with self._lock:
            return self._overrun_count

    def get_status(self):
        with self._lock:
            count = self._calc_count
            mean_ms = (self._calc_time_sum / count * 1000.0) if count else 0.0
            last_ms = self._calc_time_last * 1000.0
            max_ms = self._calc_time_max * 1000.0
            overruns = self._overrun_count
            busy = self._busy
            pending = self._result_pending
        return (
            f"calc [last={last_ms:.1f}ms mean={mean_ms:.1f}ms max={max_ms:.1f}ms] "
            f"window={self.window_s * 1000:.0f}ms overruns={overruns}/{count} "
            f"(busy={int(busy)}, pending={int(pending)})"
        )
