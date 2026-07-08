"""IO thread + main-thread compute scheduling for physical cartpole control.

Thread roles:
  - IO thread (driver ``_io_loop``): calls :meth:`tick` every polling period.
    Evaluates the Secloc gate (``should_trigger``) when idle, snapshots state,
    applies control (ZOH or fresh on deadline), never runs compute.
  - Main thread: :meth:`run_compute_loop` runs ``compute_step`` / ``step``.f

On a trigger the state is snapshotted and main computes in the background. On the
DEADLINE tick - ``apply_window_polling_loops - 1`` polling loops after the trigger,
i.e. the trigger tick itself when the window equals one polling period -
:meth:`tick` BLOCKS until the computation is finished and returns the fresh
control, so it leaves for the actuator in that same loop iteration. Between
trigger and deadline the previous control is held (zero-order hold).

Scheduling is counted in ticks, not measured timestamps, so it is deterministic.

The wrapper does not proxy controller attributes; the driver keeps its direct
controller reference for everything but the per-iteration control.
"""
import threading
import time as _time

import numpy as np


class SplitControlLoop:
    def __init__(self, controller, apply_window_polling_loops, polling_period_s, name="controller"):
        """
        :param controller: a template_controller instance. Controllers without
            should_trigger/compute_step fall back to always-trigger + step().
        :param apply_window_polling_loops: trigger-to-apply latency, expressed as a
            number of polling loops.
        :param polling_period_s: nominal polling-loop period in seconds; only used
            for the overrun statistic and the status string, not for scheduling.
        :param name: label used in the status string.
        """
        self.controller = controller
        self.apply_window_polling_loops = int(apply_window_polling_loops)
        if self.apply_window_polling_loops < 1:
            raise ValueError(
                f"apply_window_polling_loops must be >= 1 (got {apply_window_polling_loops})"
            )
        self.apply_window_s = self.apply_window_polling_loops * float(polling_period_s)
        self.name = name

        self._has_split = (
            hasattr(controller, "should_trigger")
            and hasattr(controller, "compute_step")
        )

        self._lock = threading.Lock()
        self._applied_Q = 0.0
        self._snapshot = None
        self._result_Q = 0.0
        self._result_pending = False
        self._busy = False
        self._polling_loops_since_trigger = 0
        self._last_applied_now = False
        self._generation = 0

        self._calc_count = 0
        self._calc_time_sum = 0.0
        self._calc_time_max = 0.0
        self._calc_time_last = 0.0
        self._overrun_count = 0

        self._wake = threading.Event()
        self._compute_done = threading.Event()
        self._stop = threading.Event()

    def stop(self):
        """Signal :meth:`run_compute_loop` to exit. Safe to call from any thread."""
        self._stop.set()
        self._wake.set()

    def prepare_for_run(self):
        """Clear stop flag before starting a new experiment."""
        self._stop.clear()

    def reset(self):
        """Drop any in-flight/pending computation and clear the held output.

        Does not reset the wrapped controller; the driver does that separately.
        """
        with self._lock:
            self._generation += 1
            self._applied_Q = 0.0
            self._result_Q = 0.0
            self._result_pending = False
            was_busy = self._busy
            self._busy = False
            self._polling_loops_since_trigger = 0
            self._snapshot = None
            self._last_applied_now = False
        self._wake.clear()
        if was_busy:
            # Unblock IO waiting on deadline if main is mid-compute (result will be discarded).
            self._compute_done.set()

    # ------------------------------------------------------------------ IO thread
    def tick(self, s, time=None, updated_attributes=None):
        """Called every polling-loop iteration on the IO thread. Returns Q to apply.

        Non-blocking except on the deadline tick, where it waits for main to
        finish so the fresh control is returned within the same loop iteration.
        """
        if updated_attributes is None:
            updated_attributes = {}

        if hasattr(self.controller, "peek_secloc_gate"):
            self.controller.peek_secloc_gate(s, time=time, updated_attributes=updated_attributes)

        with self._lock:
            if self._busy or self._result_pending:
                self._polling_loops_since_trigger += 1
            idle = (not self._busy) and (not self._result_pending)

        if idle and self._should_trigger(s, time, updated_attributes):
            with self._lock:
                self._snapshot = (
                    np.array(s, copy=True),
                    time,
                    dict(updated_attributes),
                )
                self._polling_loops_since_trigger = 0
                self._busy = True
            self._compute_done.clear()
            self._wake.set()

        applied_now = False
        with self._lock:
            deadline_reached = (
                (self._busy or self._result_pending)
                and self._polling_loops_since_trigger >= self.apply_window_polling_loops - 1
            )
        if deadline_reached:
            self._wait_for_computation()
            with self._lock:
                if self._result_pending:
                    self._applied_Q = self._result_Q
                    self._result_pending = False
                    applied_now = True

        with self._lock:
            self._last_applied_now = applied_now
            q = self._applied_Q
        return q

    def _wait_for_computation(self):
        timeout_s = max(1.0, 10.0 * self.apply_window_s)
        if not self._compute_done.wait(timeout=timeout_s):
            print(
                f"[{self.name}] controller computation did not finish within "
                f"{timeout_s:.1f}s; holding previous control",
                flush=True,
            )

    def _should_trigger(self, s, time, updated_attributes):
        if self._has_split:
            return bool(
                self.controller.should_trigger(
                    s, time=time, updated_attributes=updated_attributes
                )
            )
        return True

    # ------------------------------------------------------------------ main thread
    def run_compute_loop(self):
        """Main thread only. Blocks until :meth:`stop` is called."""
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
                self._compute_done.set()
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
            except Exception as exc:
                print(f"[{self.name}] controller computation failed: {exc}", flush=True)
                ok = False
                q = None
            dt = _time.time() - t0

            discard = False
            with self._lock:
                if gen_local != self._generation:
                    self._busy = False
                    discard = True
                else:
                    self._calc_count += 1
                    self._calc_time_last = dt
                    self._calc_time_sum += dt
                    self._calc_time_max = max(self._calc_time_max, dt)
                    if dt > self.apply_window_s:
                        self._overrun_count += 1
                    if ok:
                        self._result_Q = q
                        self._result_pending = True
                    self._busy = False
            self._compute_done.set()
            if discard:
                continue

    # ------------------------------------------------------------------ reporting
    @property
    def last_applied_now(self):
        with self._lock:
            return self._last_applied_now

    @property
    def is_busy(self):
        with self._lock:
            return self._busy or self._result_pending

    @property
    def applied_Q(self):
        with self._lock:
            return self._applied_Q

    @property
    def calc_time_last(self):
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
            f"window={self.apply_window_s * 1000:.0f}ms overruns={overruns}/{count} "
            f"(busy={int(busy)}, pending={int(pending)})"
        )
