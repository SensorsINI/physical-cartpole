"""CPU affinity helpers for the control process.

Pinning the process to a single core makes the TensorFlow-based control step far
more time-predictable (no cross-core migration, warm caches). The pin MUST be set
before TensorFlow is imported so that the TF/XLA/Eigen worker threads, which are
spawned on first use, inherit the affinity mask of the main thread. This is why
control.py calls set_control_cpu_affinity() at the very top, before anything
triggers the TensorFlow import.

Kept dependency-free (only the standard library) so it can be imported and run
before the TF environment variables are configured.
"""

import os


def parse_cpu_spec(cpu_spec):
    """Parse a "2", "2,3", or "2-3" style spec into a set of CPU indices."""
    cpus = set()
    for part in str(cpu_spec).split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            cpus.update(range(int(start), int(end) + 1))
        else:
            cpus.add(int(part))
    return cpus


def is_pinned():
    """True if the process is restricted to fewer cores than the machine has."""
    if not hasattr(os, "sched_getaffinity"):
        return False
    try:
        affinity = os.sched_getaffinity(0)
    except OSError:
        return False
    total = os.cpu_count() or len(affinity)
    return len(affinity) < total


def set_thread_cpu_affinity(cpu_spec, thread_label="thread"):
    """Pin the CALLING thread only (Linux). Used to separate the chip-polling loop
    from the controller worker. Returns the applied affinity list, or None."""
    cpu_spec = (cpu_spec or "").strip()
    if cpu_spec.lower() in {"", "none", "off", "false"}:
        return None
    if not hasattr(os, "sched_setaffinity"):
        print(f"{thread_label} CPU affinity: not supported on this platform")
        return None

    requested = parse_cpu_spec(cpu_spec)
    try:
        # Note: this returns/sets affinity of the calling thread, not the whole process.
        available = os.sched_getaffinity(0)
        all_cpus = set(range(os.cpu_count() or 0))
        selected = requested & (all_cpus or requested)
        if not selected:
            print(
                f"{thread_label} CPU affinity: requested {sorted(requested)} not available; "
                f"keeping {sorted(available)}"
            )
            return None
        os.sched_setaffinity(0, selected)
        result = sorted(os.sched_getaffinity(0))
    except OSError as exc:
        print(f"{thread_label} CPU affinity: failed to apply ({exc})")
        return None
    print(f"{thread_label} CPU affinity: {result}")
    return result


def set_control_cpu_affinity(cpu_spec):
    """Pin the current process to ``cpu_spec``.

    Returns the resulting sorted affinity list, or None if nothing was applied
    (disabled, unsupported platform, or none of the requested cores available).
    """
    cpu_spec = (cpu_spec or "").strip()
    if cpu_spec.lower() in {"", "none", "off", "false"}:
        print("Control CPU affinity: disabled")
        return None
    if not hasattr(os, "sched_setaffinity"):
        print("Control CPU affinity: not supported on this platform")
        return None

    requested = parse_cpu_spec(cpu_spec)
    available = os.sched_getaffinity(0)
    selected = requested & available
    if not selected:
        print(
            f"Control CPU affinity: requested {sorted(requested)} not available; "
            f"keeping {sorted(available)}"
        )
        return None

    os.sched_setaffinity(0, selected)
    result = sorted(os.sched_getaffinity(0))
    print(f"Control CPU affinity: {result}")
    return result


def configure_control_cpu_policy(affinity_spec):
    """Decide and apply the pre-TensorFlow CPU/thread policy for the control process.

    MUST be called before TensorFlow is imported: TF reads the thread counts and
    XLA flags at init, and CPU affinity only propagates to the TF/XLA worker threads
    that are spawned *after* the mask is set.

    The policy is driven entirely by the requested affinity spec (from
    globals.CONTROL_CPU_AFFINITY, overridable by CARTPOLE_CONTROL_CPU_AFFINITY):

    - Non-empty spec -> pin to those core(s) AND force single-threaded TF/XLA/OMP.
      This is the time-predictable setup for single-threaded TF optimizers ('rpgd').
    - Empty/"off"    -> nothing is pinned and threading is left at TF's auto default,
      which is what parallel optimizers such as the C/OpenMP backend 'rpgd-c' need
      (a single-core pin would throttle their worker threads).

    So: choose pinning by setting/clearing CONTROL_CPU_AFFINITY for the optimizer you
    run; no optimizer name is needed here. CARTPOLE_TF_THREADS overrides the thread
    count when pinning.

    Returns a dict summarizing the applied policy.
    """
    spec = os.environ.get("CARTPOLE_CONTROL_CPU_AFFINITY", affinity_spec or "")
    affinity = set_control_cpu_affinity(spec)
    pin_single_core = affinity is not None

    if pin_single_core:
        os.environ.setdefault("CARTPOLE_TF_THREADS", "1")
        threads = os.environ["CARTPOLE_TF_THREADS"]
        os.environ.setdefault("CARTPOLE_XLA_CPU_THREADS", threads)
        os.environ.setdefault("TF_NUM_INTRAOP_THREADS", threads)
        os.environ.setdefault("TF_NUM_INTEROP_THREADS", threads)
        os.environ.setdefault("OMP_NUM_THREADS", threads)
        os.environ.setdefault(
            "XLA_FLAGS",
            "--xla_cpu_multi_thread_eigen=false "
            f"--xla_cpu_multi_thread_eigen_thread_count={os.environ['CARTPOLE_XLA_CPU_THREADS']}",
        )

    print(f"Control CPU policy: single-core pin = {pin_single_core}")
    return {
        "pin_single_core": pin_single_core,
        "affinity": affinity,
        "tf_threads": os.environ.get("TF_NUM_INTRAOP_THREADS", "auto (unpinned)"),
        "xla_flags": os.environ.get("XLA_FLAGS"),
    }
