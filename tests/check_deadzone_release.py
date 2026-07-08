"""Check MPC experiment recordings for dead-zone release artifacts.

The firmware extrapolates the angle while the FPGA reports rail contamination
(potentiometer gap, +1.28..+1.58 rad on this rig). On downward crossings the
analog reading keeps settling for ~15-20 ms after the rail flag clears; a
freeze that releases into that ramp injects a large angle snap against the
motion and a derivative sign flip (seen in CPP_mpc__2026-07-08_14-15-36 at
t=1.23 s and t=20.62 s). The settling hold added to
treat_deadangle_with_derivative() must remove those.

For every invalid_steps episode end, this script checks the next few polls:
  - consecutive angle steps must stay below a physical-motion bound plus the
    firmware settling tolerance,
  - angleD must not flip sign right after release.

Usage:
  python tests/check_deadzone_release.py Driver/ExperimentRecordings/CPP_mpc__<...>.csv [more.csv]

Exit code 0 if all files pass, 1 otherwise.
"""

import sys

import numpy as np
import pandas as pd

PHYSICAL_RATE_LIMIT_RAD_S = 30.0  # generous bound on true pole motion
SETTLING_MARGIN_RAD = 0.20        # firmware settling tolerance (~0.15 rad) + drift
POST_RELEASE_POLLS = 5


def wrap(d):
    d = np.where(d > np.pi, d - 2 * np.pi, d)
    return np.where(d < -np.pi, d + 2 * np.pi, d)


def check(csv_path):
    df = pd.read_csv(csv_path, comment="#")
    t = df["time"].to_numpy()
    t = t - t[0]
    ang = df["angle"].to_numpy()
    angD = df["angleD"].to_numpy()
    flagged = df["invalid_steps"].to_numpy() > 0
    dt = float(np.median(np.diff(t)))
    step_limit = PHYSICAL_RATE_LIMIT_RAD_S * dt + SETTLING_MARGIN_RAD

    ends = np.where(flagged[:-1] & ~flagged[1:])[0]
    print(f"{csv_path}:")
    print(f"  {int(flagged.sum())} flagged polls, {len(ends)} episode ends, "
          f"dt={dt * 1e3:.1f} ms, post-release step limit {step_limit:.2f} rad")

    ok = True
    if not flagged.any():
        print("  WARNING: invalid_steps never fired — dead-zone firmware not active "
              "on the board, or the pole never crossed the gap")
        ok = False

    for e in ends:
        steps = wrap(np.diff(ang[e:e + POST_RELEASE_POLLS + 1]))
        worst = float(np.abs(steps).max()) if steps.size else 0.0
        d_after = angD[min(e + POST_RELEASE_POLLS - 1, len(angD) - 1)]
        flip = np.sign(angD[e]) != np.sign(d_after) and abs(angD[e]) > 1.0 and abs(d_after) > 1.0
        if worst > step_limit or flip:
            ok = False
            print(f"  BAD release at t={t[e]:.2f}s angle={ang[e]:+.2f}: "
                  f"max post-release step {worst:.2f} rad, "
                  f"angleD {angD[e]:+.1f} -> {d_after:+.1f}")

    print("  => PASS" if ok else "  => FAIL")
    return ok


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    return 0 if all([check(p) for p in argv]) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
