"""Offline analysis of hardware angle filter recordings.

Consumes the .npz files produced by run_filter_experiment.py and produces:
  static  - noise/resolution table per filter configuration (std, effective
            bits gained, glitch counts, agreement with the Python reference
            model replayed on the recorded raw stream) + comparison plots.
  dynamic - swing recordings: hardware filtered vs raw overlay, offline replay
            of alternative configurations on the same raw stream, filter lag
            estimate, and derivative (angular velocity) noise comparison.

Usage (no hardware needed):
  python Driver/DataAnalysis/HardwareFilterTest/analyze_filter_experiment.py
  (or pass explicit .npz paths as arguments)
"""

import argparse
import glob
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from filter_reference_model import filter_stream  # noqa: E402

OUTPUT_DIR = os.path.join(SCRIPT_DIR, "output")

MODE_NAMES = {0: "raw", 1: "median", 2: "trimmed_mean"}

# Configurations replayed offline on recorded raw streams (label, window, trim, mode).
REPLAY_CONFIGS = [
    ("median63_old_design", 63, 0, 1),
    ("average63_trim0",     63, 0, 2),
    ("trimmed63_t7_default", 63, 7, 2),
    ("trimmed63_t15",       63, 15, 2),
]

GLITCH_THRESHOLD_12BIT = 20  # matches MAX_ADC_STEP invalid-jump threshold in firmware


def glitch_count(stream_16bit):
    diffs = np.abs(np.diff(stream_16bit.astype(np.int64))) / 16.0
    return int(np.count_nonzero(diffs > GLITCH_THRESHOLD_12BIT))


def analyze_static(path):
    data = np.load(path, allow_pickle=False)
    labels = [str(l) for l in data["meta_labels"]]
    interval_us = int(data["meta_interval_us"])
    print(f"\n=== Static sweep: {os.path.basename(path)} (interval {interval_us} us) ===")

    # Raw noise baseline from the passthrough recording (raw stream is present
    # in every recording; use the first one).
    raw_baseline = data[f"{labels[0]}__raw"].astype(np.float64)
    raw_std = raw_baseline.std()

    header = (f"{'configuration':24s} {'window':>6s} {'trim':>4s} {'mode':>12s} "
              f"{'std(16b)':>9s} {'std(12b LSB)':>12s} {'bits gained':>11s} "
              f"{'glitches':>8s} {'model dev(16b)':>14s}")
    print(header)
    print("-" * len(header))

    rows = []
    for label in labels:
        filtered = data[f"{label}__filtered"].astype(np.float64)
        raw = data[f"{label}__raw"]
        window, trim, mode = data[f"{label}__config"]
        std16 = filtered.std()
        bits = np.log2(raw_std / std16) if std16 > 0 else float("inf")
        glitches = glitch_count(data[f"{label}__filtered"])
        # Statistical cross-check against the reference model replayed on the
        # simultaneously recorded raw stream. Not sample-exact (hardware slides
        # at the XADC rate, the recording is subsampled): compare means/stds.
        replay = filter_stream(raw, window, trim, mode).astype(np.float64)
        model_dev = abs(replay.mean() - filtered.mean())
        rows.append((label, window, trim, mode, std16, bits, glitches, model_dev))
        print(f"{label:24s} {window:6d} {trim:4d} {MODE_NAMES[int(mode)]:>12s} "
              f"{std16:9.2f} {std16 / 16:12.3f} {bits:11.2f} {glitches:8d} {model_dev:14.2f}")

    print(f"\nBaseline raw std: {raw_std:.2f} (16-bit codes) = {raw_std / 16:.3f} 12-bit LSB")
    print("'bits gained' = log2(std_raw / std_filtered); up to ~3 for a pure average of 63")
    print("white-noise samples (sqrt(63) ~ 8x), higher if the raw stream contains glitches.")
    print("'model dev' = |mean(hardware filtered) - mean(reference model on raw)|;")
    print("should be well below one 12-bit LSB (16 in these units).")

    # Plot: filtered std per configuration + a zoomed time series comparison.
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    names = [r[0] for r in rows]
    stds = [r[4] for r in rows]
    axes[0].barh(names, stds)
    axes[0].set_xlabel("std of filtered output (16-bit codes)")
    axes[0].set_title("Static noise per filter configuration")
    axes[0].invert_yaxis()

    n_show = min(2000, len(raw_baseline))
    t = np.arange(n_show) * interval_us * 1e-3
    axes[1].plot(t, raw_baseline[:n_show], label="raw", alpha=0.4, linewidth=0.7)
    for label in ("median63_old_design", "trimmed63_t7_default"):
        if f"{label}__filtered" in data:
            axes[1].plot(t, data[f"{label}__filtered"][:n_show].astype(float), label=label, linewidth=1.0)
    axes[1].set_xlabel("time [ms]")
    axes[1].set_ylabel("code (16-bit domain)")
    axes[1].set_title("Static time series (first samples)")
    axes[1].legend()
    fig.tight_layout()
    out = path.replace(".npz", "_analysis.png")
    fig.savefig(out, dpi=130)
    print(f"Saved {out}")


def estimate_lag_samples(raw, filtered, max_lag=200):
    """Lag (in samples) maximizing cross-correlation of the detrended signals."""
    r = raw.astype(np.float64) - raw.mean()
    f = filtered.astype(np.float64) - filtered.mean()
    lags = np.arange(0, max_lag)
    best_lag, best_corr = 0, -np.inf
    for lag in lags:
        corr = np.dot(r[: len(r) - lag], f[lag:])
        if corr > best_corr:
            best_corr, best_lag = corr, int(lag)
    return best_lag


def analyze_dynamic(path):
    data = np.load(path, allow_pickle=False)
    raw = data["raw"]
    filtered = data["filtered"]
    window, trim, mode = data["config"]
    interval_us = int(data["meta_interval_us"])
    dt = interval_us * 1e-6
    print(f"\n=== Dynamic swing: {os.path.basename(path)} "
          f"(hardware config: window={window}, trim={trim}, mode={MODE_NAMES[int(mode)]}) ===")

    # Hardware group delay is ~window/2 XADC samples (2.2 us each) = ~70 us for
    # window 63, far below one recording interval; measured lag should be 0-1.
    lag = estimate_lag_samples(raw, filtered)
    print(f"Measured filtered-vs-raw lag: {lag} recording samples ({lag * interval_us} us). "
          f"Expected ~0 (hardware group delay ~{int(window) // 2 * 2.2:.0f} us).")

    print(f"Raw glitches (>|{GLITCH_THRESHOLD_12BIT}| 12-bit LSB per step): {glitch_count(raw)}; "
          f"hardware filtered: {glitch_count(filtered)}")

    # Offline replay of alternative configurations on the same excitation.
    # IMPORTANT: replays run on the subsampled recording (one sample per
    # interval_us), so their effective time window is interval_us/2.2us times
    # longer than the hardware's. Use them to compare noise/outlier behavior
    # between configurations, not to judge absolute lag (the hardware lag is
    # the direct raw-vs-filtered measurement above).
    print(f"\n{'configuration':24s} {'residual std vs replayed avg':>28s} {'deriv std [codes/s]':>20s}")
    smooth_ref = filter_stream(raw, 63, 0, 2).astype(np.float64)  # pure average as smooth reference
    replays = {}
    for label, w, tr, m in REPLAY_CONFIGS:
        replay = filter_stream(raw, w, tr, m).astype(np.float64)
        replays[label] = replay
        resid = (replay - smooth_ref)[100:]  # skip fill-in transient
        deriv = np.diff(replay) / dt
        print(f"{label:24s} {resid.std():28.2f} {deriv.std():20.0f}")
    deriv_hw = np.diff(filtered.astype(np.float64)) / dt
    deriv_raw = np.diff(raw.astype(np.float64)) / dt
    print(f"{'hardware filtered':24s} {'-':>28s} {deriv_hw.std():20.0f}")
    print(f"{'raw':24s} {'-':>28s} {deriv_raw.std():20.0f}")
    print("(derivative std mixes true motion and noise; compare configurations, not absolute values)")

    # Plots: full swing, zoom near the region of fastest motion, derivative.
    t = np.arange(len(raw)) * dt
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    axes[0].plot(t, raw, label="raw", alpha=0.5, linewidth=0.6)
    axes[0].plot(t, filtered, label=f"hardware ({MODE_NAMES[int(mode)]} w={window} t={trim})", linewidth=0.8)
    axes[0].set_title("Full recording")
    axes[0].set_xlabel("time [s]")
    axes[0].legend()

    speed = np.abs(np.diff(smooth_ref))
    center = int(np.argmax(speed))
    lo, hi = max(0, center - 300), min(len(raw), center + 300)
    axes[1].plot(t[lo:hi], raw[lo:hi], ".", label="raw", alpha=0.5, markersize=3)
    axes[1].plot(t[lo:hi], filtered[lo:hi], label="hardware filtered", linewidth=1.2)
    for label in ("median63_old_design", "trimmed63_t7_default"):
        axes[1].plot(t[lo:hi], replays[label][lo:hi], label=f"replay {label}", linewidth=0.9, alpha=0.8)
    axes[1].set_title("Zoom at fastest motion (filter tracking)")
    axes[1].set_xlabel("time [s]")
    axes[1].legend()

    axes[2].plot(t[1:], deriv_raw, label="raw derivative", alpha=0.4, linewidth=0.5)
    axes[2].plot(t[1:], deriv_hw, label="hardware filtered derivative", linewidth=0.7)
    axes[2].set_title("Angular derivative (noise amplification check)")
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylabel("codes/s (16-bit domain)")
    axes[2].legend()
    fig.tight_layout()
    out = path.replace(".npz", "_analysis.png")
    fig.savefig(out, dpi=130)
    print(f"Saved {out}")


def rail_episodes(dz_age, interval_us, merge_gap_ms=25.0):
    """Group rail contacts into episodes using dz_age.

    dz_age counts XADC samples (~2.2 us) since the last rail contact, saturating
    at 0xFFFF. A recorded sample belongs to an active episode if the hardware
    saw a rail contact within the last recording interval (age * 2.2us <=
    interval_us) — this also catches contacts BETWEEN recorded samples.

    Episodes separated by less than merge_gap_ms are merged: during a crossing
    the electrical slew between the rails produces a ~1.5 ms contact gap, and
    the pole traverses the ~17 deg gap in tens of ms; both belong to one
    physical dead-zone visit.

    Returns a list of (start_idx, end_idx) recorded-sample ranges.
    """
    contact = dz_age.astype(np.int64) * 2.2 <= interval_us
    episodes = []
    i = 0
    n = len(contact)
    while i < n:
        if contact[i]:
            j = i
            while j < n and contact[j]:
                j += 1
            episodes.append((i, j))
            i = j
        else:
            i += 1

    gap_samples = int(merge_gap_ms * 1e3 / interval_us)
    merged = []
    for ep in episodes:
        if merged and ep[0] - merged[-1][1] <= gap_samples:
            merged[-1] = (merged[-1][0], ep[1])
        else:
            merged.append(ep)
    return merged


def analyze_deadzone(path):
    data = np.load(path, allow_pickle=False)
    raw = data["raw"].astype(np.int64)
    filtered = data["filtered"].astype(np.int64)
    dz_window = data["dz_window"].astype(np.int64)
    dz_status = data["dz_status"].astype(np.int64)
    dz_age = data["dz_age"].astype(np.int64)
    window, trim, mode = data["config"]
    before = data["counters_before"]
    after = data["counters_after"]
    interval_us = int(data["meta_interval_us"])
    maneuver = str(data["meta_maneuver"])
    dt = interval_us * 1e-6
    t = np.arange(len(raw)) * dt

    print(f"\n=== Dead zone ({maneuver}): {os.path.basename(path)} ===")
    low_hits = int(after[3] - before[3])
    high_hits = int(after[4] - before[4])
    print(f"Hardware rail hits during recording (XADC samples @ ~2.2 us): "
          f"low={low_hits}, high={high_hits}")

    episodes = rail_episodes(dz_age, interval_us)
    print(f"Rail episodes (grouped by dz_age): {len(episodes)}")

    def track_side(idx_range):
        """Side of the on-track signal in idx_range: 'high', 'low' or None."""
        seg = raw[idx_range]
        seg = seg[(seg > 20 * 16) & (seg < 4090 * 16)]  # on-track samples only
        if len(seg) < 3:
            return None
        return "high" if seg.mean() > 2048 * 16 else "low"

    for k, (a, b) in enumerate(episodes):
        seg_status = dz_status[a:b]
        sides = []
        if np.any(seg_status & 1):
            sides.append("low")
        if np.any(seg_status & 2):
            sides.append("high")
        # NOTE: rails touched do NOT discriminate crossing vs turnaround: once
        # the wiper leaves the track the floating input decays to the low rail
        # electrically, whichever way the pole moves afterwards. Classify by
        # the on-track side BEFORE vs AFTER the episode instead.
        margin = max(3, int(10e-3 / dt))  # ~10 ms of context on each side
        pre = track_side(slice(max(0, a - margin), a))
        post = track_side(slice(b, min(len(raw), b + margin)))
        if pre is None or post is None:
            kind = "unclassified (episode at recording edge)"
        elif pre == post:
            kind = f"TURNAROUND (entered and left on {pre} side)"
        else:
            kind = f"CROSSING ({pre} -> {post})"
        max_contam = int(dz_window[a:b].max()) if b > a else 0
        print(f"  episode {k}: t={a * dt:.2f}-{b * dt:.2f}s ({(b - a) * dt * 1e3:.0f} ms), "
              f"rails touched={'+'.join(sides) if sides else 'between-sample only'}, {kind}, "
              f"max window contamination {max_contam}/{int(window)}")

    # Consistency: recorded dz_status must match thresholds applied to recorded raw
    # (thresholds from goniometer_zynq.h: 20*16 low, 4090*16 high).
    exp_status = ((raw <= 20 * 16).astype(np.int64) * 1) | ((raw >= 4090 * 16).astype(np.int64) * 2)
    mism = int(np.count_nonzero(exp_status != dz_status))
    print(f"dz_status vs thresholds on recorded raw: {mism} mismatches (expect 0)")

    # Detection lead: how much earlier does dz_window rise above 0 than the
    # moment the recorded raw actually jumps to the other side.
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    axes[0].plot(t, raw / 16.0, ".", ms=2, alpha=0.6, label="raw (12-bit)")
    axes[0].plot(t, filtered / 16.0, lw=0.9, label="hardware filtered")
    axes[0].set_ylabel("ADC code")
    axes[0].set_title(f"{maneuver} — raw/filtered with hardware dead-zone tracking")
    axes[0].legend(fontsize=8)

    axes[1].plot(t, dz_window, lw=0.9, label="dz_window (near-rail samples in window)")
    axes[1].plot(t, dz_status * int(window) / 2, lw=0.6, alpha=0.6,
                 label="dz_status (scaled: low=1, high=2)")
    axes[1].set_ylabel("samples / flag")
    axes[1].legend(fontsize=8)

    age_ms = np.minimum(dz_age * 2.2e-3, 50.0)
    axes[2].plot(t, age_ms, lw=0.9, label="dz_age (ms since rail contact, clipped at 50)")
    axes[2].set_xlabel("time [s]")
    axes[2].set_ylabel("ms")
    axes[2].legend(fontsize=8)
    fig.tight_layout()
    out = path.replace(".npz", "_analysis.png")
    fig.savefig(out, dpi=130)
    print(f"Saved {out}")


def analyze_firmware(path):
    """Validate the firmware dead-zone handling from a streamed-state recording.

    Pass criteria:
      1. invalid_steps pulses at least once (hardware flag reaches the PC).
      2. During flagged episodes the reported angle never jumps by more than
         a small fraction of full scale between consecutive polls — the freeze
         must glide it through the zone instead of flipping to the far side.
      3. angleD during episodes stays within the range seen on clean polls
         (derivative is held, not recomputed across the gap).
    """
    data = np.load(path, allow_pickle=False)
    angle = data["angle"].astype(np.float64)
    angleD = data["angleD"].astype(np.float64)
    invalid = data["invalid_steps"].astype(np.int64)
    chip_time = data["chip_time"].astype(np.float64)
    angle_360 = float(data["meta_angle_360_adc"])
    t = chip_time - chip_time[0]

    def wrap_diff(d):
        d = np.where(d > angle_360 / 2, d - angle_360, d)
        d = np.where(d <= -angle_360 / 2, d + angle_360, d)
        return d

    steps = wrap_diff(np.diff(angle))
    flagged = invalid > 0
    n_episodes = int(np.count_nonzero(np.diff(flagged.astype(np.int8)) == 1)) + int(flagged[0])

    print(f"\n=== Firmware dead-zone handling: {os.path.basename(path)} ===")
    print(f"State messages: {len(angle)} over {t[-1]:.1f} s; "
          f"flagged polls: {int(flagged.sum())} in ~{n_episodes} episodes")

    ok = True
    if not np.any(flagged):
        print("FAIL: invalid_steps never pulsed — no dead-zone contact seen by firmware")
        ok = False
    else:
        # Classify each step by where it sits relative to flagged polls:
        # - inside:   both endpoints flagged -> pure extrapolation, must be smooth
        # - boundary: exactly one endpoint flagged -> entry, or the exit resync
        #   step where extrapolation drift is corrected; larger is legitimate,
        #   but a flip to the far side (~angle_360/2) is not
        # - clean:    neither endpoint flagged (with one-poll margin) -> baseline
        inside = flagged[:-1] & flagged[1:]
        boundary = flagged[:-1] ^ flagged[1:]
        dilated = np.convolve(flagged.astype(np.int8), np.ones(3, dtype=np.int8), mode="same") > 0
        clean = ~dilated[:-1] & ~dilated[1:]

        max_inside = float(np.abs(steps[inside]).max()) if np.any(inside) else 0.0
        max_boundary = float(np.abs(steps[boundary]).max()) if np.any(boundary) else 0.0
        max_clean = float(np.abs(steps[clean]).max()) if np.any(clean) else 0.0
        inside_limit = angle_360 / 8    # extrapolation is linear, steps stay small
        boundary_limit = angle_360 / 3  # resync correction ok, half-turn flip not
        print(f"max |angle step|: inside episodes {max_inside:.0f} (limit {inside_limit:.0f}), "
              f"at boundaries {max_boundary:.0f} (limit {boundary_limit:.0f}), "
              f"clean {max_clean:.0f} ADC units "
              f"(flip-to-far-side would be ~{angle_360 / 2:.0f})")
        if max_inside > inside_limit:
            print("FAIL: angle jumped inside an episode — freeze did not hold")
            ok = False
        if max_boundary > boundary_limit:
            print("FAIL: half-turn flip at an episode boundary — freeze released into garbage")
            ok = False

        clean_d = np.abs(angleD[:-1][clean]) if np.any(clean) else np.array([0.0])
        ep_d = np.abs(angleD[flagged])
        d_limit = max(1.5 * clean_d.max(), 1e-9)
        print(f"max |angleD| inside episodes: {ep_d.max():.1f} (clean max: {clean_d.max():.1f})")
        if ep_d.max() > d_limit:
            print("FAIL: angleD spiked during an episode — derivative crossed the gap")
            ok = False

    print("FIRMWARE TEST PASSED" if ok else "FIRMWARE TEST FAILED")

    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    axes[0].plot(t, angle, lw=0.8)
    for a, b in _flag_spans(t, flagged):
        axes[0].axvspan(a, b, color="red", alpha=0.15)
    axes[0].set_ylabel("angle [ADC units]")
    axes[0].set_title("Streamed state during dead-zone swing (red: firmware flagged contamination)")
    axes[1].plot(t, angleD, lw=0.8)
    for a, b in _flag_spans(t, flagged):
        axes[1].axvspan(a, b, color="red", alpha=0.15)
    axes[1].set_ylabel("angleD [ADC/poll]")
    axes[2].step(t, invalid, lw=0.8, where="post")
    axes[2].set_ylabel("invalid_steps")
    axes[2].set_xlabel("time [s]")
    fig.tight_layout()
    out = path.replace(".npz", "_analysis.png")
    fig.savefig(out, dpi=130)
    print(f"Saved {out}")
    return ok


def _flag_spans(t, flagged):
    spans = []
    i = 0
    n = len(flagged)
    while i < n:
        if flagged[i]:
            j = i
            while j < n and flagged[j]:
                j += 1
            spans.append((t[i], t[min(j, n - 1)]))
            i = j
        else:
            i += 1
    return spans


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="*", help="Specific .npz recordings; defaults to everything in output/.")
    args = parser.parse_args()

    files = args.files or sorted(glob.glob(os.path.join(OUTPUT_DIR, "*.npz")))
    if not files:
        print(f"No recordings found in {OUTPUT_DIR}. Run run_filter_experiment.py first.")
        return 1

    for path in files:
        name = os.path.basename(path)
        if name.startswith("static_sweep"):
            analyze_static(path)
        elif name.startswith("dynamic_swing"):
            analyze_dynamic(path)
        elif name.startswith("deadzone_"):
            analyze_deadzone(path)
        elif name.startswith("firmware_swing"):
            analyze_firmware(path)
        else:
            print(f"Skipping unrecognized file {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
