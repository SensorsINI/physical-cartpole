"""Phase 1: compute Secloc skip rate from CPP experiment recordings.

Compare CSV-derived skip % with terminal status after each rig run.

Usage (from repo root):
  python tests/analyze_secloc_cpp_csv.py path/to/CPP_pid__....csv
  python tests/analyze_secloc_cpp_csv.py --latest
  python tests/analyze_secloc_cpp_csv.py --latest 3

Recordings live under Driver/ExperimentRecordings/ (gitignored).
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

import pandas as pd

REPO_ROOT = Path(__file__).resolve().parents[1]
RECORDINGS_DIR = REPO_ROOT / "Driver" / "ExperimentRecordings"
HEADER_RE = re.compile(r"^#\s*([^:]+):\s*(.*)$")


def parse_cpp_header(path: Path) -> dict[str, str]:
    meta: dict[str, str] = {}
    with path.open(newline="") as handle:
        for line in handle:
            if not line.startswith("#"):
                break
            match = HEADER_RE.match(line.strip())
            if match:
                meta[match.group(1).strip()] = match.group(2).strip()
    return meta


def load_cpp_dataframe(path: Path) -> pd.DataFrame:
    # float_precision='round_trip' recovers the exact float64 bits the driver
    # logged; the default parser can be off by 1 ulp, which flips Secloc gate
    # decisions that sit exactly on the log_base threshold.
    return pd.read_csv(path, comment="#", float_precision="round_trip")


def skip_rate_from_csv(path: Path) -> dict:
    meta = parse_cpp_header(path)
    df = load_cpp_dataframe(path)
    if "secloc_skipped_update" not in df.columns:
        raise KeyError(
            f"{path.name}: column 'secloc_skipped_update' not found "
            f"(USE_SECLOC=False or old recording?)"
        )
    skipped = df["secloc_skipped_update"].astype(float)
    applied = 1.0 - skipped
    n = len(skipped)
    return {
        "path": path,
        "rows": n,
        "skip_pct": 100.0 * skipped.mean(),
        "apply_pct": 100.0 * applied.mean(),
        "updates": int(applied.sum()),
        "meta": meta,
    }


def format_report(result: dict) -> str:
    meta = result["meta"]
    lines = [
        f"File: {result['path'].name}",
        f"  rows: {result['rows']}",
        f"  skip rate: {result['skip_pct']:.2f}%  "
        f"({result['rows'] - result['updates']} skipped / {result['rows']} polls)",
        f"  apply rate: {result['apply_pct']:.2f}%  ({result['updates']} updates)",
    ]
    for key in (
        "Secloc gate",
        "Secloc log_base",
        "Secloc ref_period",
        "Secloc dead_ang",
        "Secloc dead_pos",
        "IO CPU affinity",
        "Control CPU affinity",
    ):
        if key in meta:
            lines.append(f"  {key}: {meta[key]}")
    return "\n".join(lines)


def latest_recordings(limit: int) -> list[Path]:
    files = sorted(RECORDINGS_DIR.glob("CPP_*.csv"), key=lambda p: p.stat().st_mtime)
    return files[-limit:]


def print_phase1_checklist() -> None:
    print(
        """
Phase 1 checklist (pid + USE_SECLOC, 5 ms polling)
------------------------------------------------
1. Driver/globals.py: CONTROLLER_NAME='pid', USE_SECLOC=True
2. Edit config_secloc.yml log_base for each run (file reloads on save):
     Run A: log_base: 1.00
     Run B: log_base: 1.05
     Run C: log_base: 1.10
3. python Driver/control.py  -> calibrate -> start control -> record ~30 s -> stop
4. Analyze each CSV:
     python tests/analyze_secloc_cpp_csv.py --latest
5. Compare CSV skip % with terminal line:
     "Secloc skipped X% of controller updates ..."
"""
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Analyze Secloc skip rate in CPP CSV")
    parser.add_argument("csv_files", nargs="*", help="CPP recording path(s)")
    parser.add_argument(
        "--latest",
        nargs="?",
        const=1,
        type=int,
        metavar="N",
        help="Analyze the N most recent CPP recordings in ExperimentRecordings",
    )
    parser.add_argument(
        "--checklist",
        action="store_true",
        help="Print Phase 1 run checklist",
    )
    args = parser.parse_args(argv)

    if args.checklist:
        print_phase1_checklist()
        return 0

    paths: list[Path] = [Path(p) for p in args.csv_files]
    if args.latest is not None:
        if not RECORDINGS_DIR.is_dir():
            print(f"No recordings directory: {RECORDINGS_DIR}", file=sys.stderr)
            return 1
        paths.extend(latest_recordings(args.latest))

    if not paths:
        print_phase1_checklist()
        parser.print_help()
        return 1

    exit_code = 0
    for path in paths:
        if not path.is_file():
            print(f"Missing file: {path}", file=sys.stderr)
            exit_code = 1
            continue
        try:
            result = skip_rate_from_csv(path)
            print(format_report(result))
            print()
        except Exception as exc:
            print(f"Error reading {path}: {exc}", file=sys.stderr)
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
