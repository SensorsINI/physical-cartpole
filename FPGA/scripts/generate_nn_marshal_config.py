#!/usr/bin/env python3
"""
Generate nn_marshal_config.h for the nn_marshal HLS IP from a deployed network folder.

Reads:
  - myproject.vhd (or any *project.vhd): AXI port widths and output count
  - hls4ml_config.yml (optional): ap_fixed input/output precisions
  - norm_vectors/*.csv (or explicit --norm-csv-dir): normalization coefficients

Usage:
  python generate_nn_marshal_config.py \\
      --vhd-dir FPGA/NeuralNetworks/hls4ml_dense_1out_8_07_07_2026 \\
      --norm-csv-dir path/to/norm_vectors

Called automatically at the end of Convert_Network_With_hls4ml.py when possible.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from datetime import datetime, timezone
from pathlib import Path


AP_FIXED_RE = re.compile(r"ap_fixed<(\d+),(\d+)>")


def _parse_ap_fixed(text: str) -> tuple[int, int]:
    match = AP_FIXED_RE.search(text)
    if not match:
        raise ValueError(f"Could not parse ap_fixed precision in: {text!r}")
    return int(match.group(1)), int(match.group(2))


def _read_yaml_scalar_block(path: Path, key_path: list[str]) -> str | None:
    """Minimal YAML reader for nested scalar values (no PyYAML dependency)."""
    if not path.is_file():
        return None
    lines = path.read_text(encoding="utf-8").splitlines()
    depth = 0
    targets = key_path[:]
    for line in lines:
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(line) - len(line.lstrip(" "))
        level = indent // 2
        if level < len(targets) and stripped.startswith(f"{targets[level]}:"):
            if level == len(targets) - 1:
                value = stripped.split(":", 1)[1].strip()
                return value.strip("'\"")
    return None


def _load_hls4ml_precisions(vhd_dir: Path) -> tuple[tuple[int, int], tuple[int, int]]:
    for name in ("hls4ml_config.yml", "config_hls.yml"):
        cfg = vhd_dir / name
        if not cfg.is_file():
            continue
        text = cfg.read_text(encoding="utf-8")
        input_match = re.search(
            r"input_1:\s*\n(?:\s+.+\n)*?\s+result:\s*(\S+)", text, re.MULTILINE
        )
        output_match = re.search(
            r"layers_\d+_linear:\s*\n(?:\s+.+\n)*?\s+result:\s*(\S+)", text
        )
        if not output_match:
            output_match = re.search(
                r"linear_ap_fixed[^\n]*linear_config\d+_s", text
            )
        if input_match:
            in_prec = _parse_ap_fixed(input_match.group(1))
            # Last layers_*_linear result in file is usually the network output type.
            outputs = re.findall(
                r"layers_\d+_linear:\s*\n(?:\s+.+\n)*?\s+result:\s*(\S+)",
                text,
            )
            if outputs:
                out_prec = _parse_ap_fixed(outputs[-1])
                return in_prec, out_prec
    raise FileNotFoundError(
        f"No hls4ml_config.yml with input_1 precision found in {vhd_dir}"
    )


def _parse_vhd_geometry(vhd_dir: Path) -> tuple[int, int, int]:
    vhd_files = sorted(vhd_dir.glob("myproject.vhd"))
    if not vhd_files:
        vhd_files = sorted(vhd_dir.glob("*project.vhd"))
    if not vhd_files:
        raise FileNotFoundError(f"No myproject.vhd in {vhd_dir}")

    text = vhd_files[0].read_text(encoding="utf-8")
    input_m = re.search(
        r"input_1_V\s*:\s*IN\s+STD_LOGIC_VECTOR\s*\(\s*(\d+)\s+downto\s+0\s*\)",
        text,
        re.IGNORECASE,
    )
    if not input_m:
        raise ValueError("Could not find input_1_V port in myproject.vhd")

    output_ports = re.findall(
        r"layer\d+_out_\d+_V\s*:\s*OUT\s+STD_LOGIC_VECTOR\s*\(\s*(\d+)\s+downto\s+0\s*\)",
        text,
        re.IGNORECASE,
    )
    if not output_ports:
        raise ValueError("Could not find layer*_out_*_V ports in myproject.vhd")

    input_total_bits = int(input_m.group(1)) + 1
    output_count = len(output_ports)
    output_bits_each = int(output_ports[0]) + 1
    return input_total_bits, output_bits_each, output_count


def _read_norm_csv(path: Path) -> list[float]:
    with path.open(newline="", encoding="utf-8") as handle:
        row = next(csv.reader(handle))
    return [float(x) for x in row]


def _load_norm_vectors(norm_dir: Path) -> tuple[list[float], list[float], list[float], list[float]]:
    files = {
        "a": norm_dir / "normalization_vec_a.csv",
        "b": norm_dir / "normalization_vec_b.csv",
        "A": norm_dir / "denormalization_vec_A.csv",
        "B": norm_dir / "denormalization_vec_B.csv",
    }
    missing = [name for name, path in files.items() if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            f"Missing normalization CSV(s) in {norm_dir}: {', '.join(missing)}"
        )
    return (
        _read_norm_csv(files["a"]),
        _read_norm_csv(files["b"]),
        _read_norm_csv(files["A"]),
        _read_norm_csv(files["B"]),
    )


def _format_float_array(name: str, values: list[float]) -> str:
    lines = [f"static const float {name}[{len(values)}] = {{"]
    row = "    "
    for idx, value in enumerate(values):
        row += f"{value:.8f}f"
        if idx != len(values) - 1:
            row += ", "
        if len(row) > 72:
            lines.append(row.rstrip())
            row = "    "
    if row.strip():
        lines.append(row.rstrip())
    lines.append("};")
    return "\n".join(lines)


def generate_nn_marshal_config(
    vhd_dir: Path,
    norm_csv_dir: Path,
    *,
    input_wire_order: list[str] | None = None,
    output_path: Path | None = None,
) -> Path:
    vhd_dir = vhd_dir.resolve()
    norm_csv_dir = norm_csv_dir.resolve()
    output_path = output_path or (vhd_dir / "nn_marshal_config.h")

    (in_total, in_int), (out_total, out_int) = _load_hls4ml_precisions(vhd_dir)
    input_bits_vhd, output_bits_each, output_count = _parse_vhd_geometry(vhd_dir)

    if input_bits_vhd % in_total != 0:
        raise ValueError(
            f"Input bus width {input_bits_vhd} is not a multiple of "
            f"ap_fixed<{in_total},{in_int}> width"
        )
    input_count = input_bits_vhd // in_total

    if output_bits_each != out_total:
        raise ValueError(
            f"Output width mismatch: hls4ml ap_fixed<{out_total},{out_int}> vs "
            f"{output_bits_each}-bit VHDL port"
        )

    norm_a, norm_b, denorm_a, denorm_b = _load_norm_vectors(norm_csv_dir)
    if len(norm_a) != input_count:
        raise ValueError(
            f"normalization_vec_a has {len(norm_a)} entries, expected {input_count}"
        )
    if len(denorm_a) != output_count:
        raise ValueError(
            f"denormalization_vec_A has {len(denorm_a)} entries, expected {output_count}"
        )

    wire_order = input_wire_order or [
        "angleD", "angle_cos", "angle_sin", "position", "positionD",
        "target_equilibrium", "target_position",
    ]
    if len(wire_order) != input_count:
        wire_order = [f"input_{i}" for i in range(input_count)]

    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M UTC")
    body = f"""#ifndef NN_MARSHAL_CONFIG_H
#define NN_MARSHAL_CONFIG_H

/*
 * Auto-generated by FPGA/scripts/generate_nn_marshal_config.py
 * ({stamp})
 *
 * Per-network compile-time configuration for nn_marshal HLS. Re-synthesize
 * nn_marshal_hls when this file changes. controller_io_parameters.vhd must
 * match the input/output bit widths below.
 *
 * Source VHDL : {vhd_dir.name}/myproject.vhd
 * Norm vectors: {norm_csv_dir}
 */

#define NN_IO_INPUT_COUNT        {input_count}
#define NN_IO_OUTPUT_COUNT       {output_count}

#define NN_IO_INPUT_TOTAL_BITS   {in_total}
#define NN_IO_INPUT_INT_BITS     {in_int}

#define NN_IO_OUTPUT_TOTAL_BITS  {out_total}
#define NN_IO_OUTPUT_INT_BITS    {out_int}

#define NN_IO_INPUT_FRAC_BITS    (NN_IO_INPUT_TOTAL_BITS - NN_IO_INPUT_INT_BITS)
#define NN_IO_OUTPUT_FRAC_BITS   (NN_IO_OUTPUT_TOTAL_BITS - NN_IO_OUTPUT_INT_BITS)

/* Input wire order (physical floats on the gate<->marshal link): */
/* {", ".join(wire_order)} */

{_format_float_array("NN_NORM_A", norm_a)}

{_format_float_array("NN_NORM_B", norm_b)}

{_format_float_array("NN_DENORM_A", denorm_a)}

{_format_float_array("NN_DENORM_B", denorm_b)}

#endif /* NN_MARSHAL_CONFIG_H */
"""
    output_path.write_text(body, encoding="utf-8")
    return output_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vhd-dir", type=Path, required=True,
        help="Deployed network folder containing myproject.vhd and hls4ml_config.yml",
    )
    parser.add_argument(
        "--norm-csv-dir", type=Path, required=True,
        help="Directory with normalization_vec_*.csv and denormalization_vec_*.csv",
    )
    parser.add_argument(
        "--output", type=Path, default=None,
        help="Output header path (default: <vhd-dir>/nn_marshal_config.h)",
    )
    args = parser.parse_args(argv)

    out = generate_nn_marshal_config(
        args.vhd_dir, args.norm_csv_dir, output_path=args.output
    )
    print(f"Wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
