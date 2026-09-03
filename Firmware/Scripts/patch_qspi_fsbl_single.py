#!/usr/bin/env python3
"""Force the generated Zynq FSBL to read the 16 MiB QSPI in 1-bit mode."""

from pathlib import Path
import sys


if len(sys.argv) != 2:
    raise SystemExit(f"usage: {Path(sys.argv[0]).name} <zynq_fsbl/qspi.c>")

path = Path(sys.argv[1])
source = path.read_text()

old = """\
\t\t\tcase QSPI_BUSWIDTH_FOUR:
\t\t\t\t{
\t\t\t\t\tfsbl_printf(DEBUG_INFO,"QSPI is in 4-bit mode\\r\\n");
\t\t\t\t\tConfigCmd = SINGLE_QSPI_CONFIG_FAST_QUAD_READ;
\t\t\t\t}
\t\t\t\tbreak;
"""
new = """\
\t\t\tcase QSPI_BUSWIDTH_FOUR:
\t\t\t\t{
\t\t\t\t\t/*
\t\t\t\t\t * The Zybo S25FL128S may have CR1[1] QUAD clear.
\t\t\t\t\t * A 4-bit linear read then returns 0x888888xx and
\t\t\t\t\t * stalls the FSBL before it can load the bitstream.
\t\t\t\t\t */
\t\t\t\t\tfsbl_printf(DEBUG_INFO,"QSPI: forcing reliable 1-bit linear reads\\r\\n");
\t\t\t\t\tConfigCmd = SINGLE_QSPI_CONFIG_FAST_READ;
\t\t\t\t}
\t\t\t\tbreak;
"""

if new in source:
    print(f"{path}: 1-bit QSPI FSBL patch already present")
elif old in source:
    path.write_text(source.replace(old, new, 1))
    print(f"{path}: patched 16 MiB single-flash reads to 1-bit mode")
else:
    raise SystemExit(
        f"{path}: expected Vitis 2020.1 QSPI block not found; refusing to patch"
    )
