#!/usr/bin/env bash
set -euo pipefail

# ====== USER CONFIG ======

VIVADO_PROJECT_NAME="CartpoleDriverZynq_AXIS"
PLATFORM_POSTFIX="test"

# =========================

VIVADO_BIN="/tools/Xilinx/Vivado/2020.1/bin"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VITIS_WORKSPACE="${SCRIPT_DIR}/../Firmware/VitisProjects"
VIVADO_PROJECTS_ROOT="${SCRIPT_DIR}/VivadoProjects"

DATE_YYYYMMDD="$(date +%Y%m%d)"
PLATFORM_NAME="${VIVADO_PROJECT_NAME}_${DATE_YYYYMMDD}_${PLATFORM_POSTFIX}"
PROJECT_DIR="${VIVADO_PROJECTS_ROOT}/${VIVADO_PROJECT_NAME}"
XPR_PATH="${PROJECT_DIR}/${VIVADO_PROJECT_NAME}.xpr"
PLATFORM_OUT_DIR="${VITIS_WORKSPACE}/${PLATFORM_NAME}"
XSA_PATH="${PLATFORM_OUT_DIR}/${PLATFORM_NAME}.xsa"

export PATH="${VIVADO_BIN}:$PATH"

# -------- Early debug of resolved paths --------
echo "DEBUG PATHS:"
echo "  SCRIPT_DIR=${SCRIPT_DIR}"
echo "  VIVADO_PROJECT_NAME=${VIVADO_PROJECT_NAME}"
echo "  VIVADO_PROJECTS_ROOT=${VIVADO_PROJECTS_ROOT}"
echo "  PROJECT_DIR=${PROJECT_DIR}"
echo "  XPR_PATH=${XPR_PATH}"
echo "  PLATFORM_NAME=${PLATFORM_NAME}"
echo "  PLATFORM_OUT_DIR=${PLATFORM_OUT_DIR}"
echo "  XSA_PATH=${XSA_PATH}"
echo "  EXPECTED_IMPL_DIR=${PROJECT_DIR}/${VIVADO_PROJECT_NAME}.runs/impl_1"

# Check if impl_1 directory and bitstream exist
if [ -d "${PROJECT_DIR}/${VIVADO_PROJECT_NAME}.runs/impl_1" ]; then
    echo "  impl_1 directory: EXISTS"
    if [ -f "${PROJECT_DIR}/${VIVADO_PROJECT_NAME}.runs/impl_1"/*.bit ]; then
        echo "  Bitstream: EXISTS"
    else
        echo "  Bitstream: NOT FOUND"
    fi
else
    echo "  impl_1 directory: NOT FOUND"
fi

echo

if ! command -v xsct >/dev/null 2>&1; then
	if [ -x "/tools/Xilinx/Vitis/2020.1/bin/xsct" ]; then
		export PATH="/tools/Xilinx/Vitis/2020.1/bin:$PATH"
	fi
fi

if ! command -v vivado >/dev/null 2>&1; then
	echo "ERROR: vivado not found in PATH. Check VIVADO_BIN: ${VIVADO_BIN}" >&2
	exit 1
fi
if ! command -v xsct >/dev/null 2>&1; then
	echo "ERROR: xsct (Vitis) not found in PATH. Please add your Vitis bin to PATH." >&2
	exit 1
fi
if [ ! -f "${XPR_PATH}" ]; then
	echo "ERROR: Vivado project not found: ${XPR_PATH}" >&2
	exit 1
fi

mkdir -p "${PLATFORM_OUT_DIR}"

cleanup_vivado_aux() {
  if [ -d "${PROJECT_DIR}" ]; then
    (
      cd "${PROJECT_DIR}" && \
      rm -f vivado.jou vivado.log || true && \
      rm -rf .Xil || true
    )
  fi
}
trap cleanup_vivado_aux EXIT

# -------- Export hardware (fixed, with bitstream) via Vivado --------
vivado_tcl="$(mktemp -t export_hw_XXXXXXXX.tcl)"
cat > "${vivado_tcl}" <<'TCL'
open_project $::env(XPR)

# Verify impl_1 run exists
if {[llength [get_runs impl_1]] == 0} {
  puts "ERROR: impl_1 run not found in project."
  exit 1
}

# Open implemented design
if {[catch {open_run impl_1} err]} {
  puts "WARN: Could not open impl_1 checkpoint: $err"
}

# Locate bitstream
set run_dir [get_property DIRECTORY [get_runs impl_1]]
set impl_dir $run_dir
if {[file tail $run_dir] ne "impl_1"} {
  set impl_dir [file join $run_dir "impl_1"]
}

# Search for bitstream inside the implementation directory
set bitfiles [glob -nocomplain -directory $impl_dir "*.bit"]

if {[llength $bitfiles] == 0} {
  puts "INFO: Bitstream not found in $impl_dir. Forcing write_bitstream..."
  launch_runs impl_1 -to_step write_bitstream -jobs 4 -force
  wait_on_run impl_1
  # Recompute directories in case Vivado adjusted paths
  set run_dir [get_property DIRECTORY [get_runs impl_1]]
  set impl_dir $run_dir
  if {[file tail $run_dir] ne "impl_1"} {
    set impl_dir [file join $run_dir "impl_1"]
  }
  set bitfiles [glob -nocomplain -directory $impl_dir "*.bit"]
  if {[llength $bitfiles] == 0} {
    puts "ERROR: Bitstream still missing after generation."
    exit 1
  }
}

set bitfile [lindex $bitfiles 0]
puts "Found bitstream: $bitfile"

# Now export XSA with the bitstream
file mkdir $::env(OUTPUT_DIR)
set xsa_path [file join $::env(OUTPUT_DIR) "${::env(PLATFORM_NAME)}.xsa"]
puts "Writing XSA with bitstream: $xsa_path"
if {[catch {write_hw_platform -fixed -include_bit -force -file $xsa_path} err]} {
  puts "ERROR: write_hw_platform failed: $err"
  exit 1
}

puts "Hardware export complete."
exit 0


TCL

echo "Exporting hardware with bitstream from: ${XPR_PATH}"
pushd "${PROJECT_DIR}" >/dev/null
XPR="${XPR_PATH}" OUTPUT_DIR="${PLATFORM_OUT_DIR}" PLATFORM_NAME="${PLATFORM_NAME}" \
	vivado -mode batch -source "${vivado_tcl}" -log vivado.log -journal vivado.jou | cat
popd >/dev/null || true
rm -f "${vivado_tcl}"

if [ ! -f "${XSA_PATH}" ]; then
  echo "ERROR: Bitstream/XSA not available. Skipping Vitis platform creation." >&2
  exit 1
fi

# -------- Create hardware platform in Vitis workspace via XSCT --------
xsct_tcl="$(mktemp -t create_platform_XXXXXXXX.tcl)"
cat > "${xsct_tcl}" <<'TCL'
set ws $::env(VITIS_WS)
set xsa $::env(XSA)
set pname $::env(PLATFORM_NAME)

if {![file exists $xsa]} {
  puts "ERROR: XSA not found: $xsa"
  exit 1
}

file mkdir $ws
setws $ws

set existing_platforms [platform list]
if {[lsearch -exact $existing_platforms $pname] >= 0} {
  puts "Platform '$pname' already exists. Activating and cleaning..."
  platform active $pname
  catch {platform clean}
} else {
  puts "Creating platform '$pname' from: $xsa"
  platform create -name $pname -hw $xsa -out $ws
  platform active $pname
}

foreach d [domain list] {
  if {[string match "default*" $d]} {
    catch {domain delete $d}
  }
}

set proc "ps7_cortexa9_0"
# Note: platform list processors is not available in Vitis 2020.1
# Using default processor for Zynq-7000

if {[lsearch -exact [domain list] "standalone_domain"] < 0} {
  domain create -name standalone_domain -proc $proc -os standalone
}
domain active standalone_domain

platform write
if {[catch {platform generate -domains standalone_domain} err]} {
  puts "ERROR: platform generate failed: $err"
  exit 1
}

puts "Platform '$pname' ready in workspace: $ws"
exit 0
TCL

echo "Creating Vitis platform '${PLATFORM_NAME}' in workspace: ${VITIS_WORKSPACE}"
VITIS_WS="${VITIS_WORKSPACE}" XSA="${XSA_PATH}" PLATFORM_NAME="${PLATFORM_NAME}" \
	xsct "${xsct_tcl}" | cat
rm -f "${xsct_tcl}"

if [ -d "${PROJECT_DIR}" ]; then
  (
    cd "${PROJECT_DIR}" && \
    rm -f vivado.jou vivado.log && \
    rm -rf .Xil || true
  )
fi

echo "Done. Platform: ${PLATFORM_NAME}"
exit 0
